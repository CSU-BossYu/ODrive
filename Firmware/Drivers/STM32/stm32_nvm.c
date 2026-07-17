/*
* Flash-based Non-Volatile Memory (NVM)
* 
* This file supports storing and loading persistent configuration based on
* the STM32 builtin flash memory.
*
* The STM32F405xx has 12 flash sectors of heterogeneous size. We use the last
* two sectors for configuration data. These pages have a size of 128kB each.
* Setting any bit in these sectors to 0 is always possible, but setting them
* to 1 requires erasing the whole sector.
*
* We consider each sector as an array of 64-bit fields except the first N bytes, which we
* instead use as an allocation block. The allocation block is a compact bit-field (2 bit per entry)
* that keeps track of the state of each field (erased, invalid, valid).
*
* One sector is always considered the valid (read) sector and the other one is the
* target for the next write access: they can be considered to be ping-pong or double buffred.
*
* When writing a block of data, instead of always erasing the whole writable sector the
* new data is appended in the erased area. This presumably increases flash life span.
* The writable sector is only erased if there is not enough space for the new data.
*
* Every record has a sequence number, payload length and CRC32. On startup both
* sectors are checked and the newest complete record is selected. This makes
* the commit robust even if power is removed between validating the new record
* and invalidating the previous record.
*
*
* To write a new block of data atomically we first mark all associated fields
* as "invalid" (in the allocation table) then write the data and then mark the
* fields as "valid" (in the direction of increasing address).
*/

#include "stm32_nvm.h"

#include <stddef.h>
#include <string.h>

#if defined(STM32F405xx)

#include <stm32f405xx.h>
#include <stm32f4xx_hal.h>

// refer to page 75 of datasheet:
// http://www.st.com/content/ccc/resource/technical/document/reference_manual/3d/6d/5a/66/b4/99/40/d4/DM00031020.pdf/files/DM00031020.pdf/jcr:content/translations/en.DM00031020.pdf
#define FLASH_SECTOR_A FLASH_SECTOR_10
#define FLASH_SECTOR_A_BASE (const volatile uint8_t*)0x80C0000UL
#define FLASH_SECTOR_A_SIZE 0x20000UL
#define FLASH_SECTOR_B FLASH_SECTOR_11
#define FLASH_SECTOR_B_BASE (const volatile uint8_t*)0x80E0000UL
#define FLASH_SECTOR_B_SIZE 0x20000UL

#elif defined(STM32F722xx)

#include <stm32f722xx.h>
#include <stm32f7xx_hal.h>

// refer to page 68 of datasheet:
// https://www.st.com/resource/en/reference_manual/dm00305990-stm32f72xxx-and-stm32f73xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
#define FLASH_SECTOR_A FLASH_SECTOR_1
#define FLASH_SECTOR_A_BASE (const volatile uint8_t*)0x8004000UL
#define FLASH_SECTOR_A_SIZE 0x4000UL
#define FLASH_SECTOR_B FLASH_SECTOR_2
#define FLASH_SECTOR_B_BASE (const volatile uint8_t*)0x8008000UL
#define FLASH_SECTOR_B_SIZE 0x4000UL

#else
#error "unknown flash sector size"
#endif

typedef enum {
    VALID = 0,
    INVALID = 1,
    ERASED = 3
} field_state_t;

typedef struct {
    size_t index;               //!< next field to be written to (can be equal to n_data)
    const uint32_t sector_id;   //!< HAL ID of this sector
    const size_t n_data;        //!< number of 64-bit fields in this sector
    const size_t n_reserved;    //!< number of 64-bit fields in this sector that are reserved for the allocation table
    const volatile uint8_t* const alloc_table;
    const volatile uint64_t* const data;
} sector_t;

#define NVM_RECORD_MAGIC 0x324D564EUL /* "NVM2" in little-endian flash */

typedef struct {
    uint32_t magic;
    uint32_t sequence;
    uint32_t payload_length;
    uint32_t payload_crc32;
    uint32_t header_crc32;
    uint32_t reserved;
} nvm_record_header_t;

typedef struct {
    uint8_t valid;
    size_t start;
    size_t field_count;
    uint32_t sequence;
    uint32_t payload_length;
} record_candidate_t;

sector_t sectors[] = { {
    .sector_id = FLASH_SECTOR_A,
    .n_data = FLASH_SECTOR_A_SIZE >> 3,
    .n_reserved = (FLASH_SECTOR_A_SIZE >> 3) >> 5,
    .alloc_table = FLASH_SECTOR_A_BASE,
    .data = (uint64_t *)FLASH_SECTOR_A_BASE
}, {
    .sector_id = FLASH_SECTOR_B,
    .n_data = FLASH_SECTOR_B_SIZE >> 3,
    .n_reserved = (FLASH_SECTOR_B_SIZE >> 3) >> 5,
    .alloc_table = FLASH_SECTOR_B_BASE,
    .data = (uint64_t *)FLASH_SECTOR_B_BASE
}};

uint8_t read_sector_; // 0 or 1 to indicate which sector to read from and which to write to
size_t n_staging_area_; // number of 64-bit values that were reserved using NVM_start_write
size_t n_valid_; // number of 64-bit fields that can be read
size_t read_start_;
size_t read_length_;
size_t staging_payload_length_;
uint32_t current_sequence_;
uint8_t read_has_header_;

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t length) {
    while (length--) {
        crc ^= *data++;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
        }
    }
    return crc;
}

static uint32_t crc32(const uint8_t *data, size_t length) {
    return crc32_update(0xFFFFFFFFUL, data, length) ^ 0xFFFFFFFFUL;
}

static uint32_t header_crc32(const nvm_record_header_t *header) {
    return crc32((const uint8_t *)header, offsetof(nvm_record_header_t, header_crc32));
}

static const uint32_t FLASH_ERR_FLAGS =
#if defined(FLASH_FLAG_EOP)
        FLASH_FLAG_EOP |
#endif
#if defined(FLASH_FLAG_OPERR)
        FLASH_FLAG_OPERR |
#endif
#if defined(FLASH_FLAG_WRPERR)
        FLASH_FLAG_WRPERR |
#endif
#if defined(FLASH_FLAG_PGAERR)
        FLASH_FLAG_PGAERR |
#endif
#if defined(FLASH_FLAG_PGSERR)
        FLASH_FLAG_PGSERR |
#endif
#if defined(FLASH_FLAG_PGPERR)
        FLASH_FLAG_PGPERR |
#endif
        0;

static void HAL_FLASH_ClearError() {
    __HAL_FLASH_CLEAR_FLAG(FLASH_ERR_FLAGS);
}


// @brief Erases a flash sector. This sets all bits in the sector to 1.
// The sector's current index is reset to the minimum value (n_reserved).
// @returns 0 on success or a non-zero error code otherwise
int erase(sector_t *sector) {
    FLASH_EraseInitTypeDef erase_struct = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
#if defined(FLASH_OPTCR_nDBANK)
        .Banks = 0, // only used for mass erase
#endif
        .Sector = sector->sector_id,
        .NbSectors = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();
    uint32_t sector_error;
    if (HAL_FLASHEx_Erase(&erase_struct, &sector_error) != HAL_OK)
        goto fail;
    sector->index = sector->n_reserved;

    HAL_FLASH_Lock();
    return 0;
fail:
    HAL_FLASH_Lock();
    //printf("erase failed: %u \r\n", HAL_FLASH_GetError());
    return HAL_FLASH_GetError(); // non-zero
}


// @brief Writes states into the allocation table.
// The write operation goes in the direction of increasing indices.
// @param state: 11: erased, 10: writing, 00: valid data
// @returns 0 on success or a non-zero error code otherwise
int set_allocation_state(sector_t *sector, size_t index, size_t count, field_state_t state) {
    if (index < sector->n_reserved)
        return -1;
    if (index + count > sector->n_data)
        return -1;

    // expand state to state for 4 values
    const uint8_t states = (state << 0) | (state << 2) | (state << 4) | (state << 6);
    
    // handle unaligned start
    uint8_t mask = ~(0xff << ((index & 0x3) << 1));
    count += index & 0x3;
    index -= index & 0x3;

    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();

    // write states
    for (; count >= 4; count -= 4, index += 4) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uintptr_t)&sector->alloc_table[index >> 2], states | mask) != HAL_OK)
            goto fail;
        mask = 0;
    }

    // handle unaligned end
    if (count) {
        mask |= ~(0xff >> ((4 - count) << 1));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uintptr_t)&sector->alloc_table[index >> 2], states | mask) != HAL_OK)
            goto fail;
    }
    
    HAL_FLASH_Lock();
    return 0;
fail:
    HAL_FLASH_Lock();
    return HAL_FLASH_GetError(); // non-zero
}

// @brief Reads the allocation table from behind to determine how many fields match the
// reference state.
// @param sector: The sector on which to perform the search
// @param max_index: The maximum index that should be considered
// @param ref_state: The reference state
// @param state: Set to the first encountered state that is unequal to ref_state.
//               Set to ref_state if all encountered states are equal to ref_state.
// @returns The smallest index that points to a field with ref_state.
//          This value is at least sector->n_reserved and at most max_index.
size_t scan_allocation_table(sector_t *sector, size_t max_index, field_state_t ref_state, field_state_t *state) {
    const uint8_t ref_states = (ref_state << 0) | (ref_state << 2) | (ref_state << 4) | (ref_state << 6);
    size_t index = (((max_index + 3) >> 2) << 2); // start at the max index but round up to a multiple of 4
    size_t ignore = index - max_index;
    uint8_t states = ref_states;

    //printf("scan from %08x to %08x for %02x\r\n", index, sector->n_reserved, ref_states); osDelay(5);

    // read 4 states at a time
    for (; index >= (sector->n_reserved + 4); index -= 4) {
        states = sector->alloc_table[(index - 1) >> 2];
        if (ignore) { // ignore the upper 1, 2 or 3 states if max_index was unaligned
            uint8_t ignore_mask = ~(0xff >> (ignore << 1));
            states = (states & ~ignore_mask) | (ref_states & ignore_mask);
            ignore = 0;
        }
        if (states != ref_states)
            break;
    }

    // once we encounterd a byte with any state mismatch determine which of the 4 states it is
    for (; ((states >> 6) == (ref_states & 0x3)) && (index > sector->n_reserved); index--) {
        states <<= 2;
    }
    
    *state = states >> 6;
    //printf("(it's %02x)\r\n", index); osDelay(5);
    return index;
}

static record_candidate_t inspect_latest_record(sector_t *sector, field_state_t tail_state) {
    record_candidate_t result = {0};
    if (tail_state != VALID) {
        return result;
    }

    field_state_t preceding_state;
    const size_t start = scan_allocation_table(sector, sector->index, VALID, &preceding_state);
    const size_t field_count = sector->index - start;
    if ((field_count << 3) < sizeof(nvm_record_header_t)) {
        return result;
    }

    const nvm_record_header_t *header = (const nvm_record_header_t *)&sector->data[start];
    if ((header->magic != NVM_RECORD_MAGIC) ||
        (header->reserved != 0) ||
        (header->header_crc32 != header_crc32(header))) {
        return result;
    }

    const size_t available_payload_bytes = (field_count << 3) - sizeof(nvm_record_header_t);
    if (header->payload_length > available_payload_bytes) {
        return result;
    }
    const size_t record_length = sizeof(nvm_record_header_t) + header->payload_length;
    if (((record_length + 7) >> 3) != field_count) {
        return result;
    }

    const uint8_t *payload = (const uint8_t *)header + sizeof(*header);
    if (header->payload_crc32 != crc32(payload, header->payload_length)) {
        return result;
    }

    result.valid = 1;
    result.start = start;
    result.field_count = field_count;
    result.sequence = header->sequence;
    result.payload_length = header->payload_length;
    return result;
}

static int sequence_is_newer(uint32_t lhs, uint32_t rhs) {
    return (int32_t)(lhs - rhs) > 0;
}

// Loads the head of the NVM data.
// If this function fails subsequent calls to NVM functions (other than NVM_init or NVM_erase)
// cause undefined behavior.
// @returns 0 on success or a non-zero error code otherwise
int NVM_init(void) {
    field_state_t sector0_state, sector1_state;
    sectors[0].index = scan_allocation_table(&sectors[0], sectors[0].n_data,
                ERASED, &sector0_state);
    sectors[1].index = scan_allocation_table(&sectors[1], sectors[1].n_data,
                ERASED, &sector1_state);
    //printf("sector states: %02x, %02x\r\n", sector0_state, sector1_state); osDelay(5);

    const record_candidate_t candidate0 = inspect_latest_record(&sectors[0], sector0_state);
    const record_candidate_t candidate1 = inspect_latest_record(&sectors[1], sector1_state);

    read_sector_ = 0;
    if (candidate1.valid && (!candidate0.valid || sequence_is_newer(candidate1.sequence, candidate0.sequence))) {
        read_sector_ = 1;
    }

    const record_candidate_t *selected = read_sector_ ? &candidate1 : &candidate0;
    if (selected->valid) {
        read_start_ = selected->start;
        n_valid_ = selected->field_count;
        read_length_ = selected->payload_length;
        current_sequence_ = selected->sequence;
        read_has_header_ = 1;
    } else {
        // One-time compatibility path for records written by the previous
        // headerless format. ConfigManager still validates its CRC16/version;
        // the next successful save migrates the data to NVM2.
        if ((sector0_state == VALID) || (sector1_state == VALID)) {
            read_sector_ = (sector1_state == VALID) ? 1 : 0;
            sector_t *legacy_sector = &sectors[read_sector_];
            field_state_t preceding_state;
            read_start_ = scan_allocation_table(legacy_sector, legacy_sector->index,
                VALID, &preceding_state);
            n_valid_ = legacy_sector->index - read_start_;
            read_length_ = n_valid_ << 3;
            read_has_header_ = 0;
        } else {
            read_start_ = sectors[read_sector_].n_reserved;
            n_valid_ = 0;
            read_length_ = 0;
            read_has_header_ = 1;
        }
        current_sequence_ = 0;
    }

    n_staging_area_ = 0;
    staging_payload_length_ = 0;

    int status = 0;
    /*// bring non-valid sectors into a known state
    this is not absolutely required
    if (sector0_state != VALID)
        status |= erase(&sectors[0]);
    if (sector1_state != VALID)
        status |= erase(&sectors[1]);
    */
    return status;
}

// @brief Erases all data in the NVM.
//
// If this function fails subsequent calls to NVM functions (other than NVM_init or NVM_erase)
// cause undefined behavior.
// Caution: this function may take a long time (like 1 second)
//
// @returns 0 on success or a non-zero error code otherwise
int NVM_erase(void) {
    read_sector_ = 0;
    sectors[0].index = sectors[0].n_reserved;
    sectors[1].index = sectors[1].n_reserved;
    read_start_ = sectors[0].n_reserved;
    read_length_ = 0;
    n_valid_ = 0;
    n_staging_area_ = 0;
    staging_payload_length_ = 0;
    current_sequence_ = 0;
    read_has_header_ = 1;

    int state = 0;
    state |= erase(&sectors[0]);
    state |= erase(&sectors[1]);
    return state;
}

// @brief Returns the maximum number of bytes that can be read using NVM_read.
// This holds until NVM_commit is called.
size_t NVM_get_max_read_length(void) {
    return read_length_;
}

// @brief Returns the maximum length (in bytes) that can passed to NVM_start_write.
// This holds until NVM_commit is called.
size_t NVM_get_max_write_length(void) {
    sector_t *target = &sectors[1 - read_sector_];
    const size_t capacity = (target->n_data - target->n_reserved) << 3;
    return capacity > sizeof(nvm_record_header_t) ? capacity - sizeof(nvm_record_header_t) : 0;
}

// @brief Reads from the latest committed block in the non-volatile memory.
// The function either succeeds or leaves the provided buffer unmodified.
// @param offset: offset in bytes (0 meaning the beginning of the valid area)
// @param data: buffer to write to
// @param length: length in bytes (if (offset + length) is out of range, the function fails)
// @returns 0 on success or a non-zero error code otherwise
int NVM_read(size_t offset, uint8_t *data, size_t length) {
    if ((offset > read_length_) || (length > read_length_ - offset))
        return -1;
    sector_t *read_sector = &sectors[read_sector_];
    const uint8_t *src_ptr = (const uint8_t *)&read_sector->data[read_start_] + offset;
    if (read_has_header_) {
        src_ptr += sizeof(nvm_record_header_t);
    }
    memcpy(data, src_ptr, length);
    return 0;
}

// @brief Starts an atomic write operation.
//
// The most recent valid NVM data is not modified or invalidated until NVM_commit is called.
// The length must be at most equal to the size indicated by NVM_get_max_write_length().
//
// @param length: Length of the staging block that should be created
int NVM_start_write(size_t length) {
    int status = 0;
    sector_t *target = &sectors[1 - read_sector_];

    const size_t payload_length = length;
    length = (sizeof(nvm_record_header_t) + length + 7) >> 3; // round to 64-bit fields
    if (length > target->n_data - target->n_reserved)
        return -1;

    // make room for the new data
    if (length > target->n_data - target->index)
        if ((status = erase(target)))
            return status;

    // invalidate the fields we're about to write
    status = set_allocation_state(target, target->index, length, INVALID);
    if (status)
        return status;

    n_staging_area_ = length;
    staging_payload_length_ = payload_length;

    const nvm_record_header_t header = {
        .magic = NVM_RECORD_MAGIC,
        .sequence = current_sequence_ + 1,
        .payload_length = payload_length,
        .payload_crc32 = 0xFFFFFFFFUL,
        .header_crc32 = 0xFFFFFFFFUL,
        .reserved = 0
    };
    status = NVM_write((size_t)-sizeof(nvm_record_header_t), (uint8_t *)&header, sizeof(header));
    if (status) {
        n_staging_area_ = 0;
        staging_payload_length_ = 0;
    }
    return status;
}

// @brief Writes to the current data block that was opened with NVM_start_write.
//
// The operation fails if (offset + length) is larger than the length passed to NVM_start_write.
// The most recent valid NVM data is not modified or invalidated until NVM_commit is called.
// Warning: Writing different data to the same area multiple times during a single transaction
// will cause data corruption.
//
// @param offset: The offset in bytes, 0 being the beginning of the staging block.
// @param data: Pointer to the data that should be written
// @param length: Data length in bytes
int NVM_write(size_t offset, uint8_t *data, size_t length) {
    uintptr_t physical_offset;
    if (offset == (size_t)-sizeof(nvm_record_header_t)) {
        physical_offset = 0;
    } else {
        if ((offset > staging_payload_length_) || (length > staging_payload_length_ - offset))
            return -1;
        physical_offset = sizeof(nvm_record_header_t) + offset;
    }
    if (physical_offset + length > (n_staging_area_ << 3))
        return -1;
    sector_t *target = &sectors[1 - read_sector_];
    offset = physical_offset;

    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();

    // handle unaligned start
    for (; (offset & 0x3) && length; ++data, ++offset, --length)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                ((uintptr_t)&target->data[target->index]) + offset, *data) != HAL_OK)
            goto fail;

    // write 32-bit values (64-bit doesn't work)
    for (; length >= 4; data += 4, offset += 4, length -=4)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                ((uintptr_t)&target->data[target->index]) + offset, *(uint32_t*)data) != HAL_OK)
            goto fail;

    // handle unaligned end
    for (; length; ++data, ++offset, --length)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                ((uintptr_t)&target->data[target->index]) + offset, *data) != HAL_OK)
            goto fail;

    HAL_FLASH_Lock();
    return 0;
fail:
    HAL_FLASH_Lock();
    return HAL_FLASH_GetError(); // non-zero
}

// @brief Commits the new data to NVM atomically.
int NVM_commit(void) {
    sector_t *read_sector = &sectors[read_sector_];
    sector_t *write_sector = &sectors[1 - read_sector_];

    if (!n_staging_area_) {
        return -1;
    }

    nvm_record_header_t *header = (nvm_record_header_t *)&write_sector->data[write_sector->index];
    const uint8_t *payload = (const uint8_t *)header + sizeof(*header);
    const uint32_t payload_crc = crc32(payload, staging_payload_length_);

    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();
    int status = 0;
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uintptr_t)&header->payload_crc32, payload_crc) != HAL_OK) {
        status = HAL_FLASH_GetError();
    } else {
        const uint32_t header_crc = header_crc32(header);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uintptr_t)&header->header_crc32, header_crc) != HAL_OK) {
            status = HAL_FLASH_GetError();
        }
    }
    HAL_FLASH_Lock();
    if (status || (header->payload_crc32 != payload_crc) ||
        (header->header_crc32 != header_crc32(header))) {
        return status ? status : -1;
    }

    // The allocation state is the final atomic commit marker.
    status = set_allocation_state(write_sector, write_sector->index, n_staging_area_, VALID);
    if (status)
        return status;

    read_start_ = write_sector->index;
    write_sector->index += n_staging_area_;
    n_valid_ = n_staging_area_;
    read_length_ = staging_payload_length_;
    current_sequence_ = header->sequence;
    read_has_header_ = 1;
    n_staging_area_ = 0;
    staging_payload_length_ = 0;
    read_sector_ = 1 - read_sector_;

    // invalidate the other sector
    if (read_sector->index < read_sector->n_data) {
        status = set_allocation_state(read_sector, read_sector->index, 1, INVALID);
        read_sector->index += 1;
    } else {
        status = erase(read_sector);
    }

    // Once the new record is valid, failure to invalidate the old record is
    // harmless: sequence-based startup selection deterministically picks this
    // record. Treat invalidation as best-effort housekeeping.
    return 0;
}
