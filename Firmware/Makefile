# Convenience wrapper around the CMake firmware build.

BUILD_DIR ?= build
BOARD_VERSION ?= v3.6-56V
CONFIG_DEBUG ?= false
CONFIG_STRICT ?= false
CONFIG_USE_LTO ?= false
CONFIG_ENABLE_DISASM ?= false

CMAKE_CONFIGURE_ARGS = \
	-DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
	-DBOARD_VERSION=$(BOARD_VERSION) \
	-DCONFIG_DEBUG=$(CONFIG_DEBUG) \
	-DCONFIG_STRICT=$(CONFIG_STRICT) \
	-DCONFIG_USE_LTO=$(CONFIG_USE_LTO) \
	-DCONFIG_ENABLE_DISASM=$(CONFIG_ENABLE_DISASM)

ifneq ($(ARM_COMPILER_PATH),)
CMAKE_CONFIGURE_ARGS += -DARM_COMPILER_PATH=$(ARM_COMPILER_PATH)
endif

FIRMWARE = $(BUILD_DIR)/ODriveFirmware.elf
PROGRAMMER_CMD=$(if $(value PROGRAMMER),-c 'hla_serial $(PROGRAMMER)',)

ifeq ($(OS),Windows_NT)
  CMAKE_GENERATOR ?= Unix Makefiles
else
  CMAKE_GENERATOR ?= Unix Makefiles
endif

ifeq ($(findstring v3.,$(BOARD_VERSION)),v3.)
  OPENOCD := openocd -f interface/stlink-v2.cfg $(PROGRAMMER_CMD) -f target/stm32f4x.cfg -c init
  GDB := arm-none-eabi-gdb --ex 'target extended-remote | openocd -f "interface/stlink-v2.cfg" -f "target/stm32f4x.cfg" -c "gdb_port pipe; log_output openocd.log"' --ex 'monitor reset halt'
else
  $(error CMake wrapper currently supports v3 board versions only)
endif

$(info board version: $(BOARD_VERSION))

configure:
	cmake -S . -B $(BUILD_DIR) -G "$(CMAKE_GENERATOR)" $(CMAKE_CONFIGURE_ARGS)

all: configure
	cmake --build $(BUILD_DIR) --target ODriveFirmware

clean:
	cmake --build $(BUILD_DIR) --target clean

distclean:
	cmake -E rm -rf $(BUILD_DIR)

flash-stlink2: all
	$(OPENOCD) \
		-c 'reset halt' \
		-c 'flash write_image erase $(FIRMWARE)' \
		-c 'reset run' \
		-c exit

gdb-stlink2:
	$(GDB) $(FIRMWARE)

erase-stlink2:
	$(OPENOCD) -c 'reset halt' -c 'flash erase_sector 0 0 last' -c exit

unlock-stlink2:
	$(OPENOCD) -c 'reset halt' -c 'stm32f2x unlock 0'

flash-bmp: all
	arm-none-eabi-gdb --ex 'target extended-remote $(BMP_PORT)' \
		--ex 'monitor swdp_scan' \
		--ex 'attach 1' \
		--ex 'load' \
		--ex 'detach' \
		--ex 'quit' \
		$(FIRMWARE)

gdb-bmp: all
	arm-none-eabi-gdb --ex 'target extended-remote /dev/stlink' \
		--ex 'monitor swdp_scan' \
		--ex 'attach 1' \
		--ex 'load' $(FIRMWARE)

flash: flash-stlink2
gdb: gdb-stlink2
erase: erase-stlink2
unlock: unlock-stlink2

.PHONY: configure all clean distclean flash-stlink2 gdb-stlink2 erase-stlink2 unlock-stlink2
.PHONY: flash-bmp gdb-bmp flash gdb erase unlock
