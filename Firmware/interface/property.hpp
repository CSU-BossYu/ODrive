#ifndef ODRIVE_INTERFACE_PROPERTY_HPP
#define ODRIVE_INTERFACE_PROPERTY_HPP

#include <cstdint>
#include <new>
#include <optional>
#include <type_traits>
#include <utility>

template<typename T>
struct Property {
    Property(void* ctx, T(*getter)(void*), void(*setter)(void*, T))
        : ctx_(ctx), getter_(getter), setter_(setter) {}
    Property(T* ctx)
        : ctx_(ctx),
          getter_([](void* ctx) { return *static_cast<T*>(ctx); }),
          setter_([](void* ctx, T value) { *static_cast<T*>(ctx) = value; }) {}

    Property& operator*() { return *this; }
    Property* operator->() { return this; }
    T read() const { return getter_(ctx_); }
    T exchange(std::optional<T> value) const {
        T old_value = getter_(ctx_);
        if (value.has_value()) {
            setter_(ctx_, *value);
        }
        return old_value;
    }

    void* ctx_;
    T(*getter_)(void*);
    void(*setter_)(void*, T);
};

template<typename T>
struct Property<const T> {
    Property(void* ctx, T(*getter)(void*)) : ctx_(ctx), getter_(getter) {}
    Property(const T* ctx)
        : ctx_(const_cast<T*>(ctx)),
          getter_([](void* ctx) { return *static_cast<const T*>(ctx); }) {}

    Property& operator*() { return *this; }
    Property* operator->() { return this; }
    T read() const { return getter_(ctx_); }

    void* ctx_;
    T(*getter_)(void*);
};

#endif
