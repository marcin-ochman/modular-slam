#ifndef MODULAR_SLAM_SLOT_STORE_HPP
#define MODULAR_SLAM_SLOT_STORE_HPP

#include <unordered_map>

#include "modular_slam/core/expected.hpp"
#include "modular_slam/core/slot.hpp"

namespace mslam
{
class SlotStore
{
  public:
    SlotStore() = default;
    SlotStore(const SlotStore&) = delete;
    SlotStore& operator=(const SlotStore&) = delete;

    SlotStore(SlotStore&&) noexcept = default;
    SlotStore& operator=(SlotStore&&) noexcept = default;

    bool containsId(SlotId id) const { return mValues.find(id) != mValues.end(); }

    template <typename T>
    Status set(Slot<T> slot, T value)
    {
        auto status = validateOrRegister(slot);
        if(!status)
        {
            return status;
        }

        T* object = new(std::nothrow) T(std::move(value));
        if(!object)
        {
            return std::unexpected(Error::outOfMemory());
        }

        Entry entry;
        entry.type = slot.type();
        entry.ptr = object;
        entry.destroy = [](void* ptr) noexcept { delete static_cast<T*>(ptr); };

        mValues[slot.id()] = std::move(entry);

        return {};
    }

    template <typename T>
    ExpectedConstRef<T> get(Slot<T> slot) const
    {
        auto result = tryGet(slot);

        if(!result)
        {
            return std::unexpected(result.error());
        }

        if(!result->has_value())
        {
            return std::unexpected(Error::missingSlot(slot.name()));
        }

        return std::cref(*result->value());
    }

    template <typename T>
    ExpectedRef<T> getMut(Slot<T> slot)
    {
        auto result = tryGetMut(slot);

        if(!result)
        {
            return std::unexpected(result.error());
        }

        if(!result->has_value())
        {
            return std::unexpected(Error::missingSlot(slot.name()));
        }

        return std::ref(*result->value());
    }

    template <typename T>
    Expected<std::optional<const T*>> tryGet(Slot<T> slot) const
    {
        auto it = mValues.find(slot.id());

        if(it == mValues.end())
        {
            return std::optional<const T*>{};
        }

        const Entry& entry = it->second;

        if(entry.type != slot.type())
        {
            return std::unexpected(Error::typeMismatch(slot.name()));
        }

        return std::optional<const T*>{static_cast<const T*>(entry.ptr)};
    }

    template <typename T>
    Expected<std::optional<T*>> tryGetMut(Slot<T> slot)
    {
        auto it = mValues.find(slot.id());

        if(it == mValues.end())
        {
            return std::optional<T*>{};
        }

        Entry& entry = it->second;

        if(entry.type != slot.type())
        {
            return std::unexpected(Error::typeMismatch(slot.name()));
        }

        return std::optional<T*>{static_cast<T*>(entry.ptr)};
    }

    template <typename T>
    Expected<bool> contains(Slot<T> slot) const
    {
        auto value = tryGet(slot);

        if(!value)
        {
            return std::unexpected(value.error());
        }

        return value->has_value();
    }

    Status mergeFrom(SlotStore&& other)
    {
        for(auto& [slotId, meta] : other.mRegistry)
        {
            auto it = mRegistry.find(slotId);

            if(it != mRegistry.end() && it->second.type != meta.type)
            {
                return std::unexpected(Error::slotConflict(meta.name));
            }

            mRegistry.emplace(slotId, std::move(meta));
        }

        for(auto& [slotId, entry] : other.mValues)
        {
            mValues[slotId] = std::move(entry);
        }

        return {};
    }

    std::vector<std::string> keys() const
    {
        std::vector<std::string> result;
        result.reserve(mRegistry.size());

        for(const auto& [_, meta] : mRegistry)
        {
            result.push_back(meta.name);
        }

        return result;
    }

  private:
    struct SlotMeta
    {
        std::string name;
        TypeToken type;
    };

    struct Entry
    {
        TypeToken type{};
        void* ptr = nullptr;
        void (*destroy)(void*) noexcept = nullptr;

        Entry() = default;

        Entry(const Entry&) = delete;
        Entry& operator=(const Entry&) = delete;

        Entry(Entry&& other) noexcept : type(other.type), ptr(other.ptr), destroy(other.destroy)
        {
            other.ptr = nullptr;
            other.destroy = nullptr;
        }

        Entry& operator=(Entry&& other) noexcept
        {
            if(this == &other)
            {
                return *this;
            }

            reset();

            type = other.type;
            ptr = other.ptr;
            destroy = other.destroy;

            other.ptr = nullptr;
            other.destroy = nullptr;

            return *this;
        }

        ~Entry() { reset(); }

        void reset() noexcept
        {
            if(ptr && destroy)
            {
                destroy(ptr);
            }

            ptr = nullptr;
            destroy = nullptr;
        }
    };

    template <typename T>
    Status validateOrRegister(Slot<T> slot)
    {
        auto it = mRegistry.find(slot.id());

        if(it == mRegistry.end())
        {
            mRegistry[slot.id()] = SlotMeta{.name = std::string(slot.name()), .type = slot.type()};
            return {};
        }

        if(it->second.type != slot.type())
        {
            return std::unexpected(Error::slotConflict(slot.name()));
        }

        return {};
    }

  private:
    std::unordered_map<SlotId, SlotMeta> mRegistry;
    std::unordered_map<SlotId, Entry> mValues;
};
} // namespace mslam

#endif // MODULAR_SLAM_SLOT_STORE_HPP
