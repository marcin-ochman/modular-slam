#ifndef MODULAR_SLAM_SLOT_HPP_
#define MODULAR_SLAM_SLOT_HPP_

#include <cstdint>
#include <string_view>

namespace mslam
{

using SlotId = std::uint64_t;

constexpr SlotId fnv1a64(std::string_view text)
{
    SlotId hash = 14695981039346656037ull;

    for(char c : text)
    {
        hash ^= static_cast<unsigned char>(c);
        hash *= 1099511628211ull;
    }

    return hash;
}

struct TypeToken
{
    const void* id = nullptr;
    std::string_view debugName = {};

    friend constexpr bool operator==(TypeToken a, TypeToken b) { return a.id == b.id; }
    friend constexpr bool operator!=(TypeToken a, TypeToken b) { return !(a == b); }
};

template <typename T>
inline constexpr char kTypeTokenAnchor = 0;

template <typename T>
constexpr TypeToken typeToken(std::string_view debugName = {})
{
    return TypeToken{.id = &kTypeTokenAnchor<T>, .debugName = debugName};
}

template <typename T>
class Slot
{
  public:
    using ValueType = T;

    constexpr explicit Slot(std::string_view name, std::string_view debugTypeName = {})
        : mId(fnv1a64(name)), mName(name), mType(typeToken<T>(debugTypeName))
    {
    }

    [[nodiscard]] constexpr SlotId id() const { return mId; }
    [[nodiscard]] constexpr std::string_view name() const { return mName; }
    [[nodiscard]] constexpr TypeToken type() const { return mType; }

  private:
    SlotId mId;
    std::string_view mName;
    TypeToken mType;
};

} // namespace mslam

#endif // MODULAR_SLAM_SLOT_HPP_
