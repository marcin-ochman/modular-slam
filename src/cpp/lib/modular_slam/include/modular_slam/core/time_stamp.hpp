#ifndef MODULAR_SLAM_TIME_STAMP_HPP_
#define MODULAR_SLAM_TIME_STAMP_HPP_

#include <chrono>
#include <compare>

namespace mslam
{
class Timestamp
{
  public:
    explicit Timestamp(int64_t ns) : mNanoseconds(ns) {}

    Timestamp() = default;
    Timestamp(const Timestamp&) = default;
    Timestamp& operator=(const Timestamp&) = default;
    Timestamp(Timestamp&&) = default;
    Timestamp& operator=(Timestamp&&) = default;

    static Timestamp now()
    {
        const auto now = std::chrono::steady_clock::now();
        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        return Timestamp(ns);
    }

    std::chrono::nanoseconds operator-(const Timestamp& other) const
    {
        return std::chrono::nanoseconds(this->mNanoseconds - other.mNanoseconds);
    }

    template <typename Rep, typename Period>
    Timestamp operator+(const std::chrono::duration<Rep, Period>& d) const
    {
        auto delta = std::chrono::duration_cast<std::chrono::nanoseconds>(d).count();
        return Timestamp(mNanoseconds + delta);
    }

    [[nodiscard]] int64_t fromEpoch() const { return mNanoseconds; }

    auto operator<=>(const Timestamp&) const = default;

  private:
    int64_t mNanoseconds{0};
};
} // namespace mslam

#endif // MODULAR_SLAM_TIME_STAMP_HPP_
