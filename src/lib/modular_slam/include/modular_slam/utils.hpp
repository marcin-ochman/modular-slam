#ifndef UTILS_HPP_
#define UTILS_HPP_

namespace mslam
{
namespace utils
{
template <typename...>
struct is_one_of
{
    static constexpr bool value = false;
};

template <typename F, typename S, typename... T>
struct is_one_of<F, S, T...>
{
    static constexpr bool value = std::is_same<F, S>::value || is_one_of<F, T...>::value;
};
template <typename F, typename S, typename... T>
using is_one_of_v = typename is_one_of<F, S, T...>::value;
} // namespace utils
} // namespace mslam

#endif /* UTILS_HPP_ */
