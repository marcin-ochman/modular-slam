#ifndef MODULAR_SLAM_CONCEPT_UTILS_HPP_
#define MODULAR_SLAM_CONCEPT_UTILS_HPP_

namespace mslam
{

template <typename Derived, template <typename...> typename TemplateBase>
concept IsDerivedFromTemplate = requires(Derived& d) { []<typename... Args>(TemplateBase<Args...>&) {}(d); };
} // namespace mslam

#endif // MODULAR_SLAM_CONCEPT_UTILS_HPP_
