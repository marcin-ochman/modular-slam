#ifndef MODULAR_SLAM_CONCEPT_UTILS_HPP_
#define MODULAR_SLAM_CONCEPT_UTILS_HPP_

namespace modular_slam
{

template <typename Derived, template <typename...> typename TemplateBase>
concept IsDerivedFromTemplate = requires(Derived& d) { []<typename... Args>(TemplateBase<Args...>&) {}(d); };
} // namespace modular_slam

#endif // MODULAR_SLAM_CONCEPT_UTILS_HPP_
