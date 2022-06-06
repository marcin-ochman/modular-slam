#ifndef MODULAR_SLAM_HPP_
#define MODULAR_SLAM_HPP_

#include "modular_slam/loop_detection.hpp"
#include "modular_slam/parameters_handler.hpp"
#include "modular_slam/types.hpp"


#include <cstdint>
#include <vector>
#include <memory>
#include <any>


namespace mslam
{

class SlamComponent
{
public:
    SlamComponent(std::shared_ptr<ParametersHandlerInterface> handler): parametersHandler{handler} {}

    template <typename T>
    T getParameterAs(const std::string& name)
    {
        return std::any_cast<T>( parametersHandler->getParameter(name));
    }

    template <typename T>
    bool setParameterAs(const std::string& name, const T& value)
    {
       return parametersHandler->setParameter(name, std::make_any(value));
    }

    private:
        std::shared_ptr<ParametersHandlerInterface> parametersHandler;
};

} // namespace mslam

#endif /* MODULAR_SLAM_HPP_ */
