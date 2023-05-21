#ifndef LOOP_DETECTION_HPP_
#define LOOP_DETECTION_HPP_

#include "modular_slam/keyframe.hpp"

#include <memory>

namespace mslam
{
template <typename StateType>
class ILoopDetector
{
  public:
    virtual std::shared_ptr<Keyframe<StateType>> detectLoop() = 0;
};
} // namespace mslam

#endif /* LOOP_DETECTION_HPP_ */
