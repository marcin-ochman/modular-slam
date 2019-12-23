#ifndef LOOP_DETECTION_HPP_
#define LOOP_DETECTION_HPP_


namespace mslam
{
  class LoopDetection
  {
  public:
    virtual bool detectLoop() = 0;
  };
}

#endif /* LOOP_DETECTION_HPP_ */
