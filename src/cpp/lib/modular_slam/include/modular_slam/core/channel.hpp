#ifndef MODULAR_SLAM_CORE_CHANNEL_HPP
#define MODULAR_SLAM_CORE_CHANNEL_HPP

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <queue>
#include <utility>

namespace mslam
{
template <typename T>
class Channel
{
  public:
    void push(T&& value)
    {
        std::unique_lock<std::mutex> lock(mMutex);
        if(mQueue.size() >= mMaxSize)
        {
            mQueue.pop();
        }

        mQueue.push(std::move(value));
        mCv.notify_one();
    }

    T pop()
    {
        std::unique_lock<std::mutex> lock(mMutex);
        mCv.wait(lock, [this]() { return !mQueue.empty(); });
        T value = std::move(mQueue.front());
        mQueue.pop();

        return value;
    }

  private:
    std::queue<T> mQueue;
    std::mutex mMutex;
    std::condition_variable mCv;
    const std::size_t mMaxSize = 100;
};
} // namespace mslam
#endif // MODULAR_SLAM_CORE_CHANNEL_HPP
