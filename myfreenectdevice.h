#ifndef MYFREENECTDEVICE_H
#define MYFREENECTDEVICE_H

#include <pthread.h>
#include <vector>

// OpenCV
#include "opencv2/core.hpp"

// Freenect
#include "libfreenect.hpp"

struct simpleMutex{
    simpleMutex() { pthread_mutex_init(&m_mutex, NULL); }
    void lock() { pthread_mutex_lock(&m_mutex); }
    void unlock() { pthread_mutex_unlock(&m_mutex); }
private:
    pthread_mutex_t m_mutex;
};

class MyFreenectDevice : public Freenect::FreenectDevice {
public:
    enum {GAMMA = 2048};

    // Constructor
    MyFreenectDevice(freenect_context *_ctx, int _index);

    // Do not call either of these functions directly even in child (for some reason)
    void VideoCallback(void *_rgb, uint32_t timestamp);
    void DepthCallback(void *_depth, uint32_t timestamp);

    // Video and depth functions
    bool getVideo(cv::Mat& output);
    bool getDepth(cv::Mat& output);

private:
    std::vector<uint8_t> m_buffer_depth;
    std::vector<uint8_t> m_buffer_rgb;
    std::vector<uint16_t> m_gamma;
    cv::Mat m_depthMat;
    cv::Mat m_rgbMat;
    cv::Mat m_ownMat;
    simpleMutex m_rgb_mutex;
    simpleMutex m_depth_mutex;
    bool m_new_rgb_frame;
    bool m_new_depth_frame;
};

#endif // MYFREENECTDEVICE_H
