#include "myfreenectdevice.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video.hpp"

MyFreenectDevice::MyFreenectDevice(freenect_context *_ctx, int _index) :
    Freenect::FreenectDevice(_ctx, _index),
    m_buffer_depth(FREENECT_DEPTH_11BIT),
    m_buffer_rgb(FREENECT_VIDEO_RGB),
    m_gamma(GAMMA),
    m_depthMat(cv::Size(640,480),CV_16UC1),
    m_rgbMat(cv::Size(640,480), CV_8UC3, cv::Scalar(0)),
    m_ownMat(cv::Size(640,480), CV_8UC3, cv::Scalar(0)),
    m_new_rgb_frame(false), m_new_depth_frame(false)
{
    float v = 0;
    for(unsigned int i = 0; i < GAMMA; ++i){
        v = (float) i/GAMMA;
        v = std::pow(v, 3) * 6;
        m_gamma[i] = v * 6 * 256;
    }
}

void MyFreenectDevice::VideoCallback(void *_rgb, uint32_t timestamp){
    m_rgb_mutex.lock();
    uint8_t *rgb = static_cast<uint8_t*>(_rgb);
    m_rgbMat.data = rgb;
    m_new_rgb_frame = true;
    m_rgb_mutex.unlock();
}

void MyFreenectDevice::DepthCallback(void *_depth, uint32_t timestamp){
    m_depth_mutex.lock();
    uint16_t* depth = static_cast<uint16_t*>(_depth);
    m_depthMat.data = (uchar*) depth;
    m_new_depth_frame = true;
    m_depth_mutex.unlock();
}

bool MyFreenectDevice::getVideo(cv::Mat &output){
    m_rgb_mutex.lock();
    if(m_new_rgb_frame){
        cv::cvtColor(m_rgbMat, output, CV_RGB2BGR);
        m_new_rgb_frame = false;
        m_rgb_mutex.unlock();
        return true;
    }
    else{
        m_rgb_mutex.unlock();
        return false;
    }
}

bool MyFreenectDevice::getDepth(cv::Mat &output){
    m_depth_mutex.lock();
    if(m_new_depth_frame) {
        m_depthMat.copyTo(output);
        m_new_depth_frame = false;
        m_depth_mutex.unlock();
        return true;
    } else {
        m_depth_mutex.unlock();
        return false;
    }
}
