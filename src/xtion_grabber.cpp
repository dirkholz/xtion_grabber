// Grabber for the xtion kernel driver
// Author: Max Schwarz <max.schwarz@online.de>

#include "xtion_grabber.h"

#include <pluginlib/class_list_macros.h>

#include <boost/make_shared.hpp>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

#include <ros/node_handle.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/photo/photo.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#if HAVE_LIBYUV
#include <libyuv.h>
#endif

namespace xtion_grabber
{

inline uint32_t yuv(float y, float u, float v)
{
    int b = y + (u-128) / 0.493;
    int r = y + (v-128) / 0.877;
    int g = 1.7*y - 0.509*r - 0.194*b;

    if(r < 0)
        r = 0;
    if(r > 255)
        r = 255;
    if(g < 0)
        g = 0;
    if(g > 255)
        g = 255;
    if(b < 0)
        b = 0;
    if(b > 255)
        b = 255;

    return b | (g << 8) | (r << 16) | (0xFF << 24);
}

XtionGrabber::XtionGrabber()
    : m_lastColorSeq(-1)
    , m_lastDepthSeq(-2)
    , m_shouldExit(false)
    , m_cameraOn(true)
{
    m_color_pool.reset(new utils::Pool<sensor_msgs::Image>);
    m_depth_pool.reset(new utils::Pool<sensor_msgs::Image>);
    m_pointCloudPool.reset(new utils::Pool<sensor_msgs::PointCloud2>);
}

XtionGrabber::~XtionGrabber()
{
    m_shouldExit = true;
    m_cameraMux.unlock();
    m_thread.join();
    if(m_cameraOn)
    {
        stopColor();
        stopDepth();
        close(m_color_fd);
        close(m_depth_fd);
    }
}

bool XtionGrabber::toggleService(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    if(!m_cameraOn)
    {
        //Turn camera on
        ros::Time start = ros::Time::now();
        init();
        NODELET_INFO_STREAM("[" << m_deviceName << "] Camera stream started in: " << (ros::Time::now()-start).toSec() << " secs.");
        m_cameraOn = !m_cameraOn;
        m_cameraMux.unlock();
        res.success = true;
        res.message = "Camera on";
    }
    else
    {
        //Turn camera off
        m_cameraMux.lock();
        m_cameraOn = !m_cameraOn;
        if(!stopColor())
            NODELET_WARN("Could not stop color streaming");
        if(!stopDepth())
            NODELET_WARN("Could not stop depth streaming");

        close(m_color_fd);
        close(m_depth_fd);
        res.success = false;
        res.message = "Camera off";
    }
    return true;
}

bool XtionGrabber::setupDepth(const std::string& device)
{
    m_depth_fd = open(device.c_str(), O_RDONLY);
    if(m_depth_fd < 0)
    {
        NODELET_INFO("Could not open depth device");
        return false;
    }

    struct v4l2_requestbuffers reqbuf;
    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_USERPTR;
    reqbuf.count = NUM_BUFS;

    if(ioctl(m_depth_fd, VIDIOC_REQBUFS, &reqbuf) != 0)
    {
        NODELET_INFO("Could not request buffers");
        return false;
    }

    for(size_t i = 0; i < NUM_BUFS; ++i)
    {
        DepthBuffer* buffer = &m_depth_buffers[i];

        buffer->image = createDepthImage();

        memset(&buffer->buf, 0, sizeof(buffer->buf));
        buffer->buf.index = i;
        buffer->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer->buf.memory = V4L2_MEMORY_USERPTR;
        buffer->buf.m.userptr = (long unsigned int)buffer->image->data.data();
        buffer->buf.length = buffer->image->data.size();

        if(ioctl(m_depth_fd, VIDIOC_QBUF, &buffer->buf) != 0)
        {
            NODELET_INFO("Could not queue buffer");
            return false;
        }
    }

    return true;
}

bool XtionGrabber::stopDepth()
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return (ioctl(m_depth_fd, VIDIOC_STREAMOFF, &type) == 0);
}

bool XtionGrabber::startDepth()
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return (ioctl(m_depth_fd, VIDIOC_STREAMON, &type) == 0);
}

bool XtionGrabber::setupColor(const std::string& device)
{
    m_color_fd = open(device.c_str(), O_RDONLY);
    if(m_color_fd < 0)
    {
        NODELET_INFO("Could not open color device");
        return false;
    }

    struct v4l2_format fmt;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = m_colorWidth;
    fmt.fmt.pix.height = m_colorHeight;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;

    if(ioctl(m_color_fd, VIDIOC_S_FMT, &fmt) != 0)
    {
        NODELET_ERROR("Could not set image format");
        return false;
    }

    m_colorWidth = fmt.fmt.pix.width;
    m_colorHeight = fmt.fmt.pix.height;

    struct v4l2_streamparm parms;
    parms.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parms.parm.capture.timeperframe.numerator = 1;
    parms.parm.capture.timeperframe.denominator = 30;

    if(ioctl(m_color_fd, VIDIOC_S_PARM, &parms) != 0)
    {
        NODELET_ERROR("Could not set image interval");
        return false;
    }

    /* Enable flicker filter for 50 Hz */
    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = V4L2_CID_POWER_LINE_FREQUENCY;
    ctrl.value = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;

    if(ioctl(m_color_fd, VIDIOC_S_CTRL, &ctrl) != 0)
    {
        NODELET_ERROR("Could not set flicker control");
        return false;
    }

    struct v4l2_requestbuffers reqbuf;
    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_USERPTR;
    reqbuf.count = NUM_BUFS;

    if(ioctl(m_color_fd, VIDIOC_REQBUFS, &reqbuf) != 0)
    {
        NODELET_ERROR("Could not request buffers");
        return false;
    }

    for(size_t i = 0; i < NUM_BUFS; ++i)
    {
        ColorBuffer* buffer = &m_color_buffers[i];

        buffer->data.resize(m_colorWidth*m_colorHeight*2);

        memset(&buffer->buf, 0, sizeof(buffer->buf));
        buffer->buf.index = i;
        buffer->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer->buf.memory = V4L2_MEMORY_USERPTR;
        buffer->buf.m.userptr = (long unsigned int)buffer->data.data();
        buffer->buf.length = buffer->data.size();

        if(ioctl(m_color_fd, VIDIOC_QBUF, &buffer->buf) != 0)
        {
            NODELET_INFO("Could not queue buffer");
            return false;
        }
    }

    return true;
}

bool XtionGrabber::stopColor()
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return (ioctl(m_color_fd, VIDIOC_STREAMOFF, &type) == 0);
}

bool XtionGrabber::startColor()
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return (ioctl(m_color_fd, VIDIOC_STREAMON, &type) == 0);
}

void XtionGrabber::onInit()
{
    ros::NodeHandle& nh = getMTPrivateNodeHandle();

    if(!nh.getParam("depth_device", m_depthDevice)
            || !nh.getParam("color_device", m_colorDevice))
    {
        ROS_FATAL("depth_device and color_device parameters are mandatory!");
        throw std::runtime_error("depth_device and color_device parameters are mandatory!");
    }

    nh.param("depth_width", m_depthWidth, 640);
    nh.param("depth_height", m_depthHeight, 480);
    nh.param("color_width", m_colorWidth, 1280);
    nh.param("color_height", m_colorHeight, 1024);
    //bool initial_state;
    //nh.param("initial_state_on", initial_state, false);
    // tf prefix
    nh.param<std::string>("tf_prefix", m_tfprefix, "");

    m_deviceName = getPrivateNodeHandle().getNamespace();

    m_nodeName = ros::this_node::getName();

    m_pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("depth_registered/points", 1);

    ros::NodeHandle color_nh(getPrivateNodeHandle(), "rgb");
    m_color_it.reset(new image_transport::ImageTransport(color_nh));
    m_pub_color = m_color_it->advertiseCamera("image_raw", 1);

    ros::NodeHandle depth_nh(getPrivateNodeHandle(), "depth_registered");
    m_depth_it.reset(new image_transport::ImageTransport(depth_nh));
    m_pub_depth = m_depth_it->advertiseCamera("image_raw", 1);


    init();
    setupRGBInfo();
    setupDepthInfo();

    m_toggleService = nh.advertiseService("toggle", &XtionGrabber::toggleService, this);

    m_thread = boost::thread(boost::bind(&XtionGrabber::read_thread, this));

    NODELET_INFO_STREAM("[" << m_deviceName << "] Initialized correctly");
    /*if(!initial_state)
    {
        NODELET_INFO_STREAM("[" << m_deviceName << "] Going into idle mode");
        std_srvs::Trigger msg;
        toggleService(msg.request, msg.response);
    }*/
}

void XtionGrabber::init()
{
    if(!setupColor(m_colorDevice))
        throw std::runtime_error("Could not setup color channel");

    if(!setupDepth(m_depthDevice))
        throw std::runtime_error("Could not setup depth channel");

    m_colorFocalLength = 525.0f * m_colorWidth / 640;

    // The depth sensor actually has a different focal length, but since
    // we are using hw registration, the rgb focal length applies.
    m_depthFocalLength = 525.0f * m_depthWidth / 640;

    m_cloudGenerator.init(m_depthWidth, m_depthHeight, m_depthFocalLength);

    if(!startColor())
        NODELET_WARN("Could not start color streaming");
    if(!startDepth())
        NODELET_WARN("Could not start depth streaming");
}

sensor_msgs::ImagePtr XtionGrabber::createDepthImage()
{
    sensor_msgs::ImagePtr img = m_depth_pool->create();

    img->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    img->width = m_depthWidth;
    img->height = m_depthHeight;
    img->step = img->width * 2;
    img->data.resize(img->step * img->height);
    img->header.frame_id = m_depth_info.header.frame_id;

    return img;
}

ros::Time timeFromTimeval(const struct timeval& tv)
{
    struct timespec current_monotonic;
    clock_gettime(CLOCK_MONOTONIC, &current_monotonic);

    ros::Time current_time = ros::Time::now();

    int64_t nsec_diff = (current_monotonic.tv_sec - tv.tv_sec) * 1000000000LL + current_monotonic.tv_nsec - tv.tv_usec*1000L;

    return ros::Time(
                current_time.sec + nsec_diff / 1000000000L,
                (current_time.nsec + nsec_diff) % 1000000000L
                );
}

void XtionGrabber::read_thread()
{
    fd_set fds;

    m_cameraMux.lock();
    ros::Time counter = ros::Time::now();
    ros::Duration timeout(2.0);

    while(!m_shouldExit && ros::ok())
    {
        //NODELET_INFO_THROTTLE(1, "publishing");
        FD_ZERO(&fds);
        FD_SET(m_depth_fd, &fds);
        FD_SET(m_color_fd, &fds);

        int ret = select(std::max(m_depth_fd, m_color_fd)+1, &fds, 0, 0, 0);

        if(ret < 0)
        {
            NODELET_INFO("Could not select()");
            return;
        }

        if(FD_ISSET(m_color_fd, &fds))
        {
            v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;

            if(ioctl(m_color_fd, VIDIOC_DQBUF, &buf) != 0)
            {
                if(errno == EAGAIN)
                    continue;
                NODELET_INFO("Could not dequeue buffer");
                return;
            }

            ColorBuffer* buffer = &m_color_buffers[buf.index];

            sensor_msgs::ImagePtr img = m_color_pool->create();
            img->width = m_colorWidth;
            img->height = m_colorHeight;
            img->step = img->width * 4;
            img->data.resize(img->step * img->height);
            img->header.stamp = timeFromTimeval(buf.timestamp);
            img->header.frame_id = m_color_info.header.frame_id;

            img->encoding = sensor_msgs::image_encodings::BGRA8;

#if HAVE_LIBYUV
            libyuv::ConvertToARGB(
                        buffer->data.data(), buffer->data.size(),
                        img->data.data(),
                        m_colorWidth*4, 0, 0, m_colorWidth, m_colorHeight, m_colorWidth, m_colorHeight,
                        libyuv::kRotate0, libyuv::FOURCC_UYVY
                        );
#else
            uint32_t* dptr = (uint32_t*)img->data.data();

            for(size_t y = 0; y < img->height; ++y)
            {
                for(size_t x = 0; x < img->width-1; x += 2)
                {
                    unsigned char* base = &buffer->data[y*m_colorWidth*2+x*2];
                    float y1 = base[1];
                    float u  = base[0];
                    float y2 = base[3];
                    float v  = base[2];

                    uint32_t rgb1 = yuv(y1, u, v);
                    uint32_t rgb2 = yuv(y2, u, v);

                    dptr[y*img->width + x + 0] = rgb1;
                    dptr[y*img->width + x + 1] = rgb2;
                }
            }
#endif

            m_lastColorImage = img;
            m_lastColorSeq = buf.sequence;

            m_color_info.header.stamp = img->header.stamp;
            if(m_pub_color.getNumSubscribers() != 0)
            {
                m_pub_color.publish(img, boost::make_shared<sensor_msgs::CameraInfo>(m_color_info));
                counter = ros::Time::now();
            }

            if(ioctl(m_color_fd, VIDIOC_QBUF, &buffer->buf) != 0)
            {
                NODELET_INFO("Could not queue buffer");
                return;
            }
        }

        if(FD_ISSET(m_depth_fd, &fds))
        {
            v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;

            if(ioctl(m_depth_fd, VIDIOC_DQBUF, &buf) != 0)
            {
                if(errno == EAGAIN)
                    continue;
                NODELET_INFO("Could not dequeue buffer");
                return;
            }

            DepthBuffer* buffer = &m_depth_buffers[buf.index];

            buffer->image->header.stamp = timeFromTimeval(buf.timestamp);

            m_lastDepthImage = buffer->image;
            m_lastDepthSeq = buf.sequence;

            m_depth_info.header.stamp = buffer->image->header.stamp;
            if(m_pub_depth.getNumSubscribers() != 0)
            {
                counter = ros::Time::now();
                m_pub_depth.publish(buffer->image, boost::make_shared<sensor_msgs::CameraInfo>(m_depth_info));
            }

            buffer->image.reset();

            buffer->image = createDepthImage();
            buffer->buf.m.userptr = (long unsigned int)buffer->image->data.data();

            if(ioctl(m_depth_fd, VIDIOC_QBUF, &buffer->buf) != 0)
            {
                NODELET_INFO("Could not queue buffer");
                return;
            }
        }

        m_cameraMux.unlock();
        if(m_lastColorSeq == m_lastDepthSeq)
        {
            if(m_pub_cloud.getNumSubscribers() != 0)
            {
                counter = ros::Time::now();
                publishPointCloud(m_lastDepthImage, &m_cloudGenerator, &m_pub_cloud);
            }
        }
        //Automatically go into idle mode if timeout is reached without subscribers
        if(ros::Time::now()-counter>timeout)
        {
            NODELET_INFO_STREAM("[" << m_deviceName << "] No subscribers. Going into idle mode");
            std_srvs::Trigger msg;
            toggleService(msg.request, msg.response);
            while(!m_pub_cloud.getNumSubscribers() && !m_pub_depth.getNumSubscribers() && !m_pub_color.getNumSubscribers() && ros::ok())
            {
                ros::Duration(0.1).sleep();
            }
            if(ros::ok())
            {
                counter = ros::Time::now();
                toggleService(msg.request, msg.response);
            }
        }
        m_cameraMux.lock();
    }
    m_cameraMux.unlock();
    std::cout << "[read_thread] exit\n";
}

void XtionGrabber::publishPointCloud(const sensor_msgs::ImageConstPtr& depth,
                                     accel::PointCloudGenerator* generator,
                                     ros::Publisher* dest)
{
    if(!m_lastColorImage || !depth)
        return;

    sensor_msgs::PointCloud2::Ptr cloud = m_pointCloudPool->create();

    cloud->header.stamp = m_lastColorImage->header.stamp;

    cloud->header.frame_id = m_color_info.header.frame_id;

    cloud->fields.resize(4);
    cloud->fields[0].name = "x";
    cloud->fields[0].offset = 0;
    cloud->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud->fields[0].count = 1;
    cloud->fields[1].name = "y";
    cloud->fields[1].offset = 4;
    cloud->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud->fields[1].count = 1;
    cloud->fields[2].name = "z";
    cloud->fields[2].offset = 8;
    cloud->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud->fields[2].count = 1;
    cloud->fields[3].name = "rgb";
    cloud->fields[3].offset = 16;
    cloud->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    cloud->fields[3].count = 1;

    cloud->width = depth->width;
    cloud->height = depth->height;
    cloud->is_bigendian = 0;
    cloud->is_dense = 1;
    cloud->point_step = 32;
    cloud->row_step = cloud->point_step * cloud->width;
    cloud->data.resize(cloud->row_step * cloud->height);

    timespec start;
    timespec end;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

    int colorOffset = 0;
    if(m_colorHeight == 1024)
    {
        // SXGA is the true sensor resolution. The xtion can only sense depth
        // in VGA resolution, which has a wider aspect ratio.
        // => skip the first 32 rows in the color data to get the same aspect
        //    ratio.
        colorOffset = 32 * m_colorWidth;
    }

    generator->generatePointCloud(
                (uint16_t*)depth->data.data(),
                (uint32_t*)m_lastColorImage->data.data() + colorOffset,
                m_colorWidth / depth->width,
                cloud->data.data()
                );

    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);

    dest->publish(cloud);
}

void XtionGrabber::setupRGBInfo()
{
    ros::NodeHandle color_nh(getPrivateNodeHandle(), "rgb");

    std::string info_url;
    getPrivateNodeHandle().param("info_url", info_url, std::string(""));

    v4l2_capability caps;
    memset(&caps, 0, sizeof(caps));
    if(ioctl(m_color_fd, VIDIOC_QUERYCAP, &caps) != 0)
    {
        NODELET_ERROR("Could not get camera information");
        return;
    }

    std::string card((const char*)caps.card);
    std::replace(card.begin(), card.end(), ' ', '_');
    card.erase(std::remove(card.begin(), card.end(), ':'));

    std::stringstream ss;
    ss << card << "_" << m_colorWidth;

    m_color_infoMgr.reset(
                new camera_info_manager::CameraInfoManager(
                    color_nh, ss.str(), info_url
                    )
                );

    if(m_color_infoMgr->isCalibrated())
    {
        NODELET_ERROR("Using saved calibration...");
        m_color_info = m_color_infoMgr->getCameraInfo();
    }
    else
    {
        /* We are reporting information about the *color* sensor here. */

        m_color_info.width = m_colorWidth;
        m_color_info.height = m_colorHeight;

        // No distortion
        m_color_info.D.resize(5, 0.0);
        m_color_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

        // Simple camera matrix: square pixels (fx = fy), principal point at center
        m_color_info.K.assign(0.0);
        m_color_info.K[0] = m_color_info.K[4] = m_colorFocalLength;
        m_color_info.K[2] = (m_colorWidth /2.0) - 0.5;
        m_color_info.K[5] = (m_colorHeight/2.0) - 0.5;
        m_color_info.K[8] = 1.0;

        // No separate rectified image plane, so R = I
        m_color_info.R.assign(0.0);
        m_color_info.R[0] = m_color_info.R[4] = m_color_info.R[8] = 1.0;

        // Then P=K(I|0) = (K|0)
        m_color_info.P.assign(0.0);
        m_color_info.P[0] = m_color_info.P[5] = m_colorFocalLength; // fx, fy
        m_color_info.P[2] = m_color_info.K[2]; // cx
        m_color_info.P[6] = m_color_info.K[5]; // cy
        m_color_info.P[10] = 1.0;
    }

    if (m_tfprefix.empty()) m_color_info.header.frame_id = m_deviceName + "_rgb_optical_frame";
    else m_color_info.header.frame_id = m_tfprefix + m_deviceName + "_rgb_optical_frame";
}

void XtionGrabber::setupDepthInfo()
{
    ros::NodeHandle depth_nh(getPrivateNodeHandle(), "depth");

    std::string info_url;
    getPrivateNodeHandle().param("info_url", info_url, std::string(""));

    v4l2_capability caps;
    memset(&caps, 0, sizeof(caps));
    if(ioctl(m_depth_fd, VIDIOC_QUERYCAP, &caps) != 0)
    {
        NODELET_ERROR("Could not get camera information");
        return;
    }

    std::string card((const char*)caps.card);
    std::replace(card.begin(), card.end(), ' ', '_');
    card.erase(std::remove(card.begin(), card.end(), ':'));

    std::stringstream ss;
    ss << card << "_" << m_depthWidth;

    m_depth_infoMgr.reset(
                new camera_info_manager::CameraInfoManager(
                    depth_nh, ss.str(), info_url
                    )
                );

    if(m_depth_infoMgr->isCalibrated())
    {
        NODELET_ERROR("Using saved calibration...");
        m_depth_info = m_depth_infoMgr->getCameraInfo();
    }
    else
    {
        /* We are reporting information about the *depth* sensor here. */

        m_depth_info.width = m_depthWidth;
        m_depth_info.height = m_depthHeight;

        // No distortion
        m_depth_info.D.resize(5, 0.0);
        m_depth_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

        // Simple camera matrix: square pixels (fx = fy), principal point at center
        m_depth_info.K.assign(0.0);
        m_depth_info.K[0] = m_depth_info.K[4] = m_depthFocalLength;
        m_depth_info.K[2] = (m_depthWidth /2.0) - 0.5;
        m_depth_info.K[5] = (m_depthHeight/2.0) - 0.5;
        m_depth_info.K[8] = 1.0;

        // No separate rectified image plane, so R = I
        m_depth_info.R.assign(0.0);
        m_depth_info.R[0] = m_depth_info.R[4] = m_depth_info.R[8] = 1.0;

        // Then P=K(I|0) = (K|0)
        m_depth_info.P.assign(0.0);
        m_depth_info.P[0] = m_depth_info.P[5] = m_depthFocalLength; // fx, fy
        m_depth_info.P[2] = m_depth_info.K[2]; // cx
        m_depth_info.P[6] = m_depth_info.K[5]; // cy
        m_depth_info.P[10] = 1.0;
    }

    if (m_tfprefix.empty()) m_depth_info.header.frame_id = m_deviceName + "_rgb_optical_frame";
    else m_depth_info.header.frame_id = m_tfprefix + m_deviceName + "_rgb_optical_frame";
}

}

PLUGINLIB_EXPORT_CLASS(xtion_grabber::XtionGrabber, nodelet::Nodelet)
