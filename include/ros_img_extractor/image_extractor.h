/*
* Software License Agreement (Modified BSD License)
*
* Copyright (c) 2014, PAL Robotics, S.L.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of PAL Robotics, S.L. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
/** \author Jeremie Deray. */

#ifndef ROS_IMG_EXTRACTOR_IMAGE_EXTRACTOR_H
#define ROS_IMG_EXTRACTOR_IMAGE_EXTRACTOR_H

// ROS headers
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

// Boost headers
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/variant.hpp>

// OpenCV header
#include <opencv2/core/core.hpp>

class ImageExtractor {

  public:

    ImageExtractor();

  protected:

    void callback(const sensor_msgs::ImageConstPtr&);

    cv::Mat _image;

    message_filters::Subscriber<sensor_msgs::Image> _subscriber;

    ros::NodeHandle _node;

    int _imgnum;


    std::string _filePath;
    std::string _topic;
};

class ImageExtractorPose : public ImageExtractor {

  public:

    ImageExtractorPose();

  protected:

    geometry_msgs::PoseStamped getPose(const ros::Time &stamp);

    bool checkPosition(geometry_msgs::PoseStamped lastImagePose,
                       geometry_msgs::PoseStamped currentPose);

    std::string _frame1, _frame2;

    void extractAtPose(const sensor_msgs::ImageConstPtr&);
    void extractAllFrame(const sensor_msgs::ImageConstPtr&);

    std::string encode(const geometry_msgs::PoseStamped& pose);
    std::string encode(const geometry_msgs::PoseStamped& pose,
                       const std::string&, int imgnum);

    tf::TransformListener _tfL;
    geometry_msgs::PoseStamped _previousPose;

     double _collectImgMaxDist;
     double _collectImgMaxAngle;
};

namespace
{
    namespace sp = message_filters::sync_policies;
}

class SyncImageExtractor {

  public:

    SyncImageExtractor();

  protected:

    template<class T> struct Sync
    {
        typedef message_filters::Synchronizer<T> type;
        typedef boost::shared_ptr<message_filters::Synchronizer<T> > Ptr;
    };

    typedef sensor_msgs::Image I;
    typedef sensor_msgs::ImageConstPtr ICPtr;

    typedef image_transport::ImageTransport It;

    typedef image_transport::SubscriberFilter SubsFil;

    typedef boost::function<void (const std::vector<ICPtr>&)> callbackPtr;

    typedef sp::ApproximateTime<I, I> ApproxSync2;
    typedef sp::ApproximateTime<I, I, I> ApproxSync3;
    typedef sp::ApproximateTime<I, I, I, I> ApproxSync4;
    typedef sp::ApproximateTime<I, I, I, I, I> ApproxSync5;
    typedef sp::ApproximateTime<I, I, I, I, I, I> ApproxSync6;
    typedef sp::ApproximateTime<I, I, I, I, I, I, I> ApproxSync7;
    typedef sp::ApproximateTime<I, I, I, I, I, I, I, I> ApproxSync8;
    typedef sp::ApproximateTime<I, I, I, I, I, I, I, I, I> ApproxSync9;

    typedef boost::variant< Sync<ApproxSync2>::Ptr,
                            Sync<ApproxSync3>::Ptr,
                            Sync<ApproxSync4>::Ptr,
                            Sync<ApproxSync5>::Ptr,
                            Sync<ApproxSync6>::Ptr,
                            Sync<ApproxSync7>::Ptr,
                            Sync<ApproxSync8>::Ptr,
                            Sync<ApproxSync9>::Ptr > VariantApproxSync;

    void callback(const std::vector<ICPtr>& vecImgPtr);

    void wrapCallback(const ICPtr&, const ICPtr&,
                      const ICPtr&, const ICPtr&,
                      const ICPtr&, const ICPtr&,
                      const ICPtr&, const ICPtr&);

    void initSyncSubs();

    cv::Mat _image;

    std::vector<std::string> _filePath;
    std::vector<std::string> _topics;
    std::string _trpHint;

    boost::shared_ptr<It> _imageTransport;
    boost::ptr_vector<SubsFil> _imgSubs;
    boost::shared_ptr<VariantApproxSync> _approxSynchronizer;

    callbackPtr _callbackptr;

    ros::NodeHandle _node;
    int _imgnum;
    int _qSize;
};

class SyncImageExtractorPose : public SyncImageExtractor {

  public:

    SyncImageExtractorPose();

  protected:

    geometry_msgs::PoseStamped getPose(const ros::Time &stamp);

    bool checkPosition(geometry_msgs::PoseStamped lastImagePose,
                       geometry_msgs::PoseStamped currentPose);

    std::string encode(const geometry_msgs::PoseStamped& pose);
    std::string encode(const geometry_msgs::PoseStamped& pose,
                       const std::string&, int imgn);

    void extractAllFrame(const std::vector<ICPtr>& vecImgPtr);
    void extractAtPose(const std::vector<ICPtr>& vecImgPtr);

    std::string _frame1, _frame2;

    double _collectImgMaxDist;
    double _collectImgMaxAngle;

    tf::TransformListener _tfL;

    geometry_msgs::PoseStamped _previousPose;
};

#endif // ROS_IMG_EXTRACTOR_IMAGE_EXTRACTOR_H
