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

#include "ros_img_extractor/image_extractor.h"

// ROS headers
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Boost headers
#include <boost/algorithm/string/replace.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include <boost/filesystem.hpp>

// OpenCV header
#include <opencv2/highgui/highgui.hpp>

#include <angles/angles.h>
#include <cstdarg>

ImageExtractor::ImageExtractor() :
    _node("~"),
    _imgnum(0)
{
    _node.param("file_path",_filePath ,std::string("./"));

    _subscriber.subscribe( _node, "extract_topic", 1 , ros::TransportHints().unreliable());

    if (!boost::filesystem::is_directory(_filePath))
    {
      ROS_ERROR_STREAM("ERROR! Directory not found: " << _filePath);
      ros::shutdown();
    }

    ROS_INFO_STREAM("Saving images in : " << _filePath);

    _subscriber.registerCallback(&ImageExtractor::callback, this);

    _topic = _subscriber.getTopic();

    ROS_INFO_STREAM("Listening to : " << _topic);
}

void ImageExtractor::callback(const sensor_msgs::ImageConstPtr& msgImg)
{
    try
    {
        _image = cv_bridge::toCvShare(msgImg, msgImg->encoding)->image;
    }
    catch (cv_bridge::Exception)
    {
        ROS_ERROR("Couldn't convert %s image", msgImg->encoding.c_str());
        return;
    }

    std::string filename = boost::str(boost::format( "%04d" ) % _imgnum )
                           + "_" + _topic + ".jpg";

    boost::filesystem::path fullPath(_filePath);
    fullPath /= filename;

    try
    {
        cv::imwrite(fullPath.c_str(), _image);
    }
    catch (cv::Exception)
    {
        ROS_ERROR_STREAM("Couldn't save image %s " << fullPath.c_str());
        return;
    }

    ROS_INFO_STREAM("Saved image : "<< filename <<" at "<< _filePath <<".");

    ++_imgnum;
}

ImageExtractorPose::ImageExtractorPose()
{
    _node.param("frame1",_frame1,std::string("/base_link"));
    _node.param("frame2",_frame2,std::string("/map"));

    bool allFrame;
    _node.param("each_frame", allFrame, false);

    _node.param("dist_to_collect", _collectImgMaxDist, (double)0.5);
    _node.param("angle_to_collect", _collectImgMaxAngle, (double)0.78);

    ROS_INFO_STREAM("Frames are : " << _frame1 << " & " << _frame2);
    ROS_INFO_STREAM("Will collect images every " << _collectImgMaxDist << " meter.");
    ROS_INFO_STREAM("Will collect images every " << _collectImgMaxAngle << " radian.");

    (!allFrame) ? _subscriber.registerCallback(&ImageExtractorPose::extractAtPose, this) :
                  _subscriber.registerCallback(&ImageExtractorPose::extractAllFrame, this);

    _previousPose = getPose(ros::Time::now());
}

std::string ImageExtractorPose::encode(const geometry_msgs::PoseStamped& pose)
{
    std::stringstream s;

    s << "Img_" << pose.pose.position.x << "_" << pose.pose.position.y << "_"
    << pose.pose.orientation.x << "_" << pose.pose.orientation.y << "_"
    << pose.pose.orientation.z << "_" << pose.pose.orientation.w << ".jpg";

    return s.str();
}

std::string ImageExtractorPose::encode(const geometry_msgs::PoseStamped& pose,
                                       const std::string& topic, int imgnum)
{
    return boost::str(boost::format( "%04d" ) % imgnum) + "_" + topic + "_"  + encode(pose);
}

void ImageExtractorPose::extractAllFrame(const sensor_msgs::ImageConstPtr& msgImg)
{
    geometry_msgs::PoseStamped pMap = getPose(msgImg->header.stamp);

    std::string filename = encode(pMap, _topic, _imgnum);

    try
    {
        _image = cv_bridge::toCvShare(msgImg, msgImg->encoding)->image;
    }
    catch (cv_bridge::Exception)
    {
        ROS_ERROR("Couldn't convert %s image", msgImg->encoding.c_str());
        return;
    }
    
    boost::filesystem::path fullPath(_filePath);
    fullPath /= filename;

    try
    {
        cv::imwrite(fullPath.c_str(), _image);
    }
    catch (cv::Exception)
    {
        ROS_ERROR_STREAM("Couldn't save image %s " << fullPath.c_str());
        return;
    }

    ROS_INFO_STREAM("Saved image : "<< filename <<" at "<< _filePath <<".");

    ++_imgnum;
}

bool ImageExtractorPose::checkPosition(geometry_msgs::PoseStamped lastImagePose,
                                       geometry_msgs::PoseStamped currentPose)
{
    // Compute linear difference
    const double dx = lastImagePose.pose.position.x - currentPose.pose.position.x;
    const double dy = lastImagePose.pose.position.y - currentPose.pose.position.y;
    const double distance_difference = hypot(dx, dy);

    // Compute angular difference
    const double lastYaw = tf::getYaw(lastImagePose.pose.orientation);
    const double currentYaw = tf::getYaw(currentPose.pose.orientation);
    const double angle_difference = angles::shortest_angular_distance(currentYaw, lastYaw);

    return ( (fabs(distance_difference) > _collectImgMaxDist) ||
             (fabs(angle_difference) > _collectImgMaxAngle) );
}

geometry_msgs::PoseStamped ImageExtractorPose::getPose(const ros::Time &stamp)
{
    geometry_msgs::PoseStamped pBase, pMap;
    pBase.header.frame_id = _frame1;
    pBase.pose.position.x = 0.0;
    pBase.pose.position.y = 0.0;
    pBase.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    ros::Time transform_stamp = stamp;

    _tfL.getLatestCommonTime(pBase.header.frame_id, _frame2, transform_stamp, NULL);

    pBase.header.stamp = transform_stamp;

    try
    {
        _tfL.transformPose(_frame2, pBase, pMap);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    return pMap;
}

void ImageExtractorPose::extractAtPose(const sensor_msgs::ImageConstPtr& msgImg)
{
    geometry_msgs::PoseStamped pMap = getPose(msgImg->header.stamp);

    if (!checkPosition(pMap, _previousPose)) return;

    std::string filename = encode(pMap, _topic, _imgnum);

    try
    {
        _image = cv_bridge::toCvShare(msgImg, msgImg->encoding)->image;
    }
    catch (cv_bridge::Exception)
    {
        ROS_ERROR("Couldn't convert %s image", msgImg->encoding.c_str());
        return;
    }

    boost::filesystem::path fullPath(_filePath);
    fullPath /= filename;

    try
    {
        cv::imwrite(fullPath.c_str(), _image);
    }
    catch (cv::Exception)
    {
        ROS_ERROR_STREAM("Couldn't save image %s " << fullPath.c_str());
        return;
    }

    ROS_INFO_STREAM("Saved image : "<< filename <<" at "<< _filePath <<".");

    _previousPose = pMap;

    ++_imgnum;
}

////////////////////////////////////////////////////////////////////////////////////////

SyncImageExtractor::SyncImageExtractor() :
    _node("~"),
    _imgnum(0)
{
    std::vector<std::string> topics,filePath;
    _node.getParam("topics", topics);
    _node.getParam("file_paths", filePath);
    _node.param("transport", _trpHint, std::string("compressed"));
    _node.param("queue_size", _qSize, (int)10);

    _imageTransport.reset( new It( _node ) );

    image_transport::TransportHints transportHint(_trpHint);

    for (int i=0; i<filePath.size(); ++i)
    {
        if (filePath[i] == "none")
        {
            continue;
        }
        else if (!boost::filesystem::is_directory(filePath[i]))
        {
            ROS_ERROR_STREAM("ERROR! Directory not found: " << filePath[i]);
        }
        else
        {
            _filePath.push_back(filePath[i]);
        }
    }

    if (_filePath.empty())
    {
        ROS_ERROR("ERROR! No valid directory !");
        ros::shutdown();
    }

    for (int i=0; i<topics.size(); ++i)
    {
        if (topics[i] != "none")
        {
            _imgSubs.push_back(new SubsFil(*_imageTransport, topics[i], 1, transportHint));
            _topics.push_back(topics[i]);
        }
    }

    if (_imgSubs.size()<2 || _imgSubs.size()>9)
    {
        ROS_ERROR("Can't listen to less than 2 topics neither more than 8 !");
        ros::shutdown();
    }
    else if (_filePath.size() != _imgSubs.size())
    {

        ROS_ERROR_STREAM("Number of topics is different from number of directories" <<
                         " to save images. \nImages will be saved in : " << _filePath[0]);

        std::string unic = _filePath[0];
        _filePath.clear();
        _filePath.resize(_imgSubs.size(), unic);
    }

    for (int i=0; i<_filePath.size(); ++i)
    {
        ROS_INFO_STREAM("Saving images from topic : " << _topics[i]
                        << " in folder : " << _filePath[i] );
         boost::replace_all(_topics[i], "/", "_");
    }
    ROS_INFO_STREAM("Image transport hint : " << _trpHint);
    ROS_INFO_STREAM("Synchronizer queue size : " << _qSize);

    initSyncSubs();

    _callbackptr = boost::bind(&SyncImageExtractor::callback, this, _1);
}

void SyncImageExtractor::callback(const std::vector<ICPtr>& vecImgPtr)
{
    for (int i=0; i<vecImgPtr.size(); ++i)
    {
        if (vecImgPtr[i].use_count() > 0)
        {
            try
            {
                _image = cv_bridge::toCvShare(vecImgPtr[i], vecImgPtr[i]->encoding)->image;
            }
            catch (cv_bridge::Exception)
            {
                ROS_ERROR("Couldn't convert %s image", vecImgPtr[i]->encoding.c_str());
                return;
            }

            std::string filename = boost::str(boost::format( "%04d" ) % _imgnum )
                    + "_" + _topics[i] + ".jpg";

	    boost::filesystem::path fullPath(_filePath[i]);
	    fullPath /= filename;

            try
            {
                cv::imwrite(fullPath.c_str(), _image);
            }
            catch (cv::Exception)
            {
                ROS_ERROR_STREAM("Couldn't save image %s " << fullPath.c_str());
                return;
            }

            ROS_INFO_STREAM("Saved image : "<< filename <<" at "<< _filePath[i] <<". \n");
        }
    }

    ++_imgnum;
}

void SyncImageExtractor::initSyncSubs()
{
    switch (_imgSubs.size())
    {
    case 2:
        _approxSynchronizer.reset(new VariantApproxSync(Sync<ApproxSync2>::Ptr(new Sync<ApproxSync2>::type(ApproxSync2(_qSize),
                                                                                                        _imgSubs[0], _imgSubs[1]))));
        boost::get<Sync<ApproxSync2>::Ptr>(*_approxSynchronizer)->registerCallback(
                    (boost::bind(&SyncImageExtractor::wrapCallback, this, _1, _2, ICPtr(), ICPtr(), ICPtr(), ICPtr(), ICPtr(), ICPtr())));
//  Make it happen
//       boost::bind(&SyncImageExtractor::callback, this, boost::assign::list_of<ICPtr>(_1)(_2)(ICPtr())(ICPtr())(ICPtr())(ICPtr())(ICPtr())(ICPtr())) );
//       boost::bind(&SyncImageExtractor::callback, this, boost::assign::list_of<ICPtr>(_1)(_2)(ICPtr())(ICPtr())(ICPtr())(ICPtr())(ICPtr())(ICPtr()).convert_to_container<std::vector<ICPtr> >() ) );
        break;

    case 3:
        _approxSynchronizer.reset(new VariantApproxSync(Sync<ApproxSync3>::Ptr(new Sync<ApproxSync3>::type(ApproxSync3(_qSize),
                                                                                            _imgSubs[0], _imgSubs[1], _imgSubs[2]))));
        boost::get<Sync<ApproxSync3>::Ptr>(*_approxSynchronizer)->registerCallback(
                    boost::bind(&SyncImageExtractor::wrapCallback, this, _1, _2, _3, ICPtr(), ICPtr(), ICPtr(), ICPtr(), ICPtr()));
        break;

    case 4:
        _approxSynchronizer.reset(new VariantApproxSync(Sync<ApproxSync4>::Ptr(new Sync<ApproxSync4>::type(ApproxSync4(_qSize),
                                                                                _imgSubs[0], _imgSubs[1], _imgSubs[2], _imgSubs[3]))));
        boost::get<Sync<ApproxSync4>::Ptr>(*_approxSynchronizer)->registerCallback(
                    boost::bind(&SyncImageExtractor::wrapCallback, this, _1, _2, _3, _4, ICPtr(), ICPtr(), ICPtr(), ICPtr()));
        break;

    case 5:
        _approxSynchronizer.reset(new VariantApproxSync(Sync<ApproxSync5>::Ptr(new Sync<ApproxSync5>::type(ApproxSync5(_qSize),
                                                                    _imgSubs[0], _imgSubs[1], _imgSubs[2], _imgSubs[3], _imgSubs[4]))));
        boost::get<Sync<ApproxSync5>::Ptr>(*_approxSynchronizer)->registerCallback(
                    boost::bind(&SyncImageExtractor::wrapCallback, this, _1, _2, _3, _4, _5, ICPtr(), ICPtr(), ICPtr()));
        break;

    case 6:
        _approxSynchronizer.reset(new VariantApproxSync(Sync<ApproxSync6>::Ptr(new Sync<ApproxSync6>::type(ApproxSync6(_qSize),
                                                        _imgSubs[0], _imgSubs[1], _imgSubs[2], _imgSubs[3], _imgSubs[4], _imgSubs[5]))));
        boost::get<Sync<ApproxSync6>::Ptr>(*_approxSynchronizer)->registerCallback(
                    boost::bind(&SyncImageExtractor::wrapCallback, this, _1, _2, _3, _4, _5, _6, ICPtr(), ICPtr()));
        break;

    case 7:
        _approxSynchronizer.reset(new VariantApproxSync(Sync<ApproxSync7>::Ptr(new Sync<ApproxSync7>::type(ApproxSync7(_qSize),
                                            _imgSubs[0], _imgSubs[1], _imgSubs[2], _imgSubs[3], _imgSubs[4], _imgSubs[5], _imgSubs[6]))));
        boost::get<Sync<ApproxSync7>::Ptr>(*_approxSynchronizer)->registerCallback(
                    boost::bind(&SyncImageExtractor::wrapCallback, this, _1, _2, _3, _4, _5, _6, _7, ICPtr()));
        break;

    case 8:
        _approxSynchronizer.reset(new VariantApproxSync(Sync<ApproxSync8>::Ptr(new Sync<ApproxSync8>::type(ApproxSync8(_qSize),
                                _imgSubs[0], _imgSubs[1], _imgSubs[2], _imgSubs[3], _imgSubs[4], _imgSubs[5], _imgSubs[6], _imgSubs[7]))));
        boost::get<Sync<ApproxSync8>::Ptr>(*_approxSynchronizer)->registerCallback(
                    boost::bind(&SyncImageExtractor::wrapCallback, this, _1, _2, _3, _4, _5, _6, _7, _8));
        break;
    }
}

void SyncImageExtractor::wrapCallback(const ICPtr& a, const ICPtr& b,
                                      const ICPtr& c, const ICPtr& d,
                                      const ICPtr& e, const ICPtr& f,
                                      const ICPtr& g, const ICPtr& h)
{
    std::vector<ICPtr> vecICPtr = boost::assign::list_of<ICPtr>(a)(b)(c)(d)(e)(f)(g)(h);

    _callbackptr(vecICPtr);
}


SyncImageExtractorPose::SyncImageExtractorPose()
{
    _node.param("frame1",_frame1,std::string("/base_link"));
    _node.param("frame2",_frame2,std::string("/map"));

    bool allFrame;
    _node.param("each_frame", allFrame, false);

    _node.param("dist_to_collect", _collectImgMaxDist, (double)0.5);
    _node.param("angle_to_collect", _collectImgMaxAngle, (double)0.78);

    ROS_INFO_STREAM("Frames are : " << _frame1 << " & " << _frame2);
    ROS_INFO_STREAM("Will collect images every " << _collectImgMaxDist << " meter.");
    ROS_INFO_STREAM("Will collect images every " << _collectImgMaxAngle << " radian.");

    (!allFrame) ? _callbackptr = boost::bind(&SyncImageExtractorPose::extractAtPose, this, _1) :
                  _callbackptr = boost::bind(&SyncImageExtractorPose::extractAllFrame, this, _1);

    _previousPose = getPose(ros::Time::now());
}

std::string SyncImageExtractorPose::encode(const geometry_msgs::PoseStamped& pose)
{
    std::stringstream s;

    s << "Img_" << pose.pose.position.x << "_" << pose.pose.position.y << "_"
    << pose.pose.orientation.x << "_" << pose.pose.orientation.y << "_"
    << pose.pose.orientation.z << "_" << pose.pose.orientation.w << ".jpg";

    return s.str();
}

std::string SyncImageExtractorPose::encode(const geometry_msgs::PoseStamped& pose,
                                           const std::string& topic, int imgn)
{
    return boost::str(boost::format( "%04d" ) % imgn ) + "_" + topic + "_" + encode(pose);
}

void SyncImageExtractorPose::extractAllFrame(const std::vector<ICPtr>& vecImgPtr)
{
    for (int i=0; i<vecImgPtr.size(); ++i)
    {
        if (vecImgPtr[i].use_count() > 0)
        {
            geometry_msgs::PoseStamped pMap = getPose(vecImgPtr[i]->header.stamp);

            try
            {
                _image = cv_bridge::toCvShare(vecImgPtr[i], vecImgPtr[i]->encoding)->image;
            }
            catch (cv_bridge::Exception)
            {
                ROS_ERROR("Couldn't convert %s image", vecImgPtr[i]->encoding.c_str());
                return;
            }

            std::string filename = encode(pMap, _topics[i], _imgnum);

	    boost::filesystem::path fullPath(_filePath[i]);
	    fullPath /= filename;

            try
	    {
                cv::imwrite(fullPath.c_str(), _image );
            }
            catch (cv::Exception)
            {
                ROS_ERROR_STREAM("Couldn't save image %s " << fullPath.c_str());
                return;
            }

            ROS_INFO_STREAM("Saved image : "<< filename <<" at "<< _filePath[i] <<". \n");
        }
    }

    ++_imgnum;
}

void SyncImageExtractorPose::extractAtPose(const std::vector<ICPtr>& vecImgPtr)
{
    geometry_msgs::PoseStamped pMap;

    for (int i=0; i<vecImgPtr.size(); ++i)
    {
        if (vecImgPtr[i].use_count() > 0)
        {
            pMap = getPose(vecImgPtr[i]->header.stamp);

            if (!checkPosition(pMap, _previousPose)) return;

            try
            {
                _image = cv_bridge::toCvShare(vecImgPtr[i], vecImgPtr[i]->encoding)->image;
            }
            catch (cv_bridge::Exception)
            {
                ROS_ERROR("Couldn't convert %s image", vecImgPtr[i]->encoding.c_str());
                return;
            }

            std::string filename = encode(pMap, _topics[i], _imgnum);

	    boost::filesystem::path fullPath(_filePath[i]);
	    fullPath /= filename;

            try
            {
                cv::imwrite(fullPath.c_str(), _image );
            }
            catch (cv::Exception)
            {
                ROS_ERROR_STREAM("Couldn't save image %s " << fullPath.c_str());
                return;
            }

            ROS_INFO_STREAM("Saved image : "<< filename <<" at "<< _filePath[i] <<". \n");
        }
    }

    _previousPose = pMap;

    ++_imgnum;
}

geometry_msgs::PoseStamped SyncImageExtractorPose::getPose(const ros::Time &stamp)
{
    geometry_msgs::PoseStamped pBase, pMap;
    pBase.header.frame_id = _frame1;
    pBase.pose.position.x = 0.0;
    pBase.pose.position.y = 0.0;
    pBase.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    ros::Time transform_stamp = stamp;

    _tfL.getLatestCommonTime(pBase.header.frame_id, _frame2, transform_stamp, NULL);

    pBase.header.stamp = transform_stamp;

    try
    {
        _tfL.transformPose(_frame2, pBase, pMap);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    return pMap;
}

bool SyncImageExtractorPose::checkPosition(geometry_msgs::PoseStamped lastImagePose,
                                           geometry_msgs::PoseStamped currentPose)
{
    // Compute linear difference
    const double dx = lastImagePose.pose.position.x - currentPose.pose.position.x;
    const double dy = lastImagePose.pose.position.y - currentPose.pose.position.y;
    const double distance_difference = hypot(dx, dy);

    // Compute angular difference
    const double lastYaw = tf::getYaw(lastImagePose.pose.orientation);
    const double currentYaw = tf::getYaw(currentPose.pose.orientation);
    const double angle_difference = angles::shortest_angular_distance(currentYaw, lastYaw);

    return ( (fabs(distance_difference) > _collectImgMaxDist) ||
             (fabs(angle_difference) > _collectImgMaxAngle) );
}
