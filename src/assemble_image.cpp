/**
 * Software License Agreement (BSD License)
 *
 *  Author: Michael Kaca
 *  Copyright (c) 2023
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "image_assembler/assemble_image.hpp"

// openCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>  // todo: move outside to wrapper for ros2 porting
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace image_assembler
{
AssembleImage::AssembleImage() : cloud_ready_(false)
{
  ros::NodeHandle nh("~"), g_nh;

  // not list-init since ros::Time::now() cannot be called prior to the first NodeHandle creation
  latest_acc_complete_time_ = ros::Time::now();

  // desired image dimensions
  nh.param<int>("image_width", image_width_, 640);
  nh.param<int>("image_height", image_height_, 480);

  nh.param<int>("loop_rate", loop_rate_, 20);
  nh.param<std::string>("save_rgb_image_path", save_rgb_image_path_, "");
  nh.param<double>("focal_point", focal_point_, 1.0);
  nh.param<double>("accumulate_time", accumulate_time_, 5.0);

  reset();

  image_rgb_pub_ = nh.advertise<sensor_msgs::Image>("rgb_image", 1);
  image_depth_pub_ = nh.advertise<sensor_msgs::Image>("depth_image", 1);

  cloud_sub_ = g_nh.subscribe("cloud", 5, &AssembleImage::cloudCB, this);

  ROS_INFO_NAMED("assemble_image",
                 "AssembleImage has been successfully initialized. Ensure that the robot is stationary");
}

void AssembleImage::cloudCB(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  latest_cloud_ = *msg;
  cloud_ready_ = true;
}

void AssembleImage::reset()
{
  // min_x_ = std::numeric_limits<double>::max();
  // min_y_ = std::numeric_limits<double>::max();
  min_x_ = std::numeric_limits<double>::max();
  min_y_ = std::numeric_limits<double>::max();
  max_x_ = std::numeric_limits<double>::min();
  max_y_ = std::numeric_limits<double>::min();
  min_intensity_ = std::numeric_limits<int>::max();
  max_intensity_ = 0;
  acc_2d_points_.clear();
}

void AssembleImage::loop()
{
  ros::Rate rate(loop_rate_);

  while (ros::ok())
  {
    if (!rate.sleep())
    {
      ROS_WARN_NAMED("assemble_image", "Loop not running at the desired rate of %d hz", loop_rate_);
    }

    // spin after the sleep so that we get the latest cloud messages
    ros::spinOnce();

    if (ros::Time::now() - latest_acc_complete_time_ > ros::Duration(accumulate_time_))
    {
      cv::Mat image_rgb(image_height_, image_width_, CV_8UC3);
      cv::Mat image_depth(image_height_, image_width_, CV_32F);
      analyzeAccumulatedPoints(image_rgb, image_depth);
      ROS_DEBUG_NAMED("assemble_image", "Size of data is %lu", acc_2d_points_.size());

      if (acc_2d_points_.empty())
      {
        ROS_WARN_NAMED("assemble_image", "No data accumulated... check your 3D sensor output!");
        continue;
      }

      if (save_rgb_image_path_.size() > 0)
      {
        cv::imwrite(save_rgb_image_path_, image_rgb);
      }

      // publish images - lazy
      cv_bridge::CvImage img_bridge;
      sensor_msgs::Image img_msg;  // message to be sent
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.frame_id = latest_cloud_.header.frame_id;
      if (image_rgb_pub_.getNumSubscribers() > 0)
      {
        img_bridge = cv_bridge::CvImage(header, "bgr8", image_rgb);
        img_bridge.toImageMsg(img_msg);
        image_rgb_pub_.publish(img_msg);
      }
      if (image_depth_pub_.getNumSubscribers() > 0)
      {
        img_bridge = cv_bridge::CvImage(header, "32FC1", image_depth);
        img_bridge.toImageMsg(img_msg);
        image_depth_pub_.publish(img_msg);
      }

      reset();
    }
    else
    {
      accumulateImage();
    }
  }
}

void AssembleImage::analyzeAccumulatedPoints(cv::Mat& rgb_image, cv::Mat& depth_image)
{
  latest_acc_complete_time_ = ros::Time::now();

  // reset RGB uint8 image
  rgb_image.setTo(cv::Scalar(0, 0, 0));
  ROS_DEBUG_NAMED("assemble_image", "Bounds: min (x,y)=(%f,%f) ... max (x,y)=(%f,%f)", min_x_, min_y_, max_x_, max_y_);
  ROS_DEBUG_NAMED("assemble_image", "Bounds: min/max intensity = %d, %d", min_intensity_, max_intensity_);

  // reset depth float image
  depth_image.setTo(0.0);

  // discretize
  double x_step = (max_x_ - min_x_) / image_width_;
  double y_step = (max_y_ - min_y_) / image_height_;

  double intensity_step = (max_intensity_ - min_intensity_) / 255.0;
  ROS_DEBUG_NAMED("assemble_image", "Step size: intensity=%f, xstep=%f, ystep=%f", intensity_step, x_step, y_step);

  for (const image_assembler::PointDataXYDI& p : acc_2d_points_)
  {
    // Note: if 2 points are discretized to the same pixel, simply override
    int discretized_x = std::min(static_cast<int>(image_width_ - 1), continuousToDiscrete(p.x, x_step, min_x_));
    int discretized_y = std::min(static_cast<int>(image_height_ - 1), continuousToDiscrete(p.y, y_step, min_y_));

    if (discretized_x < 0 || discretized_x >= image_width_ || discretized_y < 0 || discretized_y >= image_height_)
    {
      ROS_WARN_NAMED("assemble_image", "point: (x,y) = (%f,%f)", p.x, p.y);
      ROS_ERROR_NAMED("assemble_image",
                      "Implementation error: discretized values are out of bounds: discretized (x,y)=(%d,%d) ... image "
                      "dimensions: (width,height) =(%d,%d)",
                      discretized_x, discretized_y, image_width_, image_height_);
      throw std::runtime_error("Implementation error: discretized values are out of bounds");
    }

    // set RGB image
    int rgb_val =
        static_cast<uint8_t>(continuousToDiscrete(p.intensity, intensity_step, static_cast<double>(min_intensity_)));

    if (rgb_val < 0 || rgb_val > 255)
    {
      ROS_ERROR_NAMED("assemble_image", "Implementation error: rgb value out of bounds: (rgb val)=(%d)", rgb_val);
      throw std::runtime_error("Implementation error: rgb value out of bounds");
    }
    rgb_image.at<cv::Vec3b>(discretized_y, discretized_x) = cv::Vec3b(rgb_val, rgb_val, rgb_val);

    // set depth image
    depth_image.at<float>(discretized_y, discretized_x) = p.depth;
  }
}

int AssembleImage::continuousToDiscrete(double val, double step, double min_bound)
{
  return static_cast<int>(std::round((val - min_bound) / step));
}

void AssembleImage::accumulateImage()
{
  // use latest point cloud if ready

  if (!cloud_ready_)
  {
    ROS_DEBUG_NAMED("assemble_image", "Cloud not ready - not accumulating.");
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PCLPointCloud2::Ptr pcl_cloud2_ptr(new pcl::PCLPointCloud2());
  pcl_conversions::toPCL(latest_cloud_, *pcl_cloud2_ptr);
  pcl::fromPCLPointCloud2(*pcl_cloud2_ptr, *cloud);

  for (const pcl::PointXYZI& point_3d : cloud->points)
  {
    // ignore bad data
    // todo: change y to z... post good transform
    if (std::isnan(point_3d.x) || std::isnan(point_3d.y) || std::isnan(point_3d.z) || point_3d.x <= 0.0)
    {
      continue;
    }

    image_assembler::PointDataXYDI new_point;
    new_point.x = 1000 * focal_point_ * -1 * point_3d.y / point_3d.x;  // x' = f*x/z
    new_point.y = 1000 * focal_point_ * -1 * point_3d.z / point_3d.x;  // y' = f*y/z
    new_point.depth = point_3d.x;
    new_point.intensity = point_3d.intensity;
    acc_2d_points_.push_back(new_point);

    if (point_3d.x < 0)
    {
      ROS_WARN_NAMED("assemble_image", "Negative depth data found!");
    }

    // update spatial bounds for discretizing later; once all 2d points are done being accumulated, discretize using the
    // bounds and the desired image dimensions
    min_x_ = std::min(min_x_, new_point.x);
    min_y_ = std::min(min_y_, new_point.y);
    max_x_ = std::max(max_x_, new_point.x);
    max_y_ = std::max(max_y_, new_point.y);
    ROS_DEBUG_NAMED("assemble_image", "Updated bounds: min (x,y) =(%f,%f) ... max (x,y) = (%f,%f)", min_x_, min_y_,
                    max_x_, max_y_);

    // update intensity bounds
    min_intensity_ = std::min(min_intensity_, new_point.intensity);
    max_intensity_ = std::max(max_intensity_, new_point.intensity);
  }

  cloud_ready_ = false;
}

}  // namespace image_assembler
