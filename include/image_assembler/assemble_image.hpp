/**
 * @file assemble_image.cpp
 * @author Michael Kaca (dabestineuro@gmail.com)
 * @brief Tool for converting XYZI Cloud data to an RGB image based on the intensity
 * @version 0.1
 * @date 2023-07-16
 *
 * Note: this is only for stationary robots!
 * Note: uses openCV xy image standards: origin is at the top left corner, with x being horizontal, and y being vertical
 *
 * @copyright Copyright (c) 2023
 *
 *  Software License Agreement (BSD License)
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

#ifndef IMAGE_ASSEMBLER_IMAGE_ASSEMBLER_HPP_
#define IMAGE_ASSEMBLER_IMAGE_ASSEMBLER_HPP_

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_assembler/point_data_xydi.hpp>

namespace image_assembler
{

  class AssembleImage
  {
  public:
    AssembleImage();

    /**
     * @brief Publishes the assembled image(s)
     */
    void loop();

  private:
    ros::Publisher image_rgb_pub_;
    ros::Publisher image_depth_pub_;
    ros::Subscriber cloud_sub_;

    bool cloud_ready_;
    ros::Time latest_acc_complete_time_;

    sensor_msgs::Image acc_rgb_image_;
    sensor_msgs::PointCloud2 latest_cloud_;
    std::vector<image_assembler::PointDataXYDI> acc_2d_points_;
    double min_x_, min_y_, max_x_, max_y_;
    unsigned int min_intensity_, max_intensity_;

    // params
    double accumulate_time_;
    double focal_point_;
    int image_width_, image_height_;
    int loop_rate_;
    std::string save_rgb_image_path_;

    /**
     * @brief Callback for the input cloud
     */
    void cloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg);

    /**
     * @brief Accumulates the point cloud data into XYDI points, while updating the bounds for future discretization
     */
    void accumulateImage();

    /**
     * @brief Resets the bounds and accumulated data
     */
    void reset();

    /**
     * @brief Constructs the depth and rgb images based on the accumulated data
     *
     * @param rgb_image RGB Image returned by reference
     * @param depth_image Depth image returned by reference
     */
    void analyzeAccumulatedPoints(cv::Mat &rgb_image, cv::Mat &depth_image);

    /**
     * @brief Converts continuous values to discrete values
     * @param val Input value (continuous)
     * @param step desired step size
     * @param min_bound min discretized value
     * @return int Resultant discretized value
     */
    int continuousToDiscrete(double val, double step, double min_bound);
  };

} // namespace image_assembler

#endif // IMAGE_ASSEMBLER_IMAGE_ASSEMBLER_HPP_
