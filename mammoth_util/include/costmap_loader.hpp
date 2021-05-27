// MIT License
//
// Copyright (c) 2021 David Cutting
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef MAMMOTH_UTIL__COSTMAP_LOADER_HPP_
#define MAMMOTH_UTIL__COSTMAP_LOADER_HPP_

#include <memory>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace mammoth_util
{
class CostmapLoader : public rclcpp::Node
{
public:
  explicit CostmapLoader(rclcpp::NodeOptions options);

private:
  // params
  std::string costmap_filepath_param_;

  // vars
  cv::Mat costmap_image_;
  rclcpp::Time costmap_load_time_;
  int costmap_width_;
  int costmap_height_;
  int costmap_resolution_;

  // pubs subs
  rclcpp::TimerBase::SharedPtr costmap_timer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;

  //callbacks
  void costmap_timer_callback();
};
}  // namespace mammoth_util

#endif  // MAMMOTH_UTIL__COSTMAP_LOADER_HPP_
