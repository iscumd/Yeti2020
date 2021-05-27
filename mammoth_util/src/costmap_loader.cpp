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

#include <functional>
#include <chrono>
#include <string>

#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <costmap_loader.hpp>

namespace mammoth_util
{
CostmapLoader::CostmapLoader(rclcpp::NodeOptions options)
: Node("costmap_loader", options)
{
  // params
  this->declare_parameter<std::string>("costmap_filepath", "costmap.pgm");
  this->get_parameter("my_parameter", costmap_filepath_param_);

  // load image into cv::Mat
  costmap_image_ = cv::imread(costmap_filepath_param_, cv::IMREAD_GRAYSCALE);
  costmap_image_.convertTo(costmap_image_, CV_8U);
  cv::normalize(costmap_image_, costmap_image_, 0, 255, cv::NORM_MINMAX);
  costmap_load_time_ = this->get_clock()->now();
  costmap_width_ = costmap_image_.size().width;
  costmap_height_ = costmap_image_.size().height;
  costmap_resolution_ = costmap_width_ * costmap_height_;

  // set up pubs/subs
  costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", 10);
  using namespace std::chrono_literals; // this is needed for the 500ms thing
  costmap_timer_ = this->create_wall_timer(
    500ms, std::bind(&CostmapLoader::costmap_timer_callback, this));
}

void CostmapLoader::costmap_timer_callback()
{
  auto loaded_map = nav_msgs::msg::OccupancyGrid();
  auto origin = geometry_msgs::msg::Pose();
  
  //
  // populate message
  //
  loaded_map.header.stamp = this->get_clock()->now();
  loaded_map.header.frame_id = "/map";

  loaded_map.info.map_load_time = costmap_load_time_;
  loaded_map.info.width = costmap_width_;
  loaded_map.info.height = costmap_height_;
  loaded_map.info.resolution = costmap_resolution_;

  origin.position.x = origin.position.y = origin.position.z = 0; // set all to 0
  origin.orientation.x = origin.orientation.y = origin.orientation.z = 0; // set all but w to 0
  origin.orientation.w = 1;
  loaded_map.info.origin = origin;
  
  for (int i = 0; i < costmap_image_.rows; ++i)
  {
    for (int j = 0; j < costmap_image_.cols; ++j)
    {
      // i*cols + j is row major ordering single dimensional array indexing
      loaded_map.data[i*costmap_image_.cols + j] = costmap_image_.at<uchar>(i, j); // fill in data from cv::mat
    }
  }
  //
  // end populate message
  //

  costmap_publisher_->publish(loaded_map);
}

}  // namespace mammoth_util

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto lp_node = std::make_shared<mammoth_util::CostmapLoader>(options);
  exec.add_node(lp_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
