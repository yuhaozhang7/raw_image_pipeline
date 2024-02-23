//
// Copyright (c) 2021-2023, ETH Zurich, Robotic Systems Lab, Matias Mattamala. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <raw_image_pipeline_ros/raw_image_pipeline_ros.hpp>


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("raw_image_pipeline_ros");
  rclcpp::Node::SharedPtr nh_priv = nh; // rclcpp::Node::make_shared("...");

  raw_image_pipeline::RawImagePipelineRos image_proc(nh, nh_priv);
  image_proc.run();
  
  // rclcpp::waitForShutdown();
  return 0;
}
