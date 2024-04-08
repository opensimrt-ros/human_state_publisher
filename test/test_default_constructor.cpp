/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Open Source Robotics Foundation, Inc.
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


#include <string>
#include <utility>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "human_state_publisher/joint_state_listener.h"

TEST(HumanStatePublisher, assignment)
{
  human_state_publisher::HumanStatePublisher rsp;

  urdf::Model model;
  KDL::Tree tree;
  rsp = std::move(human_state_publisher::HumanStatePublisher(tree, model));
}

TEST(JointStateListener, assignment)
{
  human_state_publisher::JointStateListener jsl;

  urdf::Model model;
  KDL::Tree tree;

  jsl = std::move(human_state_publisher::JointStateListener(
    tree, human_state_publisher::MimicMap(), model));
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_default_constructor");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

