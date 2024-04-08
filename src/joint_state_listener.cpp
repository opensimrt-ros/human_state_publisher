/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Wim Meeussen */

#include <algorithm>
#include <map>
#include <string>

#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "human_state_publisher/human_state_publisher.h"
#include "human_state_publisher/joint_state_listener.h"
#include "std_msgs/Float64.h"

using namespace human_state_publisher;

JointStateListener::JointStateListener() : JointStateListener(KDL::Tree(), MimicMap())
{
}

JointStateListener::JointStateListener(const KDL::Tree& tree, const MimicMap& m, const urdf::Model& model)
	: JointStateListener(std::make_shared<HumanStatePublisher>(tree, model), m)
{
}

JointStateListener::JointStateListener(const std::shared_ptr<HumanStatePublisher>& rsp, const MimicMap& m)
	: state_publisher_(rsp), mimic_(m)
{
	ros::NodeHandle n_tilde("~");
	ros::NodeHandle n;

	//for (int ii= 0;ii<19;ii++)
	//	pubpub.push_back( n.advertise<std_msgs::Float64>("some_joint"+std::to_string(ii),1)); 
	n_tilde.param("use_tf_static", use_tf_static_, true);
	joint_state_sub_ = n.subscribe("joint_states", 1, &JointStateListener::callbackJointState, this);

	// trigger to publish fixed joints
	// if using static transform broadcaster, this will be a oneshot trigger and only run once
	publish_interval_ = ros::Duration(0.1);
	timer_ = n_tilde.createTimer(publish_interval_, &JointStateListener::callbackFixedJoint, this, use_tf_static_);
}


JointStateListener::~JointStateListener()
{}

std::string JointStateListener::getTFPrefix()
{
	ros::NodeHandle n_tilde("~");
	std::string tf_prefix;

	// get the tf_prefix parameter from the closest namespace
	std::string tf_prefix_key;
	n_tilde.searchParam("tf_prefix", tf_prefix_key);
	n_tilde.param(tf_prefix_key, tf_prefix, std::string(""));

	return tf_prefix;
}

void JointStateListener::callbackFixedJoint(const ros::TimerEvent& e)
{
	(void)e;
	state_publisher_->publishFixedTransforms(getTFPrefix(), use_tf_static_);
}

void JointStateListener::callbackJointState(const JointStateConstPtr& state)
{
	//auto t00 = ros::Time::now();

	//ROS_DEBUG("Publishing transforms for moving joints");

	// get joint positions from state message
	std::map<std::string, double> joint_positions;
	for (size_t i = 0; i < state->name.size(); ++i) {
		joint_positions.insert(make_pair(state->name[i], state->position[i]));
		//std_msgs::Float64 a_msg;
		//a_msg.data = state->position[i];
		//pubpub[i].publish(a_msg);
	}

	state_publisher_->publishTransforms(joint_positions, state->header.stamp, getTFPrefix());

	//auto t11 =ros::Time::now();
	//ROS_INFO_STREAM("\rRATE: " << 1.0/(t11 -t00).toSec());

}
