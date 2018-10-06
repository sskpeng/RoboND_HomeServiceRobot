/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <nav_msgs/Odometry.h>

// Define pickup and drop off coordinates and threshold for distance between robot and pickup and dropoff coordinates 
#define PICKUP_X 3.5
#define PICKUP_Y 2.0
#define DROPOFF_X -3.8
#define DROPOFF_Y 4.7
#define THRESHOLD 0.5

// Initially the robot is picking up 
bool picking_up = false;
bool dropping_off = false;

ros::Publisher marker_pub;
visualization_msgs::Marker marker;


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

  float odom_x = msg->pose.pose.position.x;
  float odom_y = msg->pose.pose.position.y;

  float dx;
  float dy;
  float distance;

  if (picking_up) {

    dx = odom_x - PICKUP_X;
    dy = odom_y - PICKUP_Y;
    distance = std::sqrt(dx*dx+dy*dy);
    ROS_INFO("Moving to pickup location ... distance between robot and pickup = %f", distance);
    if (distance < THRESHOLD) {
      // we are at the pickup location
      ROS_INFO("Picking up object!");

      // Delete marker 
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);

      // Wait 5 seconds before trying to show other markers
      ros::Duration(5.0).sleep();

      //turn off pick up, turn on drop off
      picking_up = false;
      dropping_off = true;
      return;
    }

  } else if (dropping_off) {


    dx = odom_x - DROPOFF_X;
    dy = odom_y - DROPOFF_Y;
    distance = std::sqrt(dx*dx+dy*dy);
    ROS_INFO("Moving to dropoff location ... distance between robot and dropoff = %f", distance);

    if (distance < THRESHOLD) {
      // we are at the dropping location
      ROS_INFO("Dropping off object!");

      marker.pose.position.x = DROPOFF_X;
      marker.pose.position.y = DROPOFF_Y;

      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);

      ros::Duration(5.0).sleep();
      picking_up = false;
      dropping_off = false;
      return;
    }
  }

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set the frame ID and timestamp
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type to be a sphere
  uint32_t shape = visualization_msgs::Marker::CUBE;
  marker.type = shape;

  // Set the marker action (ADD)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = PICKUP_X;
  marker.pose.position.y = PICKUP_Y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker
  marker.scale.x = 0.35;
  marker.scale.y = 0.35;
  marker.scale.z = 0.35;

  // Set the color
  marker.color.r = 0.3f;
  marker.color.g = 0.3f;
  marker.color.b = 1.3f;
  marker.color.a = 0.8;
  
  // never auto-delete marker
  marker.lifetime = ros::Duration();

  while (marker_pub.getNumSubscribers() < 1) {
  	ros::Duration(1.0).sleep();
  }

  ROS_INFO("Adding marker ...");
  //marker_pub.publish(marker);
  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);
  
  picking_up = true;

  // subscribe to odometry topic
  ros::Subscriber odom_sub = n.subscribe("/odom", 1, odomCallback);


  ros::Rate r(10.0); // 10 Hz
  while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
  }

  return 0;

}
