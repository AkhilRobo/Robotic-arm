# Robotic-arm
To get started generating features, launch the training.launch file to bring up the Gazebo environment. An empty environment should appear with only the sensor stick robot in the scene:

$ cd ~/catkin_ws
$ roslaunch sensor_stick training.launch
Keep an eye out for errors in the terminal and if Gazebo crashes or doesn't come up just give it another try, sometimes takes a few attempts!


Capturing Features
Next, in a new terminal, run the capture_features.py script to capture and save features for each of the objects in the environment. This script spawns each object in random orientations (default 5 orientations per object) and computes features based on the point clouds resulting from each of the random orientations.

$ cd ~/catkin_ws
$ rosrun sensor_stick capture_features.py


The features will now be captured and you can watch the objects being spawned in Gazebo. It should take 5-10 seconds for each random orientation (depending on your machine's resources). There are 7 objects total so it takes awhile to complete. When it finishes running you should have a training_set.sav file containing the features and labels for the dataset. Note: The training_set.sav file will be saved in your catkin_ws folder.

training.launch:

<launch>
  <!--Include description and control launch files-->
  <include file="$(find sensor_stick)/launch/robot_description.launch"/>
  <include file="$(find sensor_stick)/launch/robot_control.launch"/>

  <!--Launch a gazebo world-->
  <include file="$(find gazebo_ros)/launch/empty_world.launch"/>

  <!--spawn a robot in gazebo world-->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
  args="-urdf -param robot_description -x 0 -y 1.8 -z 0 -R 0 -P 0 -Y 0 -model sensor_stick"/>

  <!-- cloud transformer-->
  <node name="cloud_transformer" pkg="sensor_stick" type="cloud_transformer" respawn="false"/>

  <!-- The feature extractor node -->
  <node name="feature_extractor" pkg="sensor_stick" type="feature_extractor" respawn="false"/>

</launch>
cloud_transformer.cpp:

/*******************************************************************************
 * Copyright (C) 2017 Udacity Inc.
 *
 * This file is part of Robotic Arm: Pick and Place project for Udacity
 * Robotics nano-degree program
 *
 * All Rights Reserved.
 ******************************************************************************/

// Author: Harsh Pandya

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

/*
* Brief:
* This node transforms point cloud from /camera_link frame to /world frame
*/

class CloudTransformer
{
public:
  explicit CloudTransformer(ros::NodeHandle nh)
    : nh_(nh)
  {
    // Define Publishers and Subscribers here
    pcl_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &CloudTransformer::pclCallback, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sensor_stick/point_cloud", 1);

    buffer_.reset(new sensor_msgs::PointCloud2);
    buffer_->header.frame_id = "world";
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_pub_;
  tf::TransformListener listener_;
  sensor_msgs::PointCloud2::Ptr buffer_;

  void pclCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_msg)
  {
    listener_.waitForTransform("world", "camera_link", ros::Time::now(), ros::Duration(3.0));
    pcl_ros::transformPointCloud("world", *pcl_msg, *buffer_, listener_);
    pcl_pub_.publish(buffer_);
  }
};  // End of class CloudTransformer

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_tf");
  ros::NodeHandle nh;

  CloudTransformer tranform_cloud(nh);

  // Spin until ROS is shutdown
  while (ros::ok())
    ros::spin();

  return 0;
}

feature_extractor.cpp:

/*******************************************************************************
 * Copyright (C) 2017 Udacity Inc.
 *
 * This file is part of Robotic Arm: Pick and Place project for Udacity
 * Robotics nano-degree program
 *
 * All Rights Reserved.
 ******************************************************************************/

// Author: Brandon Kinman

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <sensor_msgs/PointCloud2.h>

#include <sensor_stick/GetNormals.h>
//#include <sensor_stick/GetFloatArrayFeature.h>

/*
* Brief:
* This node generates normal features for a point cloud
*/

class FeatureExtractor
{
public:
  explicit FeatureExtractor(ros::NodeHandle nh)
    : nh_(nh)
  {
    // Define Publishers and Subscribers here
    cluster_in_sub_ = nh_.subscribe("cluster_in", 1, &FeatureExtractor::clusterCallback, this);
    normals_out_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("normals_out", 1);
    get_normals_srv_ = nh_.advertiseService("get_normals", &FeatureExtractor::getNormalsReq, this);
    //get_vfh_srv_ = np_.advertiseService("get_vfh", &FeatureExtractor::getVFHReq, this);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber cluster_in_sub_;
  ros::Publisher normals_out_pub_;
  ros::ServiceServer get_normals_srv_;

  void clusterCallback(const sensor_msgs::PointCloud2& cloud_msg)
  {
    ROS_INFO("Cluster Received");

    pcl::PointCloud<pcl::PointXYZ> *p_cloud = new pcl::PointCloud<pcl::PointXYZ>();
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > sp_pcl_cloud(p_cloud);
    pcl::fromROSMsg(cloud_msg, *p_cloud);

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (sp_pcl_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Compute the features
    ne.compute(*cloud_normals);

    ROS_INFO("Done!");

    sensor_msgs::PointCloud2 normals_out_msg;
    pcl::toROSMsg(*cloud_normals, normals_out_msg);

    normals_out_pub_.publish(normals_out_msg);
  }

  bool getNormalsReq(sensor_stick::GetNormals::Request &req, sensor_stick::GetNormals::Response &rsp)
  {
    rsp.cluster = req.cluster;

    pcl::PointCloud<pcl::PointXYZ> *p_cloud = new pcl::PointCloud<pcl::PointXYZ>();
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > sp_pcl_cloud(p_cloud);
    pcl::fromROSMsg(req.cluster, *p_cloud);

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (sp_pcl_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Compute the features
    ne.compute(*cloud_normals);

    pcl::toROSMsg(*cloud_normals, rsp.cluster);

    return true;
  }

};  // FeatureExtractor

// bool getVFHReq(sensor_stick::GetNormals::Request &req, sensor_stick::GetNormals::Response &rsp)
// {
//   pcl::PointCloud<pcl::PointXYZ> *p_cloud = new pcl::PointCloud<pcl::PointXYZ>();
//   const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > sp_pcl_cloud(p_cloud);
//   pcl::fromROSMsg(req.cluster, *p_cloud);

//   // Create the normal estimation class, and pass the input dataset to it
//   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//   ne.setInputCloud (sp_pcl_cloud);
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
//   ne.setSearchMethod (tree);

//   // Use all neighbors in a sphere of radius 3cm
//   ne.setRadiusSearch(0.03);

//   // Output datasets
//   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

//   // Compute the features
//   ne.compute(*cloud_normals);

//   // Create the VFH estimation class, and pass the input dataset+normals to it
//   pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
//   vfh.setInputCloud(sp_pcl_cloud);
//   vfh.setInputNormals(*cloud_normals);
//   // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

//   // Create an empty kdtree representation, and pass it to the FPFH estimation object.
//   // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//   vfh.setSearchMethod (tree);

//   // Output datasets
//   pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

//   // Compute the features
//   vfh.compute (*vfhs);

//   for(size_t i = 0; i < 308; ++i)
//   {
//     vfhs.points[0].histogram[i];
//   }


// }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "feature_extractor");
  ros::NodeHandle nh("~");

  FeatureExtractor nfe(nh);

  // Spin until ROS is shutdown
  while (ros::ok())
    ros::spin();

  return 0;
}


capture_features.py:


#!/usr/bin/env python
import numpy as np
import pickle
import rospy

from sensor_stick.pcl_helper import *
from sensor_stick.training_helper import spawn_model
from sensor_stick.training_helper import delete_model
from sensor_stick.training_helper import initial_setup
from sensor_stick.training_helper import capture_sample
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.srv import GetNormals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


if __name__ == '__main__':
    rospy.init_node('capture_node')

    models = [\
       'beer',
       'bowl',
       'create',
       'disk_part',
       'hammer',
       'plastic_cup',
       'soda_can']

    # Disable gravity and delete the ground plane
    initial_setup()
    labeled_features = []

    for model_name in models:
        spawn_model(model_name)

        for i in range(5):
            # make five attempts to get a valid a point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                sample_cloud = capture_sample()
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()

                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected')
                    try_count += 1
                else:
                    sample_was_good = True

            # Extract histogram features
            chists = compute_color_histograms(sample_cloud, using_hsv=False)
            normals = get_normals(sample_cloud)
            nhists = compute_normal_histograms(normals)
            feature = np.concatenate((chists, nhists))
            labeled_features.append([feature, model_name])

        delete_model()


    pickle.dump(labeled_features, open('training_set.sav', 'wb'))



the above thing is the instructions i have,

Now my