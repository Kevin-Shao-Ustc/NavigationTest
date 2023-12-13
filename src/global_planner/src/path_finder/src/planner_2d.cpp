/*
Copyright (C) 2022 Hongkai Ye (kyle_yeh@163.com)
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#include "occ_grid/occ_map.h"
#include "path_finder/rrt_star.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

#include <nav_msgs/Path.h>

struct Config
{
    std::string map_frame_id;
    std::string robot_frame_id;
    std::string map_service_name;
    std::string goal_topic;
    std::string map_topic;
    std::string occ_map_topic;
    std::string map_metadata_topic;
    std::string path_topic;
    float inflation_radius;
    float request_position_x;
    float request_position_y;
    float request_length_x;
    float request_length_y;
    float position_error_edurance;
    float global_planner_time_inverval;
    int occ_threshold;

    Config(const ros::NodeHandle &nh_priv)
    {
    nh_priv.param<std::string>("map_frame_id", map_frame_id, "map");
    nh_priv.param<std::string>("robot_frame_id", robot_frame_id, "base_link");
    nh_priv.param<std::string>("map_service_name", map_service_name, "/map_service");
    nh_priv.param<std::string>("goal_topic", goal_topic, "/goal");
    nh_priv.param<std::string>("map_topic", map_topic, "/map");
    nh_priv.param<std::string>("occ_map_topic", occ_map_topic, "/occ_map");
    nh_priv.param<std::string>("map_metadata_topic", map_metadata_topic, "/map_metadata");
    nh_priv.param<std::string>("path_topic", path_topic, "/path");
    nh_priv.param<float>("inflation_radius", inflation_radius, 0.5);
    nh_priv.param<float>("request_position_x", request_position_x, 0.0);
    nh_priv.param<float>("request_position_y", request_position_y, 0.0);
    nh_priv.param<float>("request_length_x", request_length_x, 100.0);
    nh_priv.param<float>("request_length_y", request_length_y, 100.0);
    nh_priv.param<float>("position_error_edurance", position_error_edurance, 0.05);
    nh_priv.param<float>("global_planner_time_inverval", global_planner_time_inverval, 0.1);
    nh_priv.param<int>("occ_threshold", occ_threshold, 50);
    }
};

class Planner2d
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_;
    ros::Timer execution_timer_;
    ros::ServiceClient map_client_;
    ros::Publisher path_pub_;

    Config config_;
    
     tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    env::OccMap::Ptr env_ptr_;
    // std::shared_ptr<visualization::Visualization> vis_ptr_;
    shared_ptr<path_plan::RRTStar> rrt_star_ptr_;

    Eigen::Vector2d start_, goal_;

    bool todo_reach_goal_ = false, todo_reset_rrt_tree_ = false;

public:
    Planner2d(const ros::NodeHandle &nh, const Config &conf) : nh_(nh), config_(conf), tf_buffer(ros::Duration(10)), tf_listener(tf_buffer)
    {
        env_ptr_.reset(new env::OccMap);
        env_ptr_->init(nh_, conf.map_topic, conf.map_metadata_topic, conf.inflation_radius, conf.occ_threshold, conf.occ_map_topic);

        // vis_ptr_ = std::make_shared<visualization::Visualization>(nh_);

        rrt_star_ptr_.reset(new path_plan::RRTStar(nh_, env_ptr_));
        // rrt_star_ptr_->setVisualizer(vis_ptr_);
        path_pub_ = nh_.advertise<nav_msgs::Path>(conf.path_topic, 5);

        goal_sub_ = nh_.subscribe(conf.goal_topic, 1, &Planner2d::goalCallback, this);
        execution_timer_ = nh_.createTimer(ros::Duration(0.005), &Planner2d::executionCallback, this);
        map_client_ = nh_.serviceClient<nav_msgs::GetMap>(config_.map_service_name);

        start_.setZero();
    }
    ~Planner2d(){};

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal_msg)
    {
        float goal_diff = (goal_[0] - goal_msg->pose.position.x) * (goal_[0] - goal_msg->pose.position.x) + (goal_[1] - goal_msg->pose.position.y) * (goal_[1] - goal_msg->pose.position.y);
        if (goal_diff < 0.0001)
        {
            return;
        }
        todo_reset_rrt_tree_ = true;
        goal_[0] = goal_msg->pose.position.x;
        goal_[1] = goal_msg->pose.position.y;
        todo_reach_goal_ = true;
        ROS_INFO_STREAM("\n-----------------------------\ngoal rcved at " << goal_.transpose());
    }

    void executionCallback(const ros::TimerEvent &event)
    {
        if (!env_ptr_->mapValid() || !todo_reach_goal_)
        {
            // map not ready or no goal set, skip
            return;
        }
        // listen to tf of the robot
        tf2::Transform tf_robot;
        try
        {
            geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(config_.map_frame_id, config_.robot_frame_id, ros::Time(0));
            tf2::fromMsg(transformStamped.transform, tf_robot);
            start_[0] = tf_robot.getOrigin().getX();
            start_[1] = tf_robot.getOrigin().getY();
            // ROS_INFO_STREAM("robot pose: " << start_.transpose());
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
        if ((goal_ - start_).norm() < config_.position_error_edurance)
        {
            todo_reach_goal_ = false;
        }

        // use the goal point as the root and use the start point as the nominal "goal point"
        todo_reset_rrt_tree_ = env_ptr_->checkIfMapUpdated();
        bool rrt_star_res = rrt_star_ptr_->plan(goal_, start_, &todo_reset_rrt_tree_);
        if (rrt_star_res)
        {
            vector<vector<Eigen::Vector2d>> routes = rrt_star_ptr_->getAllPaths();
            // vis_ptr_->visualize_path_list(routes, "rrt_star_paths", visualization::blue);
            vector<Eigen::Vector2d> final_path = rrt_star_ptr_->getPath();
            // vis_ptr_->visualize_path(final_path, "rrt_star_final_path");
            // vis_ptr_->visualize_pointcloud(final_path, "rrt_star_final_wpts");
            vector<std::pair<double, double>> slns = rrt_star_ptr_->getSolutions();

            nav_msgs::Path path_msg;
            path_msg.header.frame_id = config_.map_frame_id;
            path_msg.header.stamp = ros::Time::now();
            for (auto &p : final_path)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = config_.map_frame_id;
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = p[0];
                pose.pose.position.y = p[1];
                pose.pose.position.z = 0;
                pose.pose.orientation.x = 0;
                pose.pose.orientation.y = 0;
                pose.pose.orientation.z = 0;
                pose.pose.orientation.w = 1;
                path_msg.poses.push_back(pose);
            }
            // reverse the path
            std::reverse(path_msg.poses.begin(), path_msg.poses.end());
            path_pub_.publish(path_msg);
            // ROS_INFO_STREAM("[RRT*] final path len: " << slns.back().first);
            // start_ = goal_;
        }

    };
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_finder_2d_node");
    ros::NodeHandle nh("~");

    Planner2d tester(nh, Config(ros::NodeHandle("~")));

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}