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
#include "occ_grid/raycast.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <random>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

namespace env
{
  inline void OccMap::setOccupancy(Eigen::Vector2i &id)
  {
    // cout << "id: " << id.transpose() << ", idx: " << idxToAddress(id) << ", is in map? " << isInMap(id) << endl;
    if (!isInMap(id))
      return;

    Eigen::Vector2d pos;
    indexToPos(id, pos);

    occupancy_buffer_[idxToAddress(id)] = true;

    // set inflation
    if (inflation_radius_ > 0)
    {
      Eigen::Vector2i id_inflation;
      Eigen::Vector2d pos_inflation;
      int inflation_radius_id = ceil(inflation_radius_ * resolution_inv_);
      for (int x = -inflation_radius_id; x <= inflation_radius_id; ++x)
        for (int y = -inflation_radius_id; y <= inflation_radius_id; ++y)
        {
          id_inflation = id + Eigen::Vector2i(x, y);
          if (!isInMap(id_inflation))
            continue;
          indexToPos(id_inflation, pos_inflation);
          if ((pos_inflation - pos).norm() <= inflation_radius_)
            occupancy_buffer_[idxToAddress(id_inflation)] = true;
        }
    }
  }
  // {
  //   Eigen::Vector2i id;
  //   posToIndex(pos, id);
  //   // cout << "id: " << id.transpose() << ", idx: " << idxToAddress(id) << ", is in map? " << isInMap(id) << endl;
  //   if (!isInMap(id))
  //     return;

  //   occupancy_buffer_[idxToAddress(id)] = true;
  // }

  void OccMap::globalMapCallback(const nav_msgs::OccupancyGridPtr &msg)
  {
    if (!is_global_map_metadata_valid_)
      return;
    is_global_map_valid_ = false;
    fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), false);

    Eigen::Vector2i id;
    // for (int x = 0; x < (int)msg->info.width; ++x)
    //   for (int y = 0; y < (int)msg->info.height; ++y)
    //   {
    //     if (msg->data[x + y * msg->info.width] == 100)
    //     {
    //       id << x, y;
    //       this->setOccupancy(id);
    //     }
    //   }
    for (int x = 0; x <= msg->info.width; ++x)
      for (int y = 0; y <= msg->info.height; ++y)
      {
        id << x, y;
        if (!isInMap(id))
        {
          continue;
          cout << "id: " << id.transpose() << ", idx: " << idxToAddress(id) << ", is in map? " << isInMap(id) << endl;
        }
        if (msg->data[x + y * msg->info.width] >= occ_threshold_)
        {
            this->setOccupancy(id);
            
        }
        if (msg->data[x + y * msg->info.width] != -1 && msg->data[x + y * msg->info.width] != 0 && msg->data[x + y * msg->info.width] != 100)
        {
            cout << "point at" << id.transpose() << "has likelyhood" << msg->data[x + y * msg->info.width] << endl;
        }
      }
    
      // publish the occupancy map
      nav_msgs::OccupancyGrid occ_map_msg;
      occ_map_msg.header.frame_id = msg->header.frame_id;
      occ_map_msg.header.stamp = ros::Time::now();
      occ_map_msg.info.resolution = msg->info.resolution;
      occ_map_msg.info.width = msg->info.width;
      occ_map_msg.info.height = msg->info.height;
      occ_map_msg.info.origin.position.x = msg->info.origin.position.x;
      occ_map_msg.info.origin.position.y = msg->info.origin.position.y;
      occ_map_msg.info.origin.position.z = 0;
      occ_map_msg.info.origin.orientation.x = 0;
      occ_map_msg.info.origin.orientation.y = 0;
      occ_map_msg.info.origin.orientation.z = 0;
      occ_map_msg.info.origin.orientation.w = 1;
      occ_map_msg.data.resize(occ_map_msg.info.width * occ_map_msg.info.height);
      for (int i = 0; i < occ_map_msg.info.width; ++i)
      {
          for (int j = 0; j < occ_map_msg.info.height; ++j)
          {
              occ_map_msg.data[i + j * occ_map_msg.info.width] = occupancy_buffer_[i + j * occ_map_msg.info.width] ? 100 : 0;
          }
      }
      occ_map_pub_.publish(occ_map_msg);

    is_global_map_valid_ = true;
    todo_reset_rrt_tree = true;
  }

  void OccMap::globalMapMetadataCallback( const nav_msgs::MapMetaDataPtr &msg)
  {
    if (is_global_map_metadata_valid_)
      return;
    
    this->origin_ = Eigen::Vector2d(msg->origin.position.x, msg->origin.position.y);
    this->map_size_ = Eigen::Vector2d(msg->width * msg->resolution, msg->height * msg->resolution);
    this->resolution_ = msg->resolution;
    this->resolution_inv_ = 1 / this->resolution_;

    this->grid_size_ = Eigen::Vector2i(msg->width, msg->height);

    min_range_ = origin_;
    max_range_ = origin_ + map_size_;
    
    // initialize size of buffer
    int buffer_size = grid_size_(0) * grid_size_(1);
    occupancy_buffer_.resize(buffer_size);
    fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), false);

    is_global_map_metadata_valid_ = true;
    cout << "map meta data initialized: " << endl;
  }

  void OccMap::init(const ros::NodeHandle &nh, const std::string &map_topic, const std::string &map_metadata_topic, const float &inflation_radius, const int &occ_threshold, std::string occ_map_topic)
  {
    node_ = nh;
    is_global_map_valid_ = false;
    is_global_map_metadata_valid_ = false;
    todo_reset_rrt_tree = false;
    inflation_radius_ = inflation_radius;
    occ_threshold_ = occ_threshold;

    occ_map_pub_ = node_.advertise<nav_msgs::OccupancyGrid>(occ_map_topic, 5);
    global_map_sub_ = node_.subscribe(map_topic, 1, &OccMap::globalMapCallback, this);
    global_map_metadata_sub_ = node_.subscribe(map_metadata_topic, 1, &OccMap::globalMapMetadataCallback, this);
    cout << "map initialized: " << endl;
  }
} // namespace env
