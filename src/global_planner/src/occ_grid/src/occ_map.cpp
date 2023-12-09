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
  inline void OccMap::setOccupancy(const Eigen::Vector2d &pos)
  {
    Eigen::Vector2i id;
    posToIndex(pos, id);
    // cout << "id: " << id.transpose() << ", idx: " << idxToAddress(id) << ", is in map? " << isInMap(id) << endl;
    if (!isInMap(id))
      return;

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

    fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), false);

    Eigen::Vector2d p2d;
    for (int x = 0; x < msg->info.width; ++x)
      for (int y = 0; y < msg->info.height; ++y)
      {
        if (msg->data[x + y * msg->info.width] == 100)
        {
          p2d(0) = msg->info.origin.position.x + x * msg->info.resolution;
          p2d(1) = msg->info.origin.position.y + y * msg->info.resolution;
          this->setOccupancy(p2d);
        }
      }

    is_global_map_valid_ = true;
  }

  void OccMap::globalMapMetadataCallback( const nav_msgs::MapMetaDataPtr &msg)
  {
    if (is_global_map_metadata_valid_)
      return;
    
    this->origin_ = Eigen::Vector2d(msg->origin.position.x, msg->origin.position.y);
    this->map_size_ = Eigen::Vector2d(msg->width * msg->resolution, msg->height * msg->resolution);
    this->resolution_ = msg->resolution;
    this->resolution_inv_ = 1 / this->resolution_;

    for (int i = 0; i < 3; ++i)
    {
      this->grid_size_(i) = ceil(map_size_(i) * resolution_inv_);
    }

    min_range_ = origin_;
    max_range_ = origin_ + map_size_;
    
    // initialize size of buffer
    int buffer_size = grid_size_(0) * grid_size_(1);
    occupancy_buffer_.resize(buffer_size);
    fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), false);

    // set x-y boundary occ
    for (double cx = min_range_[0] + resolution_ / 2; cx <= max_range_[0] - resolution_ / 2; cx += resolution_)
    {
      this->setOccupancy(Eigen::Vector2d(cx, min_range_[1] + resolution_ / 2));
      this->setOccupancy(Eigen::Vector2d(cx, max_range_[1] - resolution_ / 2));
    }
    for (double cy = min_range_[1] + resolution_ / 2; cy <= max_range_[1] - resolution_ / 2; cy += resolution_)
    {
      this->setOccupancy(Eigen::Vector2d(min_range_[0] + resolution_ / 2, cy));
      this->setOccupancy(Eigen::Vector2d(max_range_[0] - resolution_ / 2, cy));
    }

    is_global_map_metadata_valid_ = true;
    cout << "map meta data initialized: " << endl;
  }

  void OccMap::init(const ros::NodeHandle &nh, const std::string &map_topic, const std::string &map_metadata_topic, const float &inflation_radius)
  {
    node_ = nh;
    is_global_map_valid_ = false;
    is_global_map_metadata_valid_ = false;
    inflation_radius_ = inflation_radius;

    global_map_sub_ = node_.subscribe(map_topic, 1, &OccMap::globalMapCallback, this);
    global_map_metadata_sub_ = node_.subscribe(map_metadata_topic, 1, &OccMap::globalMapMetadataCallback, this);
    cout << "map initialized: " << endl;
  }
} // namespace env
