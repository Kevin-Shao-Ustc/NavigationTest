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
#ifndef RRT_STAR_H
#define RRT_STAR_H

#include "occ_grid/occ_map.h"
#include "sampler.h"
#include "node.h"
#include "kdtree.h"

#include <zip.h>

#include <ros/ros.h>
#include <utility>
#include <queue>


namespace path_plan
{
  class RRTStar
  {
  public:
    RRTStar(){};
    RRTStar(const ros::NodeHandle &nh, const env::OccMap::Ptr &mapPtr) : nh_(nh), map_ptr_(mapPtr)
    {
      nh_.param("RRT_Star/steer_length", steer_length_, 0.5);
      nh_.param("RRT_Star/search_radius", search_radius_, 1.0);
      nh_.param("RRT_Star/search_time", search_time_, 0.1);
      nh_.param("RRT_Star/max_tree_node_nums", max_tree_node_nums_, 1000);
      ROS_WARN_STREAM("[RRT*] param: steer_length: " << steer_length_);
      ROS_WARN_STREAM("[RRT*] param: search_radius: " << search_radius_);
      ROS_WARN_STREAM("[RRT*] param: search_time: " << search_time_);
      ROS_WARN_STREAM("[RRT*] param: max_tree_node_nums: " << max_tree_node_nums_);

      valid_tree_node_nums_ = 0;
    }
    ~RRTStar(){};

    bool plan(const Eigen::Vector2d &s, const Eigen::Vector2d &g, bool *todo_reset_rrt_tree)
    {
      if (!map_ptr_->isStateValid(s))
      {
        ROS_ERROR("[RRT*]: Start pos collide or out of bound");
        return false;
      }
      if (!map_ptr_->isStateValid(g))
      {
        ROS_ERROR("[RRT*]: Goal pos collide or out of bound");
        return false;
      }
      if (*todo_reset_rrt_tree)
      {
        reset();
        /* construct start and goal nodes */
        start_node_ = new TreeNode;
        start_node_->x = s;
        start_node_->cost_from_start = 0.0;
        goal_node_ = new TreeNode;
        goal_node_->x = g;
        goal_node_->cost_from_start = DBL_MAX; // important
        valid_tree_node_nums_ = 2;             // put start and goal in tree
        goal_found = false;
        /* kd tree init */
        kd_tree = kd_create(2);
        //Add start and goal nodes to kd tree
        kd_insert2(kd_tree, start_node_->x[0], start_node_->x[1], start_node_);
        *todo_reset_rrt_tree = false;
      }
      else
      {
        // update the goal node
        goal_node_->x = g;
        // if path not found, continue to search
        // else, rewire the goal node first
        if (goal_found)
        {
          // record the last cost
          float best_cost_before_rewire = goal_node_->cost_from_start;
          /* find parent for the new goal*/
          vector<RRTNode2DPtr> neighbour_nodes;
          std::vector<bool> is_valid_edge;
          find_parent(goal_node_->x, goal_node_->parent, neighbour_nodes, is_valid_edge, goal_node_, true);
          // check if a better path is generated after rewiring
          float epsilon = 0.00001;
          if (best_cost_before_rewire > goal_node_->cost_from_start + epsilon)
          {
            // update the best path
            vector<Eigen::Vector2d> curr_best_path;
            fillPath(goal_node_, curr_best_path);
            path_list_.emplace_back(curr_best_path);
            solution_cost_time_pair_list_.emplace_back(goal_node_->cost_from_start, 0);
          }
        }
      }

      // ROS_INFO("[RRT*]: RRT starts planning a path");
      float time_cost = rrt_star(s, g);

      if (goal_found)
      {
        final_path_use_time_ = time_cost;
        fillPath(goal_node_, final_path_);
        // ROS_INFO_STREAM("[RRT*]: first path length: " << solution_cost_time_pair_list_.front().first << ", use_time: " << first_path_use_time_);
      }
      else if (valid_tree_node_nums_ == max_tree_node_nums_)
      {
        ROS_ERROR_STREAM("[RRT*]: NOT CONNECTED TO GOAL after " << max_tree_node_nums_ << " nodes added to rrt-tree");
      }
      else
      {
        ROS_ERROR_STREAM("[RRT*]: NOT CONNECTED TO GOAL after " << time_cost << " seconds");
      }
      return goal_found;
    }

    vector<Eigen::Vector2d> getPath()
    {
      return final_path_;
    }

    vector<vector<Eigen::Vector2d>> getAllPaths()
    {
      return path_list_;
    }

    vector<std::pair<double, double>> getSolutions()
    {
      return solution_cost_time_pair_list_;
    }

    // void setVisualizer(const std::shared_ptr<visualization::Visualization> &visPtr)
    // {
    //   vis_ptr_ = visPtr;
    // };

  private:
    // nodehandle params
    ros::NodeHandle nh_;

    BiasSampler sampler_;

    double steer_length_;
    double search_radius_;
    double search_time_;
    int max_tree_node_nums_;
    int valid_tree_node_nums_;
    double first_path_use_time_;
    double final_path_use_time_;

    std::map<float, RRTNode2DPtr> cost2node_map_;
    TreeNode *start_node_;
    TreeNode *goal_node_;
    vector<Eigen::Vector2d> final_path_;
    vector<vector<Eigen::Vector2d>> path_list_;
    vector<std::pair<double, double>> solution_cost_time_pair_list_;

    // environment
    env::OccMap::Ptr map_ptr_;
    // std::shared_ptr<visualization::Visualization> vis_ptr_;

    bool isSamplerValid_ = false;
    bool goal_found = false;

    kdtree *kd_tree;

    void reset()
    {
      final_path_.clear();
      path_list_.clear();
      solution_cost_time_pair_list_.clear();
      valid_tree_node_nums_ = 0;
    }

    double calDist(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
    {
      return (p1 - p2).norm();
    }

    Eigen::Vector2d steer(const Eigen::Vector2d &nearest_node_p, const Eigen::Vector2d &rand_node_p, double len)
    {
      Eigen::Vector2d diff_vec = rand_node_p - nearest_node_p;
      double dist = diff_vec.norm();
      if (diff_vec.norm() <= len)
        return rand_node_p;
      else
        return nearest_node_p + diff_vec * len / dist;
    }

    RRTNode2DPtr addTreeNode(RRTNode2DPtr &parent, const Eigen::Vector2d &state,
                             const double &cost_from_start, const double &cost_from_parent)
    {
      RRTNode2DPtr new_node_ptr = new TreeNode;
      cost2node_map_.insert(std::make_pair(cost_from_start, new_node_ptr));
      valid_tree_node_nums_++;
      new_node_ptr->parent = parent;
      parent->children.push_back(new_node_ptr);
      new_node_ptr->x = state;
      new_node_ptr->cost_from_start = cost_from_start;
      new_node_ptr->cost_from_parent = cost_from_parent;
      return new_node_ptr;
    }

    void changeNodeParent(RRTNode2DPtr &node, RRTNode2DPtr &parent, const double &cost_from_parent)
    {
      if (node->parent)
        node->parent->children.remove(node); //DON'T FORGET THIS, remove it form its parent's children list
      node->parent = parent;
      node->cost_from_parent = cost_from_parent;
      node->cost_from_start = parent->cost_from_start + cost_from_parent;
      parent->children.push_back(node);

      // for all its descedants, change the cost_from_start and tau_from_start;
      RRTNode2DPtr descendant(node);
      std::queue<RRTNode2DPtr> Q;
      Q.push(descendant);
      while (!Q.empty())
      {
        descendant = Q.front();
        Q.pop();
        for (const auto &leafptr : descendant->children)
        {
          leafptr->cost_from_start = leafptr->cost_from_parent + descendant->cost_from_start;
          Q.push(leafptr);
        }
      }
    }

    void fillPath(const RRTNode2DPtr &n, vector<Eigen::Vector2d> &path)
    {
      path.clear();
      RRTNode2DPtr node_ptr = n;
      while (node_ptr->parent)
      {
        path.push_back(node_ptr->x);
        node_ptr = node_ptr->parent;
      }
      path.push_back(start_node_->x);
      std::reverse(std::begin(path), std::end(path));
    }

    float rrt_star(const Eigen::Vector2d &s, const Eigen::Vector2d &g)
    {
      ros::Time rrt_start_time = ros::Time::now();

      /* main loop */
      int idx = 0;
      for (idx = 0; (ros::Time::now() - rrt_start_time).toSec() < search_time_; ++idx)
      {
        /* biased random sampling */
        Eigen::Vector2d x_rand;
        /* TODO: modify the sample function for heuristic sampling*/
        if (this->map_ptr_->mapValid() && !isSamplerValid_)
        {
          sampler_.setSamplingRange(map_ptr_->getOrigin(), map_ptr_->getMapSize());
          isSamplerValid_ = true;
        }
        sampler_.samplingOnce(x_rand, goal_found, goal_node_->cost_from_start, start_node_->x, goal_node_->x);
        // samplingOnce(x_rand);
        if (!map_ptr_->isStateValid(x_rand))
        {
          continue;
        }

        struct kdres *p_nearest = kd_nearest2(kd_tree, x_rand[0], x_rand[1]);
        if (p_nearest == nullptr)
        {
          ROS_ERROR("nearest query error");
          continue;
        }
        RRTNode2DPtr nearest_node = (RRTNode2DPtr)kd_res_item_data(p_nearest);
        kd_res_free(p_nearest);

        Eigen::Vector2d x_new = steer(nearest_node->x, x_rand, steer_length_);
        if (!map_ptr_->isSegmentValid(nearest_node->x, x_new))
        {
          continue;
        }
        /* 1. find parent */
        vector<RRTNode2DPtr> neighbour_nodes;
        std::vector<bool> is_valid_edge;
        RRTNode2DPtr new_node(nullptr);
        find_parent(x_new, nearest_node, neighbour_nodes, is_valid_edge, new_node, false);

        /* 2. try to connect to goal if possible */
        double dist_to_goal = calDist(x_new, goal_node_->x);
        if (dist_to_goal <= search_radius_)
        {
          bool is_connected2goal = map_ptr_->isSegmentValid(x_new, goal_node_->x);
          // this test can be omitted if sample-rejction is applied
          bool is_better_path = goal_node_->cost_from_start > dist_to_goal + new_node->cost_from_start;
          if (is_connected2goal && is_better_path)
          {
            if (!goal_found)
            {
              first_path_use_time_ = (ros::Time::now() - rrt_start_time).toSec();
            }
            goal_found = true;
            changeNodeParent(goal_node_, new_node, dist_to_goal);
            vector<Eigen::Vector2d> curr_best_path;
            fillPath(goal_node_, curr_best_path);
            path_list_.emplace_back(curr_best_path);
            solution_cost_time_pair_list_.emplace_back(goal_node_->cost_from_start, (ros::Time::now() - rrt_start_time).toSec());
          }
        }
        /* 3. rewire */
        rewire(x_new, neighbour_nodes, is_valid_edge, new_node, (ros::Time::now() - rrt_start_time).toSec());
      }
      /* end of sample once */

      vector<Eigen::Vector2d> vertice;
      vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> edges;
      sampleWholeTree(start_node_, vertice, edges);
      // std::vector<visualization::BALL> balls;
      // balls.reserve(vertice.size());
      // visualization::BALL node_p;
      // node_p.radius = 0.06;
      // for (size_t i = 0; i < vertice.size(); ++i)
      // {
      //   node_p.center = vertice[i];
      //   balls.push_back(node_p);
      // }
      // vis_ptr_->visualize_balls(balls, "tree_vertice", visualization::Color::blue, 1.0);
      // vis_ptr_->visualize_pairline(edges, "tree_edges", visualization::Color::red, 0.04);
      return (ros::Time::now() - rrt_start_time).toSec();
    }

    void sampleWholeTree(const RRTNode2DPtr &root, vector<Eigen::Vector2d> &vertice, vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &edges)
    {
      if (root == nullptr)
        return;

      // whatever dfs or bfs
      RRTNode2DPtr node = root;
      std::queue<RRTNode2DPtr> Q;
      Q.push(node);
      while (!Q.empty())
      {
        node = Q.front();
        Q.pop();
        for (const auto &leafptr : node->children)
        {
          vertice.push_back(leafptr->x);
          edges.emplace_back(std::make_pair(node->x, leafptr->x));
          Q.push(leafptr);
        }
      }
    }
  
    void find_parent(Eigen::Vector2d &x_new, RRTNode2DPtr &nearest_node, std::vector<RRTNode2DPtr> &neighbour_nodes, std::vector<bool> &is_valid_edge, RRTNode2DPtr &new_node, bool is_goal)
    {
      // find parent
      struct kdres *nbr_set;
      nbr_set = kd_nearest_range2(kd_tree, x_new[0], x_new[1], search_radius_);
      if (nbr_set == nullptr)
      {
        ROS_ERROR("bkwd kd range query error");
        return;
      }
      while (!kd_res_end(nbr_set))
      {
        RRTNode2DPtr curr_node = (RRTNode2DPtr)kd_res_item_data(nbr_set);
        neighbour_nodes.emplace_back(curr_node);
        // store range query result so that we dont need to query again for rewire;
        kd_res_next(nbr_set); //go to next in kd tree range query result
      }
      kd_res_free(nbr_set); //reset kd tree range query

      // choose parent from kd tree range query result
      double dist2nearest = calDist(nearest_node->x, x_new);
      double min_dist_from_start(nearest_node->cost_from_start + dist2nearest);
      double cost_from_p(dist2nearest);
      RRTNode2DPtr min_node(nearest_node); //set the nearest_node as the default parent

      // TODO: Choose a parent according to potential cost-from-start values
      // ! Hints:
      // !  1. Use map_ptr_->isSegmentValid(p1, p2) to check line edge validity;
      // !  2. Default parent is [nearest_node];
      // !  3. Store your chosen parent-node-pointer, the according cost-from-parent and cost-from-start
      // !     in [min_node], [cost_from_p], and [min_dist_from_start], respectively;
      // !  4. [Optional] You can sort the potential parents first in increasing order by cost-from-start value;
      // !  5. [Optional] You can store the collison-checking results for later usage in the Rewire procedure.
      // ! Implement your own code inside the following loop

      // store the collision-checking results for later usage in the Rewire procedure.
      for (auto &curr_node : neighbour_nodes)
      {
        // check if the edge is valid
        if (!map_ptr_->isSegmentValid(curr_node->x, x_new)) continue;
        // if valid, update the cost_from_start and check if it is the better parent
        is_valid_edge.push_back(true);
        double dist2curr = calDist(curr_node->x, x_new);
        double cost_from_curr = curr_node->cost_from_start + dist2curr;
        if (cost_from_curr < min_dist_from_start)
        {
          // update the parent
          min_dist_from_start = cost_from_curr;
          cost_from_p = dist2curr;
          min_node = curr_node;
        }
      }

      // ! Implement your own code inside the above loop

      /* parent found within radius, then add a node to rrt and kd_tree */
      /* 1.1 add the randomly sampled node to rrt_tree */
      if (is_goal)
      {
        // if the new_node is the goal, update the goal node's parent
        changeNodeParent(goal_node_, min_node, cost_from_p);
        goal_found = true;
      }
      else
      {
        if (valid_tree_node_nums_ < max_tree_node_nums_)
        {
          new_node = addTreeNode(min_node, x_new, min_dist_from_start, cost_from_p);
        }
        else
        {
          // delete a node that is not in the best path and has the largest cost_from_start
          RRTNode2DPtr node_to_delete = cost2node_map_.rbegin()->second;
          // check if the node is in the best path
          bool is_in_best_path = false;
          for (auto &path : path_list_)
          {
            for (auto &node : path)
            {
              if (node == node_to_delete->x)
              {
                is_in_best_path = true;
                break;
              }
            }
            if (is_in_best_path) break;
          }
          // if not in the best path, erase it
          node_to_delete->parent->children.remove(node_to_delete);
          cost2node_map_.erase(cost2node_map_.rbegin()->first);
          delete_leaf_node(kd_tree, node_to_delete->x.data(), 0.0001, max_tree_node_nums_);
          // remove the point from the kd tree
          delete node_to_delete;
          valid_tree_node_nums_--;
          // add the new node
          new_node = addTreeNode(min_node, x_new, min_dist_from_start, cost_from_p);
        }
        /* 1.2 add the randomly sampled node to kd_tree */
        kd_insert2(kd_tree, x_new[0], x_new[1], new_node);
      }
      // end of find parent
    }

    void rewire(const Eigen::Vector2d &x_new, vector<RRTNode2DPtr> &neighbour_nodes, const vector<bool> &is_valid_edge, RRTNode2DPtr &new_node, float time_cost)
    {
      /* 3.rewire */
      // TODO: Rewire according to potential cost-from-start values
      // ! Hints:
      // !  1. Use map_ptr_->isSegmentValid(p1, p2) to check line edge validity;
      // !  2. Use changeNodeParent(node, parent, cost_from_parent) to change a node's parent;
      // !  3. the variable [new_node] is the pointer of X_new;
      // !  4. [Optional] You can test whether the node is promising before checking edge collison.
      // ! Implement your own code between the dash lines [--------------] in the following loop
      auto curr_node = neighbour_nodes.begin();
      auto is_valid_edge_ptr = is_valid_edge.begin();
      while(curr_node != neighbour_nodes.end() && is_valid_edge != is_valid_edge)
      {
        // check if the edge is valid
        if (!(*is_valid_edge_ptr))  continue;
        // if valid, update the cost_from_start and check if it is the better parent
        double dist2curr = calDist((*curr_node)->x, x_new);
        double cost_from_curr = new_node->cost_from_start + dist2curr;
        if (cost_from_curr < (*curr_node)->cost_from_start)
        {
          // record the best cost before rewire
          double best_cost_before_rewire = goal_node_->cost_from_start;
          // update the parent
          changeNodeParent(*curr_node, new_node, dist2curr);
          // check if a better path is generated after rewiring
          if (best_cost_before_rewire > goal_node_->cost_from_start)
          {
            // update the best path
            vector<Eigen::Vector2d> curr_best_path;
            fillPath(goal_node_, curr_best_path);
            path_list_.emplace_back(curr_best_path);
            solution_cost_time_pair_list_.emplace_back(goal_node_->cost_from_start, time_cost);
          }
        }
        ++curr_node;
        ++is_valid_edge_ptr;
      }
      /* end of rewire */
    }
  };

} // namespace path_plan
#endif