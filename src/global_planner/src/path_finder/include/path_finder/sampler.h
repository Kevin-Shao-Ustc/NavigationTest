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
#ifndef _BIAS_SAMPLER_
#define _BIAS_SAMPLER_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <random>

class BiasSampler
{
public:
  BiasSampler()
  {
    std::random_device rd;
    gen_ = std::mt19937_64(rd());
    uniform_rand_ = std::uniform_real_distribution<double>(0.0, 1.0);
    normal_rand_ = std::normal_distribution<double>(0.0, 1.0);
    range_.setZero();
    origin_.setZero();
  };

  void setSamplingRange(const Eigen::Vector2d origin, const Eigen::Vector2d range)
  {
    origin_ = origin;
    range_ = range;
  }

  // void samplingOnce(Eigen::Vector2d &sample, bool goal_found, double cost_from_start, Eigen::Vector2d &p_start, Eigen::Vector2d &p_goal)
  // {
  //   // rotation matrix
  //   static int has_calculated_rotation_matrix = 0;
  //   static Eigen::Vector2d hat_x;
  //   static Eigen::Vector2d hat_y;
  //   static Eigen::Vector2d hat_z;
  //   if (!has_calculated_rotation_matrix)
  //   {
  //     hat_z = (p_goal - p_start) / (p_goal - p_start).norm();
  //     Eigen::Vector2d unit_z(0, 0, 1);
  //     Eigen::Vector2d unit_y(0, 1, 0);
  //     if(abs(hat_z.dot(unit_z)) < 0.5f)
  //     {
  //       hat_x = hat_z.cross(unit_z);
  //       hat_x = hat_x / hat_x.norm();
  //       hat_y = hat_z.cross(hat_x);
  //     }
  //     else
  //     {
  //       hat_x = unit_y.cross(hat_z);
  //       hat_x = hat_x / hat_x.norm();
  //       hat_y = hat_z.cross(hat_x);
  //     }
  //   }
  //   // if a path has been found
  //   if (goal_found)
  //   {
  //     // calculate the params of the ellipsoid
  //     double a = cost_from_start / 2.0f;
  //     double c = (p_goal - p_start).norm() / 2.0f;
  //     double b = pow(a*a - c*c, 0.5f);

  //     // sample count
  //     int sample_count = 0;
  //     while (sample_count < 8)
  //     {
  //       // sample
  //       double r_ttpo_3_over_3 = uniform_rand_(gen_) / 3.0f;
  //       double sin_theta = (uniform_rand_(gen_) - 0.5f) * 2.0f;
  //       double phi = uniform_rand_(gen_) * 3.1415926f;
  //       // convert the variables to spherical coordinates
  //       double r = pow(r_ttpo_3_over_3 * 3.0f, 1 / 3.0f);
  //       double cos_theta = pow(1 - sin_theta * sin_theta, 0.5f);
  //       // calculate the coordinate of the sample point in the relative coord system
  //       double x_ = r * cos_theta * cos(phi) * b;
  //       double y_ = r * cos_theta * sin(phi) * b;
  //       double z_ = r * sin_theta * a;
  //       // calculate the sample point's coordinate in the ground coord system
  //       sample = (p_start + p_goal) / 2.0f + x_ * hat_x + y_ * hat_y + z_ * hat_z;
  //       // check if the sample point is valid
  //       bool x_valid = (sample[0] - origin_[0] > 0.0f) && (sample[0] - origin_[0] < range_[0]);
  //       bool y_valid = (sample[1] - origin_[1] > 0.0f) && (sample[1] - origin_[1] < range_[1]);
  //       bool z_valid = (sample[2] - origin_[2] > 0.0f) && (sample[2] - origin_[2] < range_[2]);
  //       // if valid, return
  //       if (x_valid && y_valid && z_valid)  return;
  //       // if not, continue the loop
  //       sample_count++;
  //     }
  //   }
  //   // if no path has been found
  //   sample[0] = uniform_rand_(gen_);
  //   sample[1] = uniform_rand_(gen_);
  //   sample[2] = uniform_rand_(gen_);
  //   sample.array() *= range_.array();
  //   sample += origin_;
  // };
  void samplingOnce(Eigen::Vector2d &sample, bool goal_found, double cost_from_start, Eigen::Vector2d &p_start, Eigen::Vector2d &p_goal)
  {
    // rotation matrix
    static int has_calculated_rotation_matrix = 0;
    static Eigen::Vector2d hat_x;
    static Eigen::Vector2d hat_y;
    if (!has_calculated_rotation_matrix)
    {
      hat_x = (p_goal - p_start) / (p_goal - p_start).norm();
      hat_y = Eigen::Vector2d(-hat_x[1], hat_x[0]);
      has_calculated_rotation_matrix = 1;
    }
    // if a path has been found
    if (goal_found)
    {
      // calculate the params of the ellipsoid
      double a = cost_from_start / 2.0f;
      double c = (p_goal - p_start).norm() / 2.0f;
      double b = pow(a*a - c*c, 0.5f);

      // sample count
      int sample_count = 0;
      while (sample_count < 8)
      {
        // sample
        double r_ttpo_2 = uniform_rand_(gen_);
        double theta = uniform_rand_(gen_) * 3.1415926f;
        // convert the variables to spherical coordinates
        double r = pow(r_ttpo_2, 1 / 2.0f);
        // calculate the coordinate of the sample point in the relative coord system
        double x_ = r * cos(theta) * a;
        double y_ = r * sin(theta) * b;
        // calculate the sample point's coordinate in the ground coord system
        sample = (p_start + p_goal) / 2.0f + x_ * hat_x + y_ * hat_y;
        // check if the sample point is valid
        bool x_valid = (sample[0] - origin_[0] > 0.0f) && (sample[0] - origin_[0] < range_[0]);
        bool y_valid = (sample[1] - origin_[1] > 0.0f) && (sample[1] - origin_[1] < range_[1]);
        // if valid, return
        if (x_valid && y_valid)  return;
        // if not, continue the loop
        sample_count++;
      }
    }
    // if no path has been found
    sample[0] = uniform_rand_(gen_);
    sample[1] = uniform_rand_(gen_);
    sample.array() *= range_.array();
    sample += origin_;
  };


  // (0.0 - 1.0)
  double getUniRandNum()
  {
    return uniform_rand_(gen_);
  }

private:
  Eigen::Vector2d range_, origin_;
  std::mt19937_64 gen_;
  std::uniform_real_distribution<double> uniform_rand_;
  std::normal_distribution<double> normal_rand_;
};

#endif
