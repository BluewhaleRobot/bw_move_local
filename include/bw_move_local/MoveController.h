/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Bluewhale Robot
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Author: Xie fusheng, Randoms
 *******************************************************************************/

#ifndef __MoveController_H__
#define __MoveController_H__
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <time.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>

#include <actionlib/server/simple_action_server.h>
#include <galileo_msg/LocalMoveAction.h>

#include <algorithm>
#include <math.h>
#include <stdlib.h>
#include <Eigen/Core>

#include <visualization_msgs/Marker.h>

#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <opencv2/core/core.hpp>

namespace bw_move_local
{

typedef enum class DO_STATUS
{
    prepare1,
    rotate1,
    rotate2,
    linear1,
    complete,
    temp1
} DO_STATUS;

class MoveController
{
  public:
    MoveController(std::string name);
    ~MoveController(void){}
    void executeCB(const galileo_msg::LocalMoveGoalConstPtr &goal);
    void resetState();
    bool doMove();
    void updateOdom(const nav_msgs::Odometry::ConstPtr& msg);
    void updateScan(const sensor_msgs::LaserScan& scan_in);
    void getThetaError(float & e_theta);
    bool getForwardError(float & e_theta,float & e_x);
    bool findNewGoalPose();
    void dealBar();
    bool isgoal_reached()
    {
      return x_goal_reached_ && theta_goal_reached_;
    }
    void filterscan(const sensor_msgs::LaserScan& scan_in);
    void dealscan();

    //二维码相关
    void updateMarkerPose(const ar_track_alvar_msgs::AlvarMarkers& currentMarkers);

    cv::Mat getTco() //获取当前二维码在摄像头坐标系下的姿态
    {
      boost::mutex::scoped_lock lock1(mMutex_armark);
      boost::mutex::scoped_lock lock2(mMutex_pose);
      mTco_ =  mTcb_*mTdb_.inv()*mTdo_;
      return mTco_.clone();
    }

    cv::Mat getTbo() //获取当前二维码在base_footprint坐标系下的姿态,这个base_footprint在摄像头水平面
    {
      boost::mutex::scoped_lock lock1(mMutex_armark);
      boost::mutex::scoped_lock lock2(mMutex_pose);
      mTbo_ =  mTdb_.inv()*mTdo_;
      return mTbo_.clone();
    }

    cv::Mat getTdb() //获取当前base_footprint在全局d坐标系下的姿态
    {
      boost::mutex::scoped_lock lock(mMutex_pose);
      return mTdb_.clone();
    }

    cv::Mat getTdo() //获取前二维码在全局d坐标系下的姿态
    {
      boost::mutex::scoped_lock lock(mMutex_armark);
      return mTdo_.clone();
    }

    void resetTdo_ready()
    {
      boost::mutex::scoped_lock lock(mMutex_armark);
      mTdo_ready_ = false;
      use_artag_ref_ready_ = false;
      resetTdo_flag_ = true;
    }

    bool isTdb_ready()
    {
      boost::mutex::scoped_lock lock(mMutex_pose);
      return mPose_flag_;
    }

    bool isTdo_ready()
    {
      boost::mutex::scoped_lock lock(mMutex_armark);
      return mTdo_ready_;
    }

    void getArtagError(float & e_theta, float & ar_dist,bool &  online_flag);

  private:
    //d->b->c->o
    boost::mutex mMutex_armark;
    ros::WallTime last_odomtime_;
    ros::WallTime last_armarktime_;
    bool mTdo_ready_;
    bool use_artag_ref_ready_;
    cv::Mat mTco_;
    cv::Mat mTbo_;
    cv::Mat mTdb_;
    cv::Mat mTdo_;
    cv::Mat mTbc_;
    cv::Mat mTcb_;
    cv::Mat mToo_; //偏移点
    bool force_no_bar_;
    bool use_artag_ref_;
    int camera_id_;
    double artag_line_width_;
    double artag_min_dist_;
    double artag_x_offset_;
    double artag_y_offset_;
    double kp_line_;
    double kd_line_;
    double ki_line_;
    double kp_angle_;
    double kd_angle_;
    double ki_angle_;

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<galileo_msg::LocalMoveAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    galileo_msg::LocalMoveFeedback feedback_;
    galileo_msg::LocalMoveResult result_;
    galileo_msg::LocalMoveGoal current_goal_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    ros::Subscriber sub3_;

    geometry_msgs::Pose mRobot_pose_;
    geometry_msgs::Pose mRobot_pose_last_;
    geometry_msgs::Pose mRobot_pose_boxedge_;

    boost::mutex mMutex_move;
    boost::mutex mMutex_pose;
    boost::mutex mMutex_scan;

    DO_STATUS mdo_status_;

    ros::WallTime moving_time_;

    double kp_theta_set_;
    double kd_theta_set_;
    double ki_theta_set_;
    double kp_x_set_;
    double kd_x_set_;
    double ki_x_set_;
    double max_theta_speed_;
    double max_x_speed_;
    double theta_torelance_;
    double x_torelance_;

    double error_theta_last_;
    double error_theta_sum_;
    double error_x_last_;
    double error_x_sum_;

    bool use_scan_;
    double bar_y_min_;
    double bar_y_max_;

    double bar_x_min_;
    double bar_x_max_;

    double filter_x_min_;
    double filter_x_max_;

    double inbox_distance_set_;

    std::vector<double> R_laserscan_;
    std::vector<double> T_laserscan_;

    bool mPose_flag_;
    bool mTf_flag_;

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_;

    geometry_msgs::PoseStamped robot_pose_;
    geometry_msgs::PoseStamped global_pose_;
    std::string global_frame_;

    bool inbox_flag_;
    double inbox_distance_;

    ros::Publisher mCmdvelPub_;

    float goal_pose[3];//x y theta
    bool x_goal_reached_;
    bool theta_goal_reached_;

    bool move_forward_enable_;
    bool rot_counter_enable_;
    bool rot_uncounter_enable_;
    bool use_forward_ref_;
    bool force_no_ref_;

    std::vector< std::vector<float> > scandata1_;// -pi -1.6
    std::vector< std::vector<float> > scandata2_; // 1.6 pi
    int scandata1_num_;
    int scandata2_num_;

    float angle_min_;
    float angle_max_;
    Eigen::ArrayXXd co_sine_map_;

    ros::Publisher marker_pub_;

    bool center23_ready_;
    bool center14_ready_;
    bool center_ready_;

    float center23_[3];
    float center14_[3];
    float center_[2];
    bool mRobot_pose_boxedge_ready_;

    bool resetTdo_flag_;
    float Tdo_roll_;
    float Tdo_pitch_;
    float Tdo_yaw_;
    float Tdo_x_;
    float Tdo_y_;
    float Tdo_z_;
    
};

}  // namespace bw_auto_dock
#endif  // DockController_H
