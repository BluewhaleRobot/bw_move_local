#include "bw_move_local/MoveController.h"

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define m_pi 3.1415926
#define MARKER_ID_DETECTION 1

namespace bw_move_local
{
MoveController::MoveController(std::string name):as_(nh_, name, boost::bind(&MoveController::executeCB, this, _1), false),action_name_(name),tf2_(tf2_buffer_)
{
    ros::NodeHandle private_nh("~/MoveLocal");

    global_frame_ = std::string("odom");

    std::string global_frame_name;
    if(private_nh.searchParam("global_frame", global_frame_name))
    {
      private_nh.param(global_frame_name, global_frame_,std::string("odom"));
    }

    kp_theta_set_ = 0.4;
    kd_theta_set_ = 0.8;
    ki_theta_set_ = 1.0;
    kp_x_set_ = 0.4;
    kd_x_set_ = 0.8;
    ki_x_set_ = 10.0;

    bool force_no_ref_ = false;
    std::string force_no_ref_name;
    if(private_nh.searchParam("force_no_ref", force_no_ref_name))
    {
      private_nh.param(force_no_ref_name, force_no_ref_,false);
    }


    std::string kp_theta_set_name;
    if(private_nh.searchParam("kp_theta_set", kp_theta_set_name))
    {
      private_nh.param(kp_theta_set_name, kp_theta_set_,0.4);
    }

    std::string kd_theta_set_name;
    if(private_nh.searchParam("kd_theta_set", kd_theta_set_name))
    {
      private_nh.param(kd_theta_set_name, kd_theta_set_,0.8);
    }

    std::string ki_theta_set_name;
    if(private_nh.searchParam("ki_theta_set", ki_theta_set_name))
    {
      private_nh.param(ki_theta_set_name, ki_theta_set_,1.0);
    }

    std::string kp_x_set_name;
    if(private_nh.searchParam("kp_x_set", kp_x_set_name))
    {
      private_nh.param(kp_x_set_name, kp_x_set_,0.4);
    }

    std::string kd_x_set_name;
    if(private_nh.searchParam("kd_x_set", kd_x_set_name))
    {
      private_nh.param(kd_x_set_name, kd_x_set_,0.8);
    }

    std::string ki_x_set_name;
    if(private_nh.searchParam("ki_x_set", ki_x_set_name))
    {
      private_nh.param(ki_x_set_name, ki_x_set_,10.0);
    }

    max_theta_speed_ = 0.3;
    max_x_speed_ = 0.2;
    theta_torelance_ = 0.02;
    x_torelance_ = 0.02;

    std::string max_theta_speed_name;
    if(private_nh.searchParam("max_theta_speed", max_theta_speed_name))
    {
      private_nh.param(max_theta_speed_name, max_theta_speed_,0.3);
    }

    std::string max_x_speed_name;
    if(private_nh.searchParam("max_x_speed", max_x_speed_name))
    {
      private_nh.param(max_x_speed_name, max_x_speed_,0.3);
    }

    std::string theta_torelance_name;
    if(private_nh.searchParam("theta_torelance", theta_torelance_name))
    {
      private_nh.param(theta_torelance_name, theta_torelance_,0.02);
    }

    std::string x_torelance_name;
    if(private_nh.searchParam("x_torelance", x_torelance_name))
    {
      private_nh.param(x_torelance_name, x_torelance_,0.02);
    }

    //激光雷达相关
    use_scan_ = true;
    bar_y_min_ = -0.25;
    bar_y_max_ = 0.25;

    bar_x_min_ = -0.1;
    bar_x_max_ = 0.25;

    filter_x_min_ = -0.1;
    filter_x_max_ = 3.0;

    inbox_distance_set_ = 0.5;

    use_forward_ref_ = false;
    std::string use_forward_ref_name;
    if(private_nh.searchParam("use_forward_ref", use_forward_ref_name))
    {
      private_nh.param(use_forward_ref_name, use_forward_ref_,false);
    }

    std::string use_scan_name;
    if(private_nh.searchParam("use_scan", use_scan_name))
    {
      private_nh.param(use_scan_name, use_scan_,true);
    }

    std::string bar_y_min_name;
    if(private_nh.searchParam("bar_y_min", bar_y_min_name))
    {
      private_nh.param(bar_y_min_name, bar_y_min_,-0.25);
    }

    std::string bar_y_max_name;
    if(private_nh.searchParam("bar_y_max", bar_y_max_name))
    {
      private_nh.param(bar_y_max_name, bar_y_max_,0.25);
    }

    std::string bar_x_min_name;
    if(private_nh.searchParam("bar_x_min", bar_x_min_name))
    {
      private_nh.param(bar_x_min_name, bar_x_min_,-0.1);
    }

    std::string bar_x_max_name;
    if(private_nh.searchParam("bar_x_max", bar_x_max_name))
    {
      private_nh.param(bar_x_max_name, bar_x_max_,0.25);
    }

    std::string filter_x_min_name;
    if(private_nh.searchParam("filter_x_min",filter_x_min_name))
    {
      private_nh.param(filter_x_min_name, filter_x_min_,-0.1);
    }

    std::string filter_x_max_name;
    if(private_nh.searchParam("filter_x_max", filter_x_max_name))
    {
      private_nh.param(filter_x_max_name, filter_x_max_,3.0);
    }

    std::string inbox_distance_set_name;
    if(private_nh.searchParam("inbox_distance_set", inbox_distance_set_name))
    {
      private_nh.param(inbox_distance_set_name, inbox_distance_set_,0.5);
    }

    std::string R_laserscan_name;
    if(private_nh.searchParam("R_laserscan", R_laserscan_name))
    {
      private_nh.getParam(R_laserscan_name, R_laserscan_);
    }
    else
    {
      R_laserscan_.push_back(-1.0);
      R_laserscan_.push_back(0.0);

      R_laserscan_.push_back(0.0);
      R_laserscan_.push_back(-1.0);

    }

    std::string T_laserscan_name;
    if(private_nh.searchParam("T_laserscan", T_laserscan_name))
    {
      private_nh.getParam(T_laserscan_name, T_laserscan_);
    }
    else
    {
      T_laserscan_.push_back(0.0);
      T_laserscan_.push_back(0.0);
    }

    error_theta_last_ = 0;
    error_theta_sum_ = 0;
    error_x_last_ = 0;
    error_x_sum_ = 0;

    mPose_flag_ = false;
    mTf_flag_ = false;
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose_.pose);
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose_.pose);

    tf2::toMsg(tf2::Transform::getIdentity(), mRobot_pose_);
    tf2::toMsg(tf2::Transform::getIdentity(), mRobot_pose_last_);

    error_theta_last_ = 0;
    error_theta_sum_ = 0;
    error_x_last_ = 0;
    error_x_sum_= 0;

    if(use_scan_)
    {
        sub1_ = nh_.subscribe("/scan", 10, &MoveController::updateScan, this);
    }
    sub2_ = nh_.subscribe("/odom", 10, &MoveController::updateOdom, this);


    mCmdvelPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/local_move/visualization_marker", 10);

    scandata1_.resize(2000,std::vector<float> (3));
    scandata2_.resize(2000,std::vector<float> (3));
    scandata1_num_ = 0;
    scandata2_num_ = 0;

    center_ready_ = false;
    center14_ready_ = false;
    center23_ready_ = false;
    mRobot_pose_boxedge_ready_ = false;

    last_odomtime_ = ros::WallTime::now();
    last_armarktime_ = ros::WallTime::now();
    mTdo_ready_ = false;
    mTco_ = cv::Mat::eye(4, 4, CV_32F);
    mTbo_ = cv::Mat::eye(4, 4, CV_32F);
    mTdb_ = cv::Mat::eye(4, 4, CV_32F);
    mTdo_ = cv::Mat::eye(4, 4, CV_32F);
    mTbc_ = cv::Mat::eye(4, 4, CV_32F);
    mTcb_ = cv::Mat::eye(4, 4, CV_32F);
    mToo_ = cv::Mat::eye(4, 4, CV_32F);

    force_no_bar_ = false;
    std::string force_no_bar_name;
    if(private_nh.searchParam("force_no_bar", force_no_bar_name))
    {
      private_nh.param(force_no_bar_name, force_no_bar_,false);
    }

    use_artag_ref_ = false;
    std::string use_artag_ref_name;
    if(private_nh.searchParam("use_artag_ref", use_artag_ref_name))
    {
      private_nh.param(use_artag_ref_name, use_artag_ref_,false);
    }

    camera_id_ = 1;
    std::string camera_id_name;
    if(private_nh.searchParam("camera_id", camera_id_name))
    {
      private_nh.param(camera_id_name, camera_id_,1);
    }

    artag_line_width_ = 0.1;
    std::string artag_line_width_name;
    if(private_nh.searchParam("artag_line_width", artag_line_width_name))
    {
      private_nh.param(artag_line_width_name, artag_line_width_,0.1);
    }

    artag_min_dist_ = 0.1;
    std::string artag_min_dist_name;
    if(private_nh.searchParam("artag_min_dist", artag_min_dist_name))
    {
      private_nh.param(artag_min_dist_name, artag_min_dist_,0.1);
    }

    artag_x_offset_ = 0;
    std::string artag_x_offset_name;
    if(private_nh.searchParam("artag_x_offset", artag_x_offset_name))
    {
      private_nh.param(artag_x_offset_name, artag_x_offset_,0.0);
    }

    artag_y_offset_ = 0;
    std::string artag_y_offset_name;
    if(private_nh.searchParam("artag_y_offset", artag_y_offset_name))
    {
      private_nh.param(artag_y_offset_name, artag_y_offset_,0.0);
    }

    kp_line_ = 0.4;
    std::string kp_line_name;
    if(private_nh.searchParam("kp_line", kp_line_name))
    {
      private_nh.param(kp_line_name, kp_line_,0.4);
    }

    kd_line_ = 0.8;
    std::string kd_line_name;
    if(private_nh.searchParam("kd_line", kd_line_name))
    {
      private_nh.param(kd_line_name, kd_line_,0.8);
    }

    ki_line_ = 1.0;
    std::string ki_line_name;
    if(private_nh.searchParam("ki_line", ki_line_name))
    {
      private_nh.param(ki_line_name, ki_line_,1.0);
    }

    kp_angle_ = 0.4;
    std::string kp_angle_name;
    if(private_nh.searchParam("kp_angle", kp_angle_name))
    {
      private_nh.param(kp_angle_name, kp_angle_,0.4);
    }

    kd_angle_ = 0.8;
    std::string kd_angle_name;
    if(private_nh.searchParam("kd_angle", kd_angle_name))
    {
      private_nh.param(kd_angle_name, kd_angle_,0.8);
    }

    ki_angle_ = 1.0;
    std::string ki_angle_name;
    if(private_nh.searchParam("ki_angle", ki_angle_name))
    {
      private_nh.param(ki_angle_name, ki_angle_,1.0);
    }

    if(use_artag_ref_)
    {

      mToo_.at<float>(0,3) = artag_x_offset_;
      mToo_.at<float>(1,3) = artag_y_offset_;

      sub3_ = nh_.subscribe("/ar_pose_marker", 1, &MoveController::updateMarkerPose, this);
      std::string camera_filepath = "/home/xiaoqiang/Documents/ros/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Data/setting5_multi.yaml";
      cv::FileStorage fSettings(camera_filepath, cv::FileStorage::READ);
      if(camera_id_ == 0)
      {
        fSettings["TbaMatrix"] >> mTbc_;
      }
      else
      {
        fSettings["Tbc2Matrix"] >> mTbc_;
        std::cout << mTbc_ <<std::endl;
        std::stringstream buf_Tbc;
        buf_Tbc <<"Tbc "<< mTbc_ <<std::endl;
        ROS_DEBUG("%s",buf_Tbc.str().c_str());
      }

      mTcb_ = mTbc_.inv();
    }

    resetTdo_flag_ = true;
    Tdo_roll_ = 0;
    Tdo_pitch_ = 0;
    Tdo_yaw_ = 0;
    Tdo_x_ = 0;
    Tdo_y_ = 0;
    Tdo_z_ = 0;
    as_.start();
}

void MoveController::executeCB(const galileo_msg::LocalMoveGoalConstPtr &goal)
{
    ROS_DEBUG("oups1");
    ros::Rate r(30);
    bool success = true;

    feedback_.distance = 0.0;
    feedback_.angle = 0.0;
    result_.result = true;

    current_goal_.distance = goal->distance;
    current_goal_.angle = goal->angle;
    current_goal_.method = goal->method;

    {
        boost::mutex::scoped_lock lock(mMutex_pose);
        if(!mPose_flag_)
        {
            ROS_DEBUG("oups2");
            as_.setAborted(galileo_msg::LocalMoveResult(), "Aborting on the goal because pose not ready");
            success = false;
            ros::param::set("/xqserial_server/params/detect_bar", true);
            return;
        }
    }

    this->resetState();
    ROS_DEBUG("oups3");
    // start executing the action
    ros::NodeHandle n;
    while(n.ok())
    {
        if(as_.isPreemptRequested())
        {
            if(as_.isNewGoalAvailable())
            {
                //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
                current_goal_ = *as_.acceptNewGoal();
                this->resetState();
            }
            else
            {
                as_.setPreempted();
                success = false;
                break;
            }
        }
        ROS_DEBUG("oups4");
        if(!doMove())
        {
          ROS_DEBUG("oups4.1");
            //移动失败，需要设为abort
            //如果进洞距离大于inbox_distance_set_，则认为已经到达目标
            if(inbox_distance_>=inbox_distance_set_)
            {
                ROS_DEBUG("oups5");
                break;
            }
            else
            {
                ROS_DEBUG("oups6");
                as_.setAborted(galileo_msg::LocalMoveResult(), "Aborting on the goal because some inner error");
                success = false;
            }
            break;
        }
        ROS_DEBUG("oups4.2");
        if(isgoal_reached())
        {
            ROS_DEBUG("oups7");
            break;
        }
        as_.publishFeedback(feedback_);
        r.sleep();
    }
    ROS_DEBUG("oups8");
    if(success)
    {
        ROS_DEBUG("oups9");
        as_.setSucceeded(result_);
    }
    ros::param::set("/xqserial_server/params/detect_bar", true);
    geometry_msgs::Twist current_vel;
    current_vel.linear.x = 0;
    current_vel.linear.y = 0;
    current_vel.linear.z = 0;
    current_vel.angular.x = 0;
    current_vel.angular.y = 0;
    current_vel.angular.z = 0;
    mCmdvelPub_.publish(current_vel);
    return;
}

void MoveController::resetState()
{
    boost::mutex::scoped_lock lock(mMutex_pose);
    inbox_flag_ = false;
    inbox_distance_ = 0.0;

    x_goal_reached_ = false;
    theta_goal_reached_ = false;

    mRobot_pose_boxedge_ = mRobot_pose_;
    mRobot_pose_boxedge_ready_ = false;

    moving_time_ = ros::WallTime::now();

    if(!mPose_flag_) return;
    float x,y,theta;
    x = mRobot_pose_.position.x;
    y = mRobot_pose_.position.y;
    tf::Quaternion q1(mRobot_pose_.orientation.x, mRobot_pose_.orientation.y, mRobot_pose_.orientation.z,
                      mRobot_pose_.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    theta = yaw;

    mdo_status_ = DO_STATUS::prepare1;
    if(current_goal_.distance>0)
    {
        //保存当前位置
        mRobot_pose_last_ = mRobot_pose_;
    }
    else if( use_forward_ref_ && current_goal_.distance<0 && std::fabs(current_goal_.angle)<=theta_torelance_)
    {

        //后退距离，选择上一次前进出发位置与当前设置值的最小值为目标值。
        float dx = mRobot_pose_last_.position.x - x;
        float dy = mRobot_pose_last_.position.y - y;
        float distance_temp = std::fabs(dx*cos(theta) + dy*sin(theta));
        if(distance_temp <= std::fabs(current_goal_.distance))
        {
            current_goal_.distance = - distance_temp;
        }
    }

    goal_pose[2] = theta + current_goal_.angle;
    if(goal_pose[2]>3.1415926) goal_pose[2] = goal_pose[2] - 2*3.1415926;
    if(goal_pose[2]<-3.1415926) goal_pose[2] = goal_pose[2] + 2*3.1415926;

    goal_pose[0] = x + current_goal_.distance*cos(goal_pose[2]);
    goal_pose[1] = y + current_goal_.distance*sin(goal_pose[2]);
    ROS_DEBUG("reset %f %f %f, %f %f %f",x,y,theta,goal_pose[0],goal_pose[1],goal_pose[2]);
}

void MoveController::update_goal_theta(float angle_delta)
{
  boost::mutex::scoped_lock lock(mMutex_pose);

  float theta;
  tf::Quaternion q1(mRobot_pose_.orientation.x, mRobot_pose_.orientation.y, mRobot_pose_.orientation.z,
                    mRobot_pose_.orientation.w);
  tf::Matrix3x3 m1(q1);
  double roll, pitch, yaw;
  m1.getRPY(roll, pitch, yaw);
  theta = yaw;
  goal_pose[2] = theta + angle_delta;
  if(goal_pose[2]>3.1415926) goal_pose[2] = goal_pose[2] - 2*3.1415926;
  if(goal_pose[2]<-3.1415926) goal_pose[2] = goal_pose[2] + 2*3.1415926;
}

void MoveController::update_goal_all(float angle_delta,float distance)
{
  //角度设成当前姿态值
  boost::mutex::scoped_lock lock(mMutex_pose);

  float x,y,theta;
  x = mRobot_pose_.position.x;
  y = mRobot_pose_.position.y;
  tf::Quaternion q1(mRobot_pose_.orientation.x, mRobot_pose_.orientation.y, mRobot_pose_.orientation.z,
                    mRobot_pose_.orientation.w);
  tf::Matrix3x3 m1(q1);
  double roll, pitch, yaw;
  m1.getRPY(roll, pitch, yaw);
  theta = yaw;

  goal_pose[2] = theta + angle_delta;
  if(goal_pose[2]>3.1415926) goal_pose[2] = goal_pose[2] - 2*3.1415926;
  if(goal_pose[2]<-3.1415926) goal_pose[2] = goal_pose[2] + 2*3.1415926;

  goal_pose[0] = x + distance*cos(goal_pose[2]);
  goal_pose[1] = y + distance*sin(goal_pose[2]);
}

bool MoveController::doMove()
{
    boost::mutex::scoped_lock lock(mMutex_move);
    ROS_DEBUG("do move 1");
    geometry_msgs::Twist current_vel;
    switch (mdo_status_)
    {
        case DO_STATUS::prepare1:
        {
            error_theta_last_ = 0;
            error_theta_sum_ = 0;
            error_x_last_ = 0;
            error_x_sum_ = 0;
            ros::param::set("/xqserial_server/params/detect_bar", false);
            x_goal_reached_ = false;
            theta_goal_reached_ = false;
            if(std::fabs(current_goal_.angle)<=theta_torelance_)
            {
                ROS_DEBUG("prepare1 %f %f ",current_goal_.angle,theta_torelance_);
                theta_goal_reached_ = true;
                mdo_status_ = DO_STATUS::linear1;
                resetTdo_ready();
                ros::Duration(0.3).sleep(); //延时200ms，让直线运动更准同时识别二维码。
            }
            else
            {
                mdo_status_ = DO_STATUS::rotate1;
            }
            break;
        }
        case DO_STATUS::rotate1:
        {
            float diff_theta;
            getThetaError(diff_theta);
            if(std::fabs(diff_theta) <= theta_torelance_)
            {
                ROS_DEBUG("rotate1 %f %f ",diff_theta,theta_torelance_);
                theta_goal_reached_ = true;
                error_theta_last_ = 0;
                error_theta_sum_ = 0;
                error_x_last_ = 0;
                error_x_sum_ = 0;
                mdo_status_ = DO_STATUS::linear1;
                current_vel.linear.x = 0;
                current_vel.linear.y = 0;
                current_vel.linear.z = 0;
                current_vel.angular.x = 0;
                current_vel.angular.y = 0;
                current_vel.angular.z = 0;
                mCmdvelPub_.publish(current_vel);
                resetTdo_ready();
                ros::Duration(0.3).sleep(); //延时200ms，让直线运动更准同时识别二维码。
            }
            else
            {
                //pid旋转
                float kp_theta = kp_theta_set_;
                float kd_theta = kp_theta_set_*30*kd_theta_set_;
                float ki_theta = kp_theta_set_/30/ki_theta_set_;

                float error_temp1 = diff_theta - error_theta_last_;
                error_theta_sum_ += diff_theta;
                if(error_theta_sum_>3.0) error_theta_sum_ = 3.0;
                if(error_theta_sum_<-3.0) error_theta_sum_ = -3.0;

                current_vel.angular.z = kp_theta*diff_theta + kd_theta*error_temp1 + ki_theta*error_theta_sum_;
                if(current_vel.angular.z > max_theta_speed_ ) current_vel.angular.z = max_theta_speed_;
                if(current_vel.angular.z < -max_theta_speed_ ) current_vel.angular.z = -max_theta_speed_;

                current_vel.linear.x = 0;
                current_vel.linear.y = 0;
                current_vel.linear.z = 0;
                current_vel.angular.x = 0;
                current_vel.angular.y = 0;
                dealBar();
                if(current_vel.angular.z>0 && !rot_uncounter_enable_) current_vel.angular.z=0;
                if(current_vel.angular.z<0 && !rot_counter_enable_) current_vel.angular.z=0;
                mCmdvelPub_.publish(current_vel);
                ROS_DEBUG("rotate1.2 %f %f ",diff_theta,current_vel.angular.z);
                if((!rot_uncounter_enable_  && current_goal_.angle >0) || (!rot_counter_enable_  && current_goal_.angle <0))
                {
                  error_theta_last_ = 0;
                  error_theta_sum_ = 0;
                  error_x_last_ = 0;
                  error_x_sum_ = 0;
                  mdo_status_ = DO_STATUS::complete;//
                  return false;
                }
                if(std::fabs(current_vel.angular.z)>0.01)
                {
                  moving_time_ = ros::WallTime::now();
                }
                else
                {
                  ros::WallDuration free_diff = ros::WallTime::now() - moving_time_;
                  if(free_diff.toSec()>5.0)
                  {
                    error_theta_last_ = 0;
                    error_theta_sum_ = 0;
                    error_x_last_ = 0;
                    error_x_sum_ = 0;
                    mdo_status_ = DO_STATUS::complete;//
                    return false;
                  }
                }
            }
            error_theta_last_ = diff_theta;
            break;
        }
        case DO_STATUS::rotate2:
        {
            float diff_theta,ar_dist;
            bool online_flag ;
            float e_y = 0;
            getArtagError(diff_theta, e_y, ar_dist,online_flag);
            if(std::fabs(diff_theta) <= theta_torelance_ )
            {
                //角度误差已经满足要求
                ROS_ERROR("rotate2.1 %f %f %f",diff_theta,theta_torelance_, e_y);
                current_vel.linear.x = 0;
                current_vel.linear.y = 0;
                current_vel.linear.z = 0;
                current_vel.angular.x = 0;
                current_vel.angular.y = 0;
                current_vel.angular.z = 0;
                mCmdvelPub_.publish(current_vel);
                ros::Duration(0.3).sleep(); //延时200ms，让直线运动更准同时识别二维码。
                error_theta_last_ = 0;
                error_theta_sum_ = 0;
                error_x_last_ = 0;
                error_x_sum_ = 0;
                if( online_flag )
                {
                  //满足线宽要求，开始后退对准
                  update_goal_all(0,current_goal_.distance);
                  mdo_status_ = DO_STATUS::linear2;
                }
                else
                {
                  //先转90度
                  if(e_y > 0)
                  {
                    update_goal_all(-m_pi/2.0,e_y);
                  }
                  else
                  {
                    update_goal_all(m_pi/2.0,-e_y);
                  }
                  last_e_y_ = e_y;
                  mdo_status_ = DO_STATUS::rotate3;

                }
            }
            else
            {
                //pid旋转
                float kp_theta = kp_angle_;
                float kd_theta = kp_angle_*30*kd_angle_;
                float ki_theta = kp_angle_/30/ki_angle_;

                float error_temp1 = diff_theta - error_theta_last_;
                error_theta_sum_ += diff_theta;
                if(error_theta_sum_>3.0) error_theta_sum_ = 3.0;
                if(error_theta_sum_<-3.0) error_theta_sum_ = -3.0;

                current_vel.angular.z = kp_theta*diff_theta + kd_theta*error_temp1 + ki_theta*error_theta_sum_;
                if(current_vel.angular.z > max_theta_speed_ ) current_vel.angular.z = max_theta_speed_;
                if(current_vel.angular.z < -max_theta_speed_ ) current_vel.angular.z = -max_theta_speed_;

                current_vel.linear.x = 0;
                current_vel.linear.y = 0;
                current_vel.linear.z = 0;
                current_vel.angular.x = 0;
                current_vel.angular.y = 0;

                mCmdvelPub_.publish(current_vel);
                ROS_DEBUG("rotate2.2 %f %f ",diff_theta,current_vel.angular.z);
            }
            error_theta_last_ = diff_theta;
            break;
        }
        case DO_STATUS::rotate3:
        {
            float diff_theta;
            getThetaError(diff_theta);
            if(std::fabs(diff_theta) <= theta_torelance_)
            {
                ROS_DEBUG("rotate3 %f %f ",diff_theta,theta_torelance_);
                error_theta_last_ = 0;
                error_theta_sum_ = 0;
                error_x_last_ = 0;
                error_x_sum_ = 0;
                mdo_status_ = DO_STATUS::linear3;
                current_vel.linear.x = 0;
                current_vel.linear.y = 0;
                current_vel.linear.z = 0;
                current_vel.angular.x = 0;
                current_vel.angular.y = 0;
                current_vel.angular.z = 0;
                mCmdvelPub_.publish(current_vel);
            }
            else
            {
                //pid旋转
                float kp_theta = kp_theta_set_;
                float kd_theta = kp_theta_set_*30*kd_theta_set_;
                float ki_theta = kp_theta_set_/30/ki_theta_set_;

                float error_temp1 = diff_theta - error_theta_last_;
                error_theta_sum_ += diff_theta;
                if(error_theta_sum_>3.0) error_theta_sum_ = 3.0;
                if(error_theta_sum_<-3.0) error_theta_sum_ = -3.0;

                current_vel.angular.z = kp_theta*diff_theta + kd_theta*error_temp1 + ki_theta*error_theta_sum_;
                if(current_vel.angular.z > max_theta_speed_ ) current_vel.angular.z = max_theta_speed_;
                if(current_vel.angular.z < -max_theta_speed_ ) current_vel.angular.z = -max_theta_speed_;

                current_vel.linear.x = 0;
                current_vel.linear.y = 0;
                current_vel.linear.z = 0;
                current_vel.angular.x = 0;
                current_vel.angular.y = 0;
                dealBar();
                if(current_vel.angular.z>0 && !rot_uncounter_enable_) current_vel.angular.z=0;
                if(current_vel.angular.z<0 && !rot_counter_enable_) current_vel.angular.z=0;
                mCmdvelPub_.publish(current_vel);
                ROS_DEBUG("rotate3.2 %f %f ",diff_theta,current_vel.angular.z);
                if((!rot_uncounter_enable_  && current_goal_.angle >0) || (!rot_counter_enable_  && current_goal_.angle <0))
                {
                  error_theta_last_ = 0;
                  error_theta_sum_ = 0;
                  error_x_last_ = 0;
                  error_x_sum_ = 0;
                  mdo_status_ = DO_STATUS::complete;//
                  return false;
                }
                if(std::fabs(current_vel.angular.z)>0.01)
                {
                  moving_time_ = ros::WallTime::now();
                }
                else
                {
                  ros::WallDuration free_diff = ros::WallTime::now() - moving_time_;
                  if(free_diff.toSec()>5.0)
                  {
                    error_theta_last_ = 0;
                    error_theta_sum_ = 0;
                    error_x_last_ = 0;
                    error_x_sum_ = 0;
                    mdo_status_ = DO_STATUS::complete;//
                    return false;
                  }
                }
            }
            error_theta_last_ = diff_theta;
            break;
        }
        case DO_STATUS::rotate4:
        {
            //再次回正
            float diff_theta,ar_dist;
            bool online_flag ;
            float e_y = 0;
            getArtagError(diff_theta, e_y, ar_dist,online_flag);
            if(std::fabs(diff_theta) <= theta_torelance_ )
            {
                //角度误差已经满足要求
                ROS_ERROR("rotate4.1 %f %f ",diff_theta,theta_torelance_);
                x_goal_reached_ = true;
                error_theta_last_ = 0;
                error_theta_sum_ = 0;
                error_x_last_ = 0;
                error_x_sum_ = 0;
                mdo_status_ = DO_STATUS::complete;//

                current_vel.linear.x = 0;
                current_vel.linear.y = 0;
                current_vel.linear.z = 0;
                current_vel.angular.x = 0;
                current_vel.angular.y = 0;
                current_vel.angular.z = 0;
                mCmdvelPub_.publish(current_vel);
            }
            else
            {
                //pid旋转
                float kp_theta = kp_angle_;
                float kd_theta = kp_angle_*30*kd_angle_;
                float ki_theta = kp_angle_/30/ki_angle_;

                float error_temp1 = diff_theta - error_theta_last_;
                error_theta_sum_ += diff_theta;
                if(error_theta_sum_>3.0) error_theta_sum_ = 3.0;
                if(error_theta_sum_<-3.0) error_theta_sum_ = -3.0;

                current_vel.angular.z = kp_theta*diff_theta + kd_theta*error_temp1 + ki_theta*error_theta_sum_;
                if(current_vel.angular.z > max_theta_speed_ ) current_vel.angular.z = max_theta_speed_;
                if(current_vel.angular.z < -max_theta_speed_ ) current_vel.angular.z = -max_theta_speed_;

                current_vel.linear.x = 0;
                current_vel.linear.y = 0;
                current_vel.linear.z = 0;
                current_vel.angular.x = 0;
                current_vel.angular.y = 0;

                mCmdvelPub_.publish(current_vel);
                ROS_DEBUG("rotate4.2 %f %f ",diff_theta,current_vel.angular.z);
            }
            error_theta_last_ = diff_theta;
            break;
        }
        case DO_STATUS::linear1:
        {
          float diff_theta,diff_distance;
          bool error_flag = getForwardError(diff_theta,diff_distance);
          if(diff_distance<0)
          {
            diff_theta = -diff_theta;
          }
          ROS_DEBUG("linear1.1 %f %f",diff_theta,diff_distance);
          if(use_artag_ref_ && (isTdo_ready() || use_artag_ref_ready_) && current_goal_.distance < 0)
          {
            use_artag_ref_ready_ = true;
            ROS_DEBUG("linear1.1.0 ");
            error_theta_last_ = 0;
            error_theta_sum_ = 0;
            error_x_last_ = 0;
            error_x_sum_ = 0;
            mdo_status_ = DO_STATUS::rotate2;//再原地旋转对准一下角度
            current_vel.linear.x = 0;
            current_vel.linear.y = 0;
            current_vel.linear.z = 0;
            current_vel.angular.x = 0;
            current_vel.angular.y = 0;
            current_vel.angular.z = 0;
            mCmdvelPub_.publish(current_vel);
            ros::Duration(0.2).sleep(); //延时200ms，让直线运动更准同时识别二维码。
          }
          else
          {
            if(std::fabs(diff_distance) <= x_torelance_ || (diff_distance*current_goal_.distance)<0.0001)
            {
                ROS_DEBUG("linear1.1.2 %f %f %f",diff_theta,diff_distance,x_torelance_);
                x_goal_reached_ = true;
                error_theta_last_ = 0;
                error_theta_sum_ = 0;
                error_x_last_ = 0;
                error_x_sum_ = 0;
                mdo_status_ = DO_STATUS::complete;//
                current_vel.linear.x = 0;
                current_vel.linear.y = 0;
                current_vel.linear.z = 0;
                current_vel.angular.x = 0;
                current_vel.angular.y = 0;
                current_vel.angular.z = 0;
                mCmdvelPub_.publish(current_vel);
            }
            else
            {
                //pid对准
                float kp_theta = kp_theta_set_;
                float kd_theta = kp_theta_set_*30*kd_theta_set_;
                float ki_theta = kp_theta_set_/30/ki_theta_set_;

                float error_temp1 = diff_theta - error_theta_last_;
                error_theta_sum_ += diff_theta;
                if(error_theta_sum_>3.0) error_theta_sum_ = 3.0;
                if(error_theta_sum_<-3.0) error_theta_sum_ = -3.0;

                current_vel.angular.z = kp_theta*diff_theta + kd_theta*error_temp1 + ki_theta*error_theta_sum_;
                if(current_vel.angular.z > max_theta_speed_ ) current_vel.angular.z = max_theta_speed_;
                if(current_vel.angular.z < -max_theta_speed_ ) current_vel.angular.z = -max_theta_speed_;


                float kp_x = kp_x_set_;
                float kd_x = kp_x_set_*30*kd_x_set_;
                float ki_x = kp_x_set_/30/ki_x_set_;

                error_temp1 = diff_distance - error_x_last_;
                error_x_sum_ += diff_distance;
                if(error_x_sum_>3.0) error_x_sum_ = 3.0;
                if(error_x_sum_<-3.0) error_x_sum_ = -3.0;

                current_vel.linear.x = kp_x*diff_distance + kd_x*error_temp1 + ki_x*error_x_sum_;
                if(current_vel.linear.x > max_x_speed_ ) current_vel.linear.x = max_x_speed_;
                if(current_vel.linear.x < -max_x_speed_ ) current_vel.linear.x = -max_x_speed_;
                if(diff_distance>=0)
                {
                  current_vel.linear.x = max_x_speed_;
                }
                else
                {
                  current_vel.linear.x = -max_x_speed_;
                }
                current_vel.linear.y = 0;
                current_vel.linear.z = 0;
                current_vel.angular.x = 0;
                current_vel.angular.y = 0;

                dealBar();

                if(current_vel.angular.z>0 && !rot_uncounter_enable_) current_vel.angular.z=0;
                if(current_vel.angular.z<0 && !rot_counter_enable_) current_vel.angular.z=0;
                if(!move_forward_enable_ && current_vel.linear.x>0) current_vel.linear.x = 0;

                mCmdvelPub_.publish(current_vel);
                if(!move_forward_enable_  && current_goal_.distance >0 && rot_counter_enable_ && rot_uncounter_enable_)
                {
                  error_theta_last_ = 0;
                  error_theta_sum_ = 0;
                  error_x_last_ = 0;
                  error_x_sum_ = 0;
                  mdo_status_ = DO_STATUS::complete;//
                  return false;
                }
                if(std::fabs(current_vel.angular.z)>0.01||std::fabs(current_vel.linear.x)>0.03)
                {
                  moving_time_ = ros::WallTime::now();
                }
                else
                {
                  ros::WallDuration free_diff = ros::WallTime::now() - moving_time_;
                  if(free_diff.toSec()>5.0)
                  {
                    error_theta_last_ = 0;
                    error_theta_sum_ = 0;
                    error_x_last_ = 0;
                    error_x_sum_ = 0;
                    mdo_status_ = DO_STATUS::complete;//
                    return false;
                  }
                }
            }
            error_theta_last_ = diff_theta;
            error_x_last_ = diff_distance;
          }

          break;
        }
        case DO_STATUS::linear2:
        {
          float diff_theta,diff_distance;
          bool error_flag = getForwardError(diff_theta,diff_distance);
          use_artag_ref_ready_ = true;
          float ar_dist;
          bool online_flag = false;
          float e_y;
          getArtagError(diff_theta, e_y, ar_dist, online_flag);
          ROS_DEBUG("linear2.1.-2 %f %f %f",diff_theta,diff_distance,ar_dist);
          if(std::fabs(diff_distance) <= x_torelance_ || (diff_distance*current_goal_.distance)<0.0001 || (current_goal_.distance <0 &&ar_dist < artag_min_dist_))
          {
            ROS_DEBUG("linear2.1.0 %f %f %f",diff_theta,ar_dist,x_torelance_);
            error_theta_last_ = 0;
            error_theta_sum_ = 0;
            error_x_last_ = 0;
            error_x_sum_ = 0;
            mdo_status_ = DO_STATUS::rotate4;//再原地旋转对准一下角度
            current_vel.linear.x = 0;
            current_vel.linear.y = 0;
            current_vel.linear.z = 0;
            current_vel.angular.x = 0;
            current_vel.angular.y = 0;
            current_vel.angular.z = 0;
            mCmdvelPub_.publish(current_vel);
            ros::Duration(0.2).sleep(); //延时200ms，让直线运动更准同时识别二维码。
          }
          else
          {
            diff_theta = e_y;
            //pid对准
            float kp_theta = kp_line_;
            float kd_theta = kp_line_*30*kd_line_;
            float ki_theta = kp_line_/30/ki_line_;

            float error_temp1 = diff_theta - error_theta_last_;
            error_theta_sum_ += diff_theta;
            if(error_theta_sum_>3.0) error_theta_sum_ = 3.0;
            if(error_theta_sum_<-3.0) error_theta_sum_ = -3.0;

            current_vel.angular.z = kp_theta*diff_theta + kd_theta*error_temp1 + ki_theta*error_theta_sum_;
            if(current_vel.angular.z > max_theta_speed_ ) current_vel.angular.z = max_theta_speed_;
            if(current_vel.angular.z < -max_theta_speed_ ) current_vel.angular.z = -max_theta_speed_;


            float kp_x = kp_x_set_;
            float kd_x = kp_x_set_*30*kd_x_set_;
            float ki_x = kp_x_set_/30/ki_x_set_;

            error_temp1 = diff_distance - error_x_last_;
            error_x_sum_ += diff_distance;
            if(error_x_sum_>3.0) error_x_sum_ = 3.0;
            if(error_x_sum_<-3.0) error_x_sum_ = -3.0;

            current_vel.linear.x = kp_x*diff_distance + kd_x*error_temp1 + ki_x*error_x_sum_;
            if(current_vel.linear.x > max_x_speed_ ) current_vel.linear.x = max_x_speed_;
            if(current_vel.linear.x < -max_x_speed_ ) current_vel.linear.x = -max_x_speed_;
            if(diff_distance>=0)
            {
              current_vel.linear.x = max_x_speed_;
            }
            else
            {
              current_vel.linear.x = -max_x_speed_;
            }
            current_vel.linear.y = 0;
            current_vel.linear.z = 0;
            current_vel.angular.x = 0;
            current_vel.angular.y = 0;

            mCmdvelPub_.publish(current_vel);
            ROS_ERROR("linear2.1.1 %f %f %f",diff_theta,error_theta_sum_,current_vel.angular.z);
          }
          error_theta_last_ = diff_theta;
          error_x_last_ = diff_distance;
          break;
        }
        case DO_STATUS::linear3:
        {
          float diff_theta,diff_distance;
          bool error_flag = getForwardError(diff_theta,diff_distance);
          if(diff_distance<0)
          {
            diff_theta = -diff_theta;
          }
          ROS_DEBUG("linear3.1 %f %f",diff_theta,diff_distance);

          if(std::fabs(diff_distance) <= x_torelance_ || diff_distance<0.0001) //这里只有前进
          {
              ROS_DEBUG("linear3.1.2 %f %f %f",diff_theta,diff_distance,x_torelance_);
              error_theta_last_ = 0;
              error_theta_sum_ = 0;
              error_x_last_ = 0;
              error_x_sum_ = 0;
              //回转90度
              //先转90度
              if(last_e_y_ > 0)
              {
                update_goal_all(m_pi/2.0, current_goal_.distance);
              }
              else
              {
                update_goal_all(-m_pi/2.0, current_goal_.distance);
              }
              mdo_status_ = DO_STATUS::rotate1;//回到第一步
              current_vel.linear.x = 0;
              current_vel.linear.y = 0;
              current_vel.linear.z = 0;
              current_vel.angular.x = 0;
              current_vel.angular.y = 0;
              current_vel.angular.z = 0;
              mCmdvelPub_.publish(current_vel);
          }
          else
          {
              //pid对准
              float kp_theta = kp_theta_set_;
              float kd_theta = kp_theta_set_*30*kd_theta_set_;
              float ki_theta = kp_theta_set_/30/ki_theta_set_;

              float error_temp1 = diff_theta - error_theta_last_;
              error_theta_sum_ += diff_theta;
              if(error_theta_sum_>3.0) error_theta_sum_ = 3.0;
              if(error_theta_sum_<-3.0) error_theta_sum_ = -3.0;

              current_vel.angular.z = kp_theta*diff_theta + kd_theta*error_temp1 + ki_theta*error_theta_sum_;
              if(current_vel.angular.z > max_theta_speed_ ) current_vel.angular.z = max_theta_speed_;
              if(current_vel.angular.z < -max_theta_speed_ ) current_vel.angular.z = -max_theta_speed_;


              float kp_x = kp_x_set_;
              float kd_x = kp_x_set_*30*kd_x_set_;
              float ki_x = kp_x_set_/30/ki_x_set_;

              error_temp1 = diff_distance - error_x_last_;
              error_x_sum_ += diff_distance;
              if(error_x_sum_>3.0) error_x_sum_ = 3.0;
              if(error_x_sum_<-3.0) error_x_sum_ = -3.0;

              current_vel.linear.x = kp_x*diff_distance + kd_x*error_temp1 + ki_x*error_x_sum_;
              if(current_vel.linear.x > max_x_speed_ ) current_vel.linear.x = max_x_speed_;
              if(current_vel.linear.x < -max_x_speed_ ) current_vel.linear.x = -max_x_speed_;
              if(diff_distance>=0)
              {
                current_vel.linear.x = max_x_speed_;
              }
              else
              {
                current_vel.linear.x = -max_x_speed_;
              }
              current_vel.linear.y = 0;
              current_vel.linear.z = 0;
              current_vel.angular.x = 0;
              current_vel.angular.y = 0;

              dealBar();

              if(current_vel.angular.z>0 && !rot_uncounter_enable_) current_vel.angular.z=0;
              if(current_vel.angular.z<0 && !rot_counter_enable_) current_vel.angular.z=0;
              if(!move_forward_enable_ && current_vel.linear.x>0) current_vel.linear.x = 0;

              mCmdvelPub_.publish(current_vel);
              if(!move_forward_enable_  && current_goal_.distance >0 && rot_counter_enable_ && rot_uncounter_enable_)
              {
                error_theta_last_ = 0;
                error_theta_sum_ = 0;
                error_x_last_ = 0;
                error_x_sum_ = 0;
                mdo_status_ = DO_STATUS::complete;//
                return false;
              }
              if(std::fabs(current_vel.angular.z)>0.01||std::fabs(current_vel.linear.x)>0.03)
              {
                moving_time_ = ros::WallTime::now();
              }
              else
              {
                ros::WallDuration free_diff = ros::WallTime::now() - moving_time_;
                if(free_diff.toSec()>5.0)
                {
                  error_theta_last_ = 0;
                  error_theta_sum_ = 0;
                  error_x_last_ = 0;
                  error_x_sum_ = 0;
                  mdo_status_ = DO_STATUS::complete;//
                  return false;
                }
              }
          }
          error_theta_last_ = diff_theta;
          error_x_last_ = diff_distance;

          break;
        }
        case DO_STATUS::complete:
            current_vel.linear.x = 0;
            current_vel.linear.y = 0;
            current_vel.linear.z = 0;
            current_vel.angular.x = 0;
            current_vel.angular.y = 0;
            current_vel.angular.z = 0;
            mCmdvelPub_.publish(current_vel);
            break;
    }
    return true;
}

void MoveController::getThetaError(float & e_theta)
{
    boost::mutex::scoped_lock lock(mMutex_pose);
    float x,y,theta;
    x = mRobot_pose_.position.x;
    y = mRobot_pose_.position.y;
    tf::Quaternion q1(mRobot_pose_.orientation.x, mRobot_pose_.orientation.y, mRobot_pose_.orientation.z,
                      mRobot_pose_.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    theta = yaw;

    e_theta = goal_pose[2] - theta;
    if(e_theta > 3.1415926) e_theta = e_theta - 2*3.1415926;
    if(e_theta < -3.1415926) e_theta = e_theta + 2*3.1415926;
    return ;
}

void MoveController::getArtagError(float & e_theta, float & e_y, float & ar_dist, bool &  online_flag)
{
  //e_theta = -Tbo_y
  //e_x 二维码中垂面与base_link的x轴交点坐标，用来表示车需要直线移动多少距离才可以进入中线
  //ar_dist 摄像头到二维码平面的距离
  //online_flag base_link原点是否在二维码中垂面设定距离范围内
  cv::Mat Tbo = getTbo();
  cv::Mat Tob = Tbo.inv();
  cv::Mat Tco = getTco();
  cv::Mat Toc = Tco.inv();
  ar_dist = std::fabs(Toc.at<float>(2,3)); //

  float Tbo_x = Tbo.at<float>(0,3);
  float Tbo_y = Tbo.at<float>(1,3);

  e_y = - Tbo_y;

  if(std::fabs(e_y)<artag_line_width_)
  {
    online_flag = true;
  }
  else
  {
    online_flag = false;
  }

  double roll, pitch, yaw;
  tf::Matrix3x3 Rbo(Tbo.at<float>(0, 0), Tbo.at<float>(0, 1), Tbo.at<float>(0, 2),
                             Tbo.at<float>(1, 0), Tbo.at<float>(1, 1), Tbo.at<float>(1, 2),
                             Tbo.at<float>(2, 0), Tbo.at<float>(2, 1), Tbo.at<float>(2, 2));
  Rbo.getRPY(roll, pitch, yaw);
  e_theta = yaw - m_pi/2.0;

}


bool MoveController::getForwardError(float & e_theta,float & e_x)
{
    boost::mutex::scoped_lock lock(mMutex_pose);
    float x,y,theta;
    x = mRobot_pose_.position.x;
    y = mRobot_pose_.position.y;
    tf::Quaternion q1(mRobot_pose_.orientation.x, mRobot_pose_.orientation.y, mRobot_pose_.orientation.z,
                      mRobot_pose_.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    theta = yaw;
    if(current_goal_.method==1 && !force_no_ref_)
    {
        boost::mutex::scoped_lock lock(mMutex_scan);
        bool find_flag = findNewGoalPose();
        if(!find_flag) return false;
        float dx = center_[0];
        float dy =center_[1];
        e_x = dx;
        e_theta = dy;
        if(current_goal_.distance<0)
        {
          dx = goal_pose[0] - x;
          dy = goal_pose[1] - y;
          e_x = dx*cos(theta) + dy*sin(theta);
          e_theta = -e_theta;
        }
        return true;
    }
    float dx = goal_pose[0] - x;
    float dy = goal_pose[1] - y;
    e_x = dx*cos(theta) + dy*sin(theta);
    e_theta = -dx*sin(theta) + dy*cos(theta);
    return true;
}

bool MoveController::findNewGoalPose()
{
    if(center14_ready_)
    {
      mRobot_pose_boxedge_ = mRobot_pose_;
      inbox_distance_ = 0;
      mRobot_pose_boxedge_ready_ = true;
    }
    else{
      if(mRobot_pose_boxedge_ready_)
      {
        float x0,y0,x1,y1;
        x0 = mRobot_pose_.position.x;
        y0 = mRobot_pose_.position.y;
        x1 = mRobot_pose_boxedge_.position.x;
        y1 = mRobot_pose_boxedge_.position.y;
        inbox_distance_ = std::sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1));
      }
      else
      {
        inbox_distance_ = 0;
      }

    }
    return center_ready_;
}

void MoveController::dealBar()
{
    boost::mutex::scoped_lock lock(mMutex_scan);
    move_forward_enable_ = true;
    rot_counter_enable_ = true; //顺时针旋转-
    rot_uncounter_enable_ = true; //逆时针旋转+
    if(force_no_bar_) return;
    float x,y;
    for(int i =0 ;i<=scandata1_num_ - 1;i++)
    {
      x = scandata1_[i][0];
      y = scandata1_[i][1];
      if(x>=bar_x_min_ && x<=bar_x_max_ && y>=bar_y_min_ && y<= bar_y_max_)
      {
        ROS_DEBUG("uncounter %f %f",x,y);
        move_forward_enable_ = false;
        //rot_uncounter_enable_ = false; // >0.15
      }
    }
    for(int i =0 ;i<=scandata2_num_ - 1;i++)
    {
      x = scandata2_[i][0];
      y = scandata2_[i][1];
      if(x>=bar_x_min_ && x<=bar_x_max_ && y>=bar_y_min_ && y<= bar_y_max_)
      {
        ROS_DEBUG("counter %f %f",x,y);
        move_forward_enable_ = false;
        //rot_counter_enable_ = false; //<-0.15
      }
    }
}

void MoveController::updateOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_pose_.header.frame_id = msg->header.frame_id;
    robot_pose_.pose = msg->pose.pose;
    robot_pose_.header.stamp = ros::Time();//获取最近时间的map坐标系下姿态
    if(mTf_flag_)
    {
      try
      {
        tf2_buffer_.transform(robot_pose_, global_pose_, global_frame_);
      }
      catch (tf2::LookupException& ex)
      {
        boost::mutex::scoped_lock lock(mMutex_pose);
        ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
        mTf_flag_ = false;
        mPose_flag_ = false;
        return;
      }
      {
        boost::mutex::scoped_lock lock(mMutex_pose);
        mRobot_pose_ = global_pose_.pose;
        mPose_flag_ = true;

        last_odomtime_ = ros::WallTime::now();

        mTdb_.at<float>(0, 3) = mRobot_pose_.position.x;
        mTdb_.at<float>(1, 3) = mRobot_pose_.position.y;
        mTdb_.at<float>(2, 3) = mRobot_pose_.position.z;
        tf::Matrix3x3 Rbc(tf::Quaternion(mRobot_pose_.orientation.x, mRobot_pose_.orientation.y, mRobot_pose_.orientation.z, mRobot_pose_.orientation.w));
        for (int i = 0; i < 3; i++)
        {
            tf::Vector3 v = Rbc.getColumn(i);
            mTdb_.at<float>(0, i) = v.getX();
            mTdb_.at<float>(1, i) = v.getY();
            mTdb_.at<float>(2, i) = v.getZ();
        }
      }
    }
    else
    {
      std::string tf_error;
      if(tf2_buffer_.canTransform(global_frame_, std::string("odom"), ros::Time(), ros::Duration(0.1), &tf_error))
      {
        mTf_flag_ = true;
      }
      else
      {
        mTf_flag_ = false;
        ROS_DEBUG("Timed out waiting for transform from %s to %s to become available before running costmap, tf error: %s",
               "odom", global_frame_.c_str(), tf_error.c_str());
      }
      {
        boost::mutex::scoped_lock lock(mMutex_pose);
        mPose_flag_ = false;
      }
    }
}
void MoveController::updateScan(const sensor_msgs::LaserScan& scan_in)
{
    boost::mutex::scoped_lock lock(mMutex_scan);
    filterscan(scan_in);
    dealscan();
    return;
}

void MoveController::filterscan(const sensor_msgs::LaserScan& scan_in)
{
    //过滤,筛选出有效数据
    scandata1_num_ = 0;
    scandata2_num_ = 0;

    size_t n_pts = scan_in.ranges.size ();
    Eigen::ArrayXXd ranges (n_pts, 2);
    Eigen::ArrayXXd output (n_pts, 2);

    // Get the ranges into Eigen format
    for (size_t i = 0; i < n_pts; ++i)
    {
      ranges (i, 0) = (double) scan_in.ranges[i];
      ranges (i, 1) = (double) scan_in.ranges[i];
    }

    // Check if our existing co_sine_map is valid
    if (co_sine_map_.rows () != (int)n_pts || angle_min_ != scan_in.angle_min || angle_max_ != scan_in.angle_max )
    {
      ROS_DEBUG ("[projectLaser] No precomputed map given. Computing one.");
      co_sine_map_ = Eigen::ArrayXXd (n_pts, 2);
      angle_min_ = scan_in.angle_min;
      angle_max_ = scan_in.angle_max;
      // Spherical->Cartesian projection
      for (size_t i = 0; i < n_pts; ++i)
      {
        co_sine_map_ (i, 0) = cos (scan_in.angle_min + (double) i * scan_in.angle_increment);
        co_sine_map_ (i, 1) = sin (scan_in.angle_min + (double) i * scan_in.angle_increment);
      }
    }

    output = ranges * co_sine_map_;

    float x1,y1;

    float range;
    for (size_t i = 0; i < n_pts; ++i)
    {
      //check to see if we want to keep the point
      float angle_now = scan_in.angle_min + i * scan_in.angle_increment;
      range = scan_in.ranges[i];
      if ( !std::isinf((double)range) && !std::isnan((double)range) && range <= scan_in.range_max && range >= scan_in.range_min)
      {
        // Copy XY
        x1 = R_laserscan_[0]*output(i, 0)+R_laserscan_[1]*output(i,1) + T_laserscan_[0];
        y1 = R_laserscan_[2]*output(i, 0)+R_laserscan_[3]*output(i,1) + T_laserscan_[1];
        if(x1>=filter_x_min_ && x1<=filter_x_max_ )
        {
          if(angle_now<=-1.6)
          {
            //存入scandata1_
            scandata1_[scandata1_num_][0] = x1;
            scandata1_[scandata1_num_][1] = y1;
            scandata1_[scandata1_num_][2] = angle_now;
            scandata1_num_ ++;
          }
          if(angle_now>=1.6)
          {
            //存入scandata2_
            scandata2_[scandata2_num_][0] = x1;
            scandata2_[scandata2_num_][1] = y1;
            scandata2_[scandata2_num_][2] = angle_now;
            scandata2_num_ ++;
          }
        }
      }

    }
}

void MoveController::dealscan()
{
  //获取雷达数据的角点和重心
  //    2***3
  //    *   *
  //  **1   4**
  float corner1_[2];
  float corner2_[2];
  float corner3_[2];
  float corner4_[2];

  corner1_[0] = 0;
  corner1_[1] = 0;
  int corner1_index_ = -1;
  corner2_[0] = 0;
  corner2_[1] = 0;
  int corner2_index_ = -1;
  corner3_[0] = 0;
  corner3_[1] = 0;
  int corner3_index_ = -1;
  corner4_[0] = 0;
  corner4_[1] = 0;
  int corner4_index_ = -1;

  float x0,y0;
  float x1,y1;
  float x2,y2;
  float distance1;
  float distance2;

  float last_diff_angle1 = -1;
  float last_diff_angle2 = -1;
  float last_diff_angle3 = -1;
  float last_diff_angle4 = -1;

  int maybe_corner1_index =-1;
  int maybe_corner2_index =-1;
  int maybe_corner3_index =-1;
  int maybe_corner4_index =-1;

  //1 2
  for(int i = 2 ;i<=scandata1_num_-2;i++ )
  {

    if(corner2_index_>=0 && corner1_index_>=0) break;

    x0 = scandata1_[i][0];
    y0 = scandata1_[i][1];
    distance1 = 0;
    for(int j = i-1;j>=0;j--)
    {
      x1 = scandata1_[j][0];
      y1 = scandata1_[j][1];
      distance1 = (x0-x1)*(x0-x1) + (y0-y1)*(y0-y1);
      if(distance1>=0.0025) break;
    }
    distance2 = 0;
    for(int j = i+1;j<=scandata1_num_-1;j++)
    {
      x2 = scandata1_[j][0];
      y2 = scandata1_[j][1];
      distance2 = (x0-x2)*(x0-x2) + (y0-y2)*(y0-y2);
      if(distance2>=0.0025) break;
    }
    if(distance1>=0.0025 && distance2>=0.0025)
    {
      float angle1,angle2;
      angle1 = std::atan2(y1-y0,x1-x0);
      angle2 = std::atan2(y2-y0,x2-x0);
      float diff_angle = angle2 - angle1;
      if(diff_angle>m_pi)
      {
        diff_angle = diff_angle - 2*m_pi;
      }
      if(diff_angle<-m_pi)
      {
        diff_angle = diff_angle + 2*m_pi;
      }
      //ROS_ERROR("scandata1 %f, %f %f %f ",diff_angle,x0,y0,scandata1_[i][2]);
      if(diff_angle<=-1.0/3.0*m_pi && diff_angle>= -2.0/3.0*m_pi && corner2_index_<0 && corner1_index_<0)
      {
        float diff_angle_temp =  std::fabs(diff_angle+1.0/2.0*m_pi);
        if(diff_angle_temp>m_pi)
        {
          diff_angle_temp = diff_angle_temp-m_pi;
        }
        //可能是2号角点
        if(maybe_corner2_index<0)
        {
          maybe_corner2_index = i;
          last_diff_angle2 = diff_angle_temp;
        }
        else if( diff_angle_temp<= last_diff_angle2)
        {
          //取相对-90度的最小值
          maybe_corner2_index = i;
          last_diff_angle2 = diff_angle_temp;
        }
      }
      else
      {
        if(maybe_corner2_index>=0)
        {
          //ROS_ERROR("corner 2.0 %f",last_diff_angle2);
          corner2_index_ = maybe_corner2_index;
        }
      }

      if(diff_angle>=1.0/3.0*m_pi && diff_angle<=2.0/3.0*m_pi &&corner1_index_<0)
      {
        float diff_angle_temp =  std::fabs(diff_angle-1.0/2.0*m_pi);
        if(diff_angle_temp>m_pi)
        {
          diff_angle_temp = diff_angle_temp-m_pi;
        }
        //可能是一号角点
        if(maybe_corner1_index<0)
        {
          maybe_corner1_index = i;
          last_diff_angle1 = diff_angle_temp;
        }
        else if( diff_angle_temp <= last_diff_angle1)
        {
          //取相对-90度的最小值
          maybe_corner1_index = i;
          last_diff_angle1 = diff_angle_temp;
        }
      }
      else
      {
        if(maybe_corner1_index>=0)
        {
          //ROS_ERROR("corner 1.0  %f",last_diff_angle1);
          corner1_index_ = maybe_corner1_index;
        }
      }
    }
  }
  if(maybe_corner2_index>=0)
  {
    //ROS_ERROR("corner 2.1 %f",last_diff_angle2);
    corner2_index_ = maybe_corner2_index;
  }

  if(maybe_corner1_index>=0)
  {
    //ROS_ERROR("corner 1.1  %f",last_diff_angle1);
    if(std::fabs(scandata1_[maybe_corner1_index][0])<1.5 && std::fabs(scandata1_[maybe_corner1_index][1])<1.5) corner1_index_ = maybe_corner1_index;
  }

  //3 4
  for(int i = scandata2_num_-2;i>0;i-- )
  {

    if(corner3_index_>=0 && corner4_index_>=0) break;

    x0 = scandata2_[i][0];
    y0 = scandata2_[i][1];
    distance1 = 0;
    for(int j = i+1;j<=scandata2_num_-1;j++)
    {
      x1 = scandata2_[j][0];
      y1 = scandata2_[j][1];
      distance1 = (x0-x1)*(x0-x1) + (y0-y1)*(y0-y1);
      if(distance1>=0.0025) break;

    }
    distance2 = 0;
    for(int j = i-1;j>=0;j--)
    {
      x2 = scandata2_[j][0];
      y2 = scandata2_[j][1];
      distance2 = (x0-x2)*(x0-x2) + (y0-y2)*(y0-y2);
      if(distance2>=0.0025) break;
    }
    if(distance1>=0.0025 && distance2>=0.0025)
    {
      float angle1,angle2;
      angle1 = std::atan2(y1-y0,x1-x0);
      angle2 = std::atan2(y2-y0,x2-x0);

      float diff_angle = angle2 - angle1;
      if(diff_angle>m_pi)
      {
        diff_angle = diff_angle - 2*m_pi;
      }
      if(diff_angle<-m_pi)
      {
        diff_angle = diff_angle + 2*m_pi;
      }
      //ROS_ERROR("scandata2 %f, %f %f %f ",diff_angle,x0,y0,scandata2_[i][2]);
      if(diff_angle<=-1.0/3.0*m_pi && diff_angle>= -2.0/3.0*m_pi && corner4_index_<0 && corner4_index_<0)
      {
        float diff_angle_temp =  std::fabs(diff_angle+1.0/2.0*m_pi);
        if(diff_angle_temp>m_pi)
        {
          diff_angle_temp = diff_angle_temp-m_pi;
        }
        //可能是4号角点
        if(maybe_corner4_index<0)
        {
          maybe_corner4_index = i;
          last_diff_angle4 = diff_angle_temp;
        }
        else if( diff_angle_temp<= last_diff_angle4)
        {
          //取相对-90度的最小值
          maybe_corner4_index = i;
          last_diff_angle4 = diff_angle_temp;
        }
      }
      else
      {
        if(maybe_corner4_index>=0)
        {
          //ROS_ERROR("corner 4.0 %f",last_diff_angle4);
          corner4_index_ = maybe_corner4_index;
        }
      }

      if(diff_angle>=1.0/3.0*m_pi && diff_angle<=2.0/3.0*m_pi &&corner3_index_<0)
      {
        float diff_angle_temp =  std::fabs(diff_angle-1.0/2.0*m_pi);
        if(diff_angle_temp>m_pi)
        {
          diff_angle_temp = diff_angle_temp-m_pi;
        }
        //可能是一号角点
        if(maybe_corner3_index<0)
        {
          maybe_corner3_index = i;
          last_diff_angle3 = diff_angle_temp;
        }
        else if( diff_angle_temp <= last_diff_angle3)
        {
          //取相对-90度的最小值
          maybe_corner3_index = i;
          last_diff_angle3 = diff_angle_temp;
        }
      }
      else
      {
        if(maybe_corner3_index>=0)
        {
          //ROS_ERROR("corner 3.0  %f",last_diff_angle3);
          corner3_index_ = maybe_corner3_index;
        }
      }
    }
  }
  if(maybe_corner3_index>=0 )
  {
    //ROS_ERROR("corner 3.1 %f",last_diff_angle3);
    corner3_index_ = maybe_corner3_index;
  }

  if(maybe_corner4_index>=0)
  {
    //ROS_ERROR("corner 4.1  %f",last_diff_angle4);
    if(std::fabs(scandata2_[maybe_corner4_index][0])<1.5 && std::fabs(scandata2_[maybe_corner4_index][1])<1.5) corner4_index_ = maybe_corner4_index;
  }

  //发布对应marker
  visualization_msgs::Marker points, arrows23, arrows14;
  points.header.frame_id = arrows23.header.frame_id = arrows14.header.frame_id= "laser";
  points.header.stamp = arrows23.header.stamp =arrows14.header.stamp  = ros::Time::now();
  points.ns = arrows23.ns = arrows14.ns = "points_and_arrows";
  points.action = arrows23.action =  arrows14.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;

  points.id = 0;
  arrows23.id = 1;
  arrows14.id = 2;

  points.type = visualization_msgs::Marker::POINTS;
  arrows14.type = visualization_msgs::Marker::ARROW;
  arrows14.type = visualization_msgs::Marker::ARROW;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.05;
  points.scale.y = 0.05;

  arrows23.scale.x = 0.8;
  arrows23.scale.y = 0.03;
  arrows23.scale.z = 0.03;

  arrows14.scale.x = 0.8;
  arrows14.scale.y = 0.03;
  arrows14.scale.z = 0.03;

  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // arrows23 is blue
  arrows23.color.b = 1.0;
  arrows23.color.a = 1.0;

  // arrows14  is red
  arrows14.color.r = 1.0;
  arrows14.color.a = 1.0;

  if(corner1_index_>=0)
  {
    geometry_msgs::Point p;
    p.x = -scandata1_[corner1_index_][0];
    p.y = -scandata1_[corner1_index_][1];
    p.z = 0.0;
    points.points.push_back(p);
  }

  if(corner2_index_>=0)
  {
    geometry_msgs::Point p;
    p.x = -scandata1_[corner2_index_][0];
    p.y = -scandata1_[corner2_index_][1];
    p.z = 0.0;
    points.points.push_back(p);
  }

  if(corner3_index_>=0)
  {
    geometry_msgs::Point p;
    p.x = -scandata2_[corner3_index_][0];
    p.y = -scandata2_[corner3_index_][1];
    p.z = 0.0;
    points.points.push_back(p);
  }

  if(corner4_index_>=0)
  {
    geometry_msgs::Point p;
    p.x = -scandata2_[corner4_index_][0];
    p.y = -scandata2_[corner4_index_][1];
    p.z = 0.0;
    points.points.push_back(p);
  }

  //计算2 3中间垂线，1 4 中间垂线，和所有点的重心（不包括3、4之外的）

  float sum_x = 0,sum_y = 0;
  int sum_num =0;
  for(int i = 0; i<= scandata1_num_ -1; i++)
  {
    if(i== corner1_index_) break;
    sum_x += scandata1_[i][0];
    sum_y += scandata1_[i][1];
    sum_num ++;
  }

  for(int i = std::max(0,corner4_index_); i<= scandata2_num_ -1; i++)
  {
    sum_x += scandata2_[i][0];
    sum_y += scandata2_[i][1];
    sum_num ++;
  }
  if(sum_num>0)
  {
    center_ready_ = true;
    center_[0] = sum_x/sum_num;
    center_[1] = sum_y/sum_num;

    geometry_msgs::Point p;
    p.x = -center_[0];
    p.y = -center_[1];
    p.z = 0.0;
    points.points.push_back(p);
  }

  if(center_ready_||corner1_index_>=0||corner2_index_>=0||corner3_index_>=0||corner4_index_>=0)
  {
    marker_pub_.publish(points);
  }

  //
  if(corner1_index_>=0 && corner4_index_>=0)
  {
    x1 = scandata2_[corner4_index_][0];
    y1 = scandata2_[corner4_index_][1];

    x2 = scandata1_[corner1_index_][0];
    y2 = scandata1_[corner1_index_][1];

    if(std::sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))<=1.0)
    {
      center14_[0] = (x1 + x2)/2.0;
      center14_[1] = (y1 + y2)/2.0;

      float prepend_angle = std::atan2(y2-y1,x2-x1) - m_pi/2.0;
      if(prepend_angle<-m_pi) prepend_angle = prepend_angle + 2*m_pi;

      center14_[2] = prepend_angle;
      //ROS_ERROR("angle14 %f",prepend_angle);
      center14_ready_ =true;

      geometry_msgs::Quaternion quat14 = tf::createQuaternionMsgFromYaw(center14_[2]+m_pi);

      arrows14.pose.position.x = -center14_[0];
      arrows14.pose.position.y = -center14_[1];
      arrows14.pose.position.z = 0.0;
      arrows14.pose.orientation = quat14;

      marker_pub_.publish(arrows14);
    }
    else
    {
      center14_[0] = 0;
      center14_[1] = 0;
      center14_ready_ = false;
    }

  }
  else
  {
    center14_[0] = 0;
    center14_[1] = 0;
    center14_ready_ = false;
  }

  if(corner2_index_>=0 && corner3_index_>=0)
  {
    x1 = scandata2_[corner3_index_][0];
    y1 = scandata2_[corner3_index_][1];

    x2 = scandata1_[corner2_index_][0];
    y2 = scandata1_[corner2_index_][1];

    if(std::sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))<=1.0)
    {
      center23_[0] = (x1 + x2)/2.0;
      center23_[1] = (y1 + y2)/2.0;

      float prepend_angle = std::atan2(y2-y1,x2-x1) -m_pi/2.0;
      if(prepend_angle<-m_pi) prepend_angle = prepend_angle + 2*m_pi;

      center23_[2] = prepend_angle;

      //ROS_ERROR("angle23 %f",prepend_angle);
      center23_ready_ =true;

      geometry_msgs::Quaternion quat23 = tf::createQuaternionMsgFromYaw(center23_[2]+m_pi);

      arrows23.pose.position.x = -center23_[0];
      arrows23.pose.position.y = -center23_[1];
      arrows23.pose.position.z = 0.0;
      arrows23.pose.orientation = quat23;

      marker_pub_.publish(arrows23);
    }
    else
    {
      center23_[0] = 0;
      center23_[1] = 0;
      center23_ready_ = false;
    }

  }
  else
  {
    center23_[0] = 0;
    center23_[1] = 0;
    center23_ready_ = false;
  }

}

void MoveController::updateMarkerPose(const ar_track_alvar_msgs::AlvarMarkers& currentMarkers)
{
    for( auto& marker_msg : currentMarkers.markers)
    {
      if(marker_msg.id == MARKER_ID_DETECTION)
      {
        boost::mutex::scoped_lock lock1(mMutex_armark);
        boost::mutex::scoped_lock lock2(mMutex_pose);

        double roll, pitch, yaw;

        mTco_.at<float>(0, 3) = marker_msg.pose.pose.position.x;
        mTco_.at<float>(1, 3) = marker_msg.pose.pose.position.y;
        mTco_.at<float>(2, 3) = marker_msg.pose.pose.position.z;
        tf::Matrix3x3 Rco(tf::Quaternion(marker_msg.pose.pose.orientation.x, marker_msg.pose.pose.orientation.y, marker_msg.pose.pose.orientation.z, marker_msg.pose.pose.orientation.w));
        for (int i = 0; i < 3; i++)
        {
            tf::Vector3 v = Rco.getColumn(i);
            mTco_.at<float>(0, i) = v.getX();
            mTco_.at<float>(1, i) = v.getY();
            mTco_.at<float>(2, i) = v.getZ();
        }
        cv::Mat Toc = mTco_.inv();
        if(Toc.at<float>(2, 3)<0) return; //错误数据
        tf::Matrix3x3 Roc(Toc.at<float>(0, 0), Toc.at<float>(0, 1), Toc.at<float>(0, 2),
                          Toc.at<float>(1, 0), Toc.at<float>(1, 1), Toc.at<float>(1, 2),
                          Toc.at<float>(2, 0), Toc.at<float>(2, 1), Toc.at<float>(2, 2));
        Roc.getRPY(roll, pitch, yaw);
        Roc.setRPY(roll, pitch, 0); //z轴旋转角度强制为0
        for (int i = 0; i < 3; i++)
        {
            tf::Vector3 v = Roc.getColumn(i);
            Toc.at<float>(0, i) = v.getX();
            Toc.at<float>(1, i) = v.getY();
            Toc.at<float>(2, i) = v.getZ();
        }
        mTco_ = Toc.inv();
        mTbo_ = mTbc_ * mTco_ * mToo_;
        cv::Mat Tob = mTbo_.inv();
        if(Tob.at<float>(2, 3)<0)
        {
          return; //错误数据
        }
        cv::Mat Tdo_now = mTdb_ * mTbo_;
        tf::Matrix3x3 Rdo(Tdo_now.at<float>(0, 0), Tdo_now.at<float>(0, 1), Tdo_now.at<float>(0, 2),
                          Tdo_now.at<float>(1, 0), Tdo_now.at<float>(1, 1), Tdo_now.at<float>(1, 2),
                          Tdo_now.at<float>(2, 0), Tdo_now.at<float>(2, 1), Tdo_now.at<float>(2, 2));
        Rdo.getRPY(roll, pitch, yaw);
        if(resetTdo_flag_)
        {
          resetTdo_flag_ = false;
          Tdo_roll_ = roll;
          Tdo_pitch_ = pitch;
          Tdo_yaw_ = yaw;
          Tdo_x_ = Tdo_now.at<float>(0, 3);
          Tdo_y_ = Tdo_now.at<float>(1, 3);
          Tdo_z_ = Tdo_now.at<float>(2, 3);
        }
        else
        {
          float a = 0.1, b= 1-a;
          Tdo_roll_ = a*Tdo_roll_ + b*roll;
          Tdo_pitch_ = a*Tdo_pitch_ + b*pitch;
          Tdo_yaw_ = a*Tdo_yaw_ + b*yaw;
          Tdo_x_ = a*Tdo_x_ + b*Tdo_now.at<float>(0, 3);
          Tdo_y_ = a*Tdo_y_ + b*Tdo_now.at<float>(1, 3);
          Tdo_z_ = a*Tdo_z_ + b*Tdo_now.at<float>(2, 3);
        }
        Rdo.setRPY(Tdo_roll_,Tdo_pitch_,Tdo_yaw_);
        for (int i = 0; i < 3; i++)
        {
            tf::Vector3 v = Rdo.getColumn(i);
            mTdo_.at<float>(0, i) = v.getX();
            mTdo_.at<float>(1, i) = v.getY();
            mTdo_.at<float>(2, i) = v.getZ();
        }
        mTdo_.at<float>(0, 3) = Tdo_x_ ;
        mTdo_.at<float>(1, 3) = Tdo_y_ ;
        mTdo_.at<float>(2, 3) = Tdo_z_ ;

        mTdo_ready_ = true;
        last_armarktime_ = ros::WallTime::now();

        mTbo_ = mTdb_.inv()*mTdo_;
        tf::Matrix3x3 Rbo(mTbo_.at<float>(0, 0), mTbo_.at<float>(0, 1), mTbo_.at<float>(0, 2),
                               mTbo_.at<float>(1, 0), mTbo_.at<float>(1, 1), mTbo_.at<float>(1, 2),
                               mTbo_.at<float>(2, 0), mTbo_.at<float>(2, 1), mTbo_.at<float>(2, 2));
        Rbo.getRPY(roll, pitch, yaw);

        Tob = mTbo_.inv();
        tf::Matrix3x3 Rob(Tob.at<float>(0, 0), Tob.at<float>(0, 1), Tob.at<float>(0, 2),
                               Tob.at<float>(1, 0), Tob.at<float>(1, 1), Tob.at<float>(1, 2),
                               Tob.at<float>(2, 0), Tob.at<float>(2, 1), Tob.at<float>(2, 2));
        double roll2,pitch2,yaw2;
        Rob.getRPY(roll2, pitch2, yaw2);
        // ROS_ERROR("bo %f %f %f, ob %f %f %f . %f %f %f ,%f %f %f",mTbo_.at<float>(0, 3),mTbo_.at<float>(1, 3),mTbo_.at<float>(2, 3),
        // Tob.at<float>(0, 3),Tob.at<float>(1, 3),Tob.at<float>(2, 3),
        // roll, pitch, yaw,roll2, pitch2, yaw2);
        ROS_DEBUG("bo %f %f %f ,error %f ",mTbo_.at<float>(0, 3),mTbo_.at<float>(1, 3),mTbo_.at<float>(2, 3), yaw-m_pi/2.0);
        ROS_DEBUG("ob %f %f %f ,id %d",Tob.at<float>(0, 3),Tob.at<float>(1, 3),Tob.at<float>(2, 3),marker_msg.id);
      }
    }
}

}//bw_move_local
