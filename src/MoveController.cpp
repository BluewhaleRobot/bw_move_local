#include "bw_move_local/MoveController.h"

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
        sub1_ = nh_.subscribe("/scan", 1, &MoveController::updateScan, this);
    }
    sub2_ = nh_.subscribe("/odom", 5, &MoveController::updateOdom, this);
    mCmdvelPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    as_.start();
}

void MoveController::executeCB(const galileo_msg::LocalMoveGoalConstPtr &goal)
{
    ROS_ERROR("oups1");
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
            ROS_ERROR("oups2");
            as_.setAborted(galileo_msg::LocalMoveResult(), "Aborting on the goal because pose not ready");
            success = false;
            ros::param::set("/xqserial_server/params/detect_bar", true);
            return;
        }
    }
    this->resetState();
    ROS_ERROR("oups3");
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
        ROS_ERROR("oups4");
        if(!doMove())
        {
            //移动失败，需要设为abort
            //如果进洞距离大于inbox_distance_set_，则认为已经到达目标
            if(inbox_distance_>=inbox_distance_set_)
            {
                ROS_ERROR("oups5");
                break;
            }
            else
            {
                ROS_ERROR("oups6");
                as_.setAborted(galileo_msg::LocalMoveResult(), "Aborting on the goal because some inner error");
                success = false;
            }
            break;
        }
        if(isgoal_reached())
        {
            ROS_ERROR("oups7");
            break;
        }
        as_.publishFeedback(feedback_);
        r.sleep();
    }
    ROS_ERROR("oups8");
    if(success)
    {
        ROS_ERROR("oups9");
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
    ROS_ERROR("reset %f %f %f, %f %f %f",x,y,theta,goal_pose[0],goal_pose[1],goal_pose[2]);
}

bool MoveController::doMove()
{
    boost::mutex::scoped_lock lock(mMutex_move);

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
                ROS_ERROR("prepare1 %f %f ",current_goal_.angle,theta_torelance_);
                theta_goal_reached_ = true;
                mdo_status_ = DO_STATUS::linear1;
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
                ROS_ERROR("rotate1 %f %f ",diff_theta,theta_torelance_);
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
                mCmdvelPub_.publish(current_vel);
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
            ROS_ERROR("linear1.1 %f %f",diff_theta,diff_distance);
            if(std::fabs(diff_distance) <= x_torelance_ || (diff_distance*current_goal_.distance)<0.0001)
            {
                ROS_ERROR("linear1.2 %f %f %f",diff_theta,diff_distance,x_torelance_);
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
                mCmdvelPub_.publish(current_vel);
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

    if(current_goal_.method==1)
    {
        bool find_flag = findNewGoalPose();
        if(!find_flag) return false;
    }

    float dx = goal_pose[0] - x;
    float dy = goal_pose[1] - y;
    e_x = dx*cos(theta) + dy*sin(theta);
    e_theta = -dx*sin(theta) + dy*cos(theta);
    return true;
}

bool MoveController::findNewGoalPose()
{
    return true;
}

void MoveController::dealBar()
{
    move_forward_enable_ = true;
    rot_counter_enable_ = true; //顺时针旋转-
    rot_uncounter_enable_ = true; //逆时针旋转+
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
    return;
}

}//bw_move_local
