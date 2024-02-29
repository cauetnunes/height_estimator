#include "laser_uav_estimator/height_estimator.h"

namespace laser_uav_estimator
{
    void HeightEstimator::onInit()
    {
        ros::NodeHandle nh("~");

        ros::Time::waitForValid();

     


        // | --------- initialize message filters subscribers --------- |
        garmin_sub_.subscribe(nh, "garmin_in", 1);
        camera_sub_.subscribe(nh, "camera_in", 1);
        odometry_sub_.subscribe(nh, "odom_height_in", 1);

        sync_.reset(new Sync(MySyncPolicy(10), garmin_sub_, camera_sub_, odometry_sub_));
        sync_->registerCallback(boost::bind(&HeightEstimator::callbackMFheight, this, _1, _2, _3));



        // teste sem camera

        /*              politica de sincronizar sem camera     */

        // sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), garmin_sub_, odometry_sub_);
        // sync_.reset(new Sync(MySyncPolicy(10), garmin_sub_, odometry_sub_));
        // sync_->registerCallback(boost::bind(&HeightEstimator::callbackMFheight, this, _1, _2));

        /* inicializando subscribers sem a camera*/

        //garmin_sub_.subscribe(nh, "garmin_in", 1);
        //odometry_sub_.subscribe(nh, "odom_height_in", 1);



        ROS_INFO("[HeightEstimator]: Initialized.");
        is_initialized_ = true;
    }


   void HeightEstimator::callbackMFheight(const mrs_msgs::Float64StampedConstPtr& garmin_msg, const mrs_msgs::Float64StampedConstPtr& camera_msg, const nav_msgs::OdometryConstPtr& odometry_msg)
    {
        ROS_INFO("CHEGOU NO CALLBACK");
        ROS_INFO("Garmin Height: %f", garmin_msg->value);
        ROS_INFO("Camera Height: %f", camera_msg->value);
        ROS_INFO("Odometry Position Z: %f", odometry_msg->pose.pose.position.z);}
    

  

  /*                CALLBACK PARA TESTAR SEM CAMERA            */


/*void HeightEstimator::callbackMFheight(const mrs_msgs::Float64StampedConstPtr& garmin_msg, const nav_msgs::OdometryConstPtr& odometry_msg)
    {
        ROS_INFO("CHEGOU NO CALLBACK");
        ROS_INFO("Garmin Height: %f", garmin_msg->value);
        ROS_INFO("Odometry Position Z: %f", odometry_msg->pose.pose.position.z);}*/
    



  /*    CALLBACKS SEPARADOS PARA TESTE DE LEITURA DOS TOPICOS      */
  
   /* void HeightEstimator::callback1(const mrs_msgs::Float64StampedConstPtr& garmin_msg) {

        ROS_INFO("cHEGOU NO CALL BACK DO GARMIN");
        ROS_INFO("Garmin height: %f", garmin_msg->value );

    }

     void HeightEstimator::callback2(const mrs_msgs::Float64StampedConstPtr& camera_msg) {

        ROS_INFO("cHEGOU NO CALL BACK Da camera");
        ROS_INFO("Camera height: %f", camera_msg->value );

    }

     void HeightEstimator::callback3(const nav_msgs::OdometryConstPtr& odometry_msg) {

        ROS_INFO("cHEGOU NO CALL BACK Da px4");
        ROS_INFO("Px4 height: %f", odometry_msg->pose.pose.position.z );

    }*/


}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(laser_uav_estimator::HeightEstimator, nodelet::Nodelet)
