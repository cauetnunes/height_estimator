#ifndef HEIGHT_ESTIMATOR_H
#define HEIGHT_ESTIMATOR_H

#include <string>
#include <atomic>
/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>

/* message filters includes */
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

/* include message from MRS package */
#include <mrs_msgs/Float64Stamped.h>

/* include message from PX4 message */
#include <nav_msgs/Odometry.h>


using namespace message_filters;



namespace laser_uav_estimator
{
    class HeightEstimator : public nodelet::Nodelet
    {
    public:
        HeightEstimator() {}
        virtual void onInit() override;

    private:

        std::atomic<bool> is_initialized_ = {false};
        ros::NodeHandle nh_;



      // TESTE FILTRO DE MENSAGENS GARMIN CAMERA PX4
      /*             message filter                  */
        typedef sync_policies::ApproximateTime<mrs_msgs::Float64Stamped, mrs_msgs::Float64Stamped, nav_msgs::Odometry> MySyncPolicy;
        typedef Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
      
      /*             SUBSCRIBERS                     */
        message_filters::Subscriber<mrs_msgs::Float64Stamped> garmin_sub_;
        message_filters::Subscriber<mrs_msgs::Float64Stamped> camera_sub_;
        message_filters::Subscriber<nav_msgs::Odometry> odometry_sub_;
      
      /*             CALLBACK PARA PRINTAR           */
        void callbackMFheight(const mrs_msgs::Float64StampedConstPtr& garmin_msg, const mrs_msgs::Float64StampedConstPtr& camera_msg, const nav_msgs::OdometryConstPtr& odometry_msg);
      
      
      
      




      // TESTE SEM A CAMERA

      /*             message filter  SEM CAMERA       */
      //typedef sync_policies::ApproximateTime<mrs_msgs::Float64Stamped, nav_msgs::Odometry> MySyncPolicy;
      // typedef message_filters::Synchronizer<MySyncPolicy> Sync;
      //boost::shared_ptr<Sync> sync_;


      /*              SUBSCRIBERS SEM CAMERA         */
      //  message_filters::Subscriber<mrs_msgs::Float64Stamped> garmin_sub_;
      //  message_filters::Subscriber<nav_msgs::Odometry> odometry_sub_;


      /*             CALLBACK PARA PRINTAR SEM CAMERA */
      //void callbackMFheight(const mrs_msgs::Float64StampedConstPtr& garmin_msg, const nav_msgs::OdometryConstPtr& odometry_msg);


     


       
    };
    };


#endif // HEIGHT_ESTIMATOR_H
