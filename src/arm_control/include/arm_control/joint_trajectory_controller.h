#ifndef ARM_CONTROL__JOINT_TRAJECTORY_CONTROLLER_H
#define ARM_CONTROL__JOINT_TRAJECTORY_CONTROLLER_H

#include "ros/ros.h"
#include <pluginlib/class_loader.h>
#include <arm_control/effort_joint_interface.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace arm_control
{
//    typedef controller_interface::Controller<hardware_interface::EffortJointInterface> PublicControllerInterface;


    typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
            hardware_interface::EffortJointInterface> JointTrajectoryController;

//    class JointTrajectoryController : public PublicControllerInterface
//    {
//    public:
//
//        // Methods of the public interface,
////
////        bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
////        {
////            return controller.init(static_cast<arm_control::EffortJointInterface *>(hw), root_nh, controller_nh);
////        }
//
//        bool initRequest(hardware_interface::RobotHW *robot_hw,
//                         ros::NodeHandle &root_nh,
//                         ros::NodeHandle &controller_nh,
//                         ClaimedResources &claimed_resources)
//        {
//            return controller.initRequest(hw,root_nh,controller_nh,claimed_resources);
//        }
//
//
//        // Methods of the private interface.
//
//        void update(const ros::Time &time, const ros::Duration &period)
//        {
//            controller.update(time, period);
//        }
//
//        void starting(const ros::Time &time)
//        {
//            controller.starting(time);
//        }
//
//        void stopping(const ros::Time &time)
//        {
//            controller.stopping(time);
//        }
//
//    private:
//        PrivateControllerInterface controller;
//    };

} // namespace

#endif