#ifndef ARM_CONTROL__JOINT_TRAJECTORY_CONTROLLER_H
#define ARM_CONTROL__JOINT_TRAJECTORY_CONTROLLER_H

#include "ros/ros.h"
#include <pluginlib/class_loader.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <arm_control/effort_joint_interface.h>

namespace arm_control
{
    typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
            arm_control::EffortJointInterface> Controller;

    class JointTrajectoryController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        /*!
         * \brief  initialize the controller.
         */
        bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
        {
            return controller.init(static_cast<arm_control::EffortJointInterface *>(hw), root_nh, controller_nh);
        }

        /*!
         * \brief Issues commands to the joint. Should be called at regular intervals.
         */
        void update(const ros::Time &time, const ros::Duration &period)
        {
            controller.update(time, period);
        }

        /*!
         * \brief Holds the current position.
         */
        void starting(const ros::Time &time)
        {
            controller.starting(time);
        }

        /*!
         * \brief Cancels the active action goal, if any.
         */
        void stopping(const ros::Time &time)
        {
            controller.stopping(time);
        }

    private:
        arm_control::Controller controller;
    };

} // namespace

#endif