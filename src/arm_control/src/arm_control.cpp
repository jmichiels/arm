#include "ros/ros.h"
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace arm_control {

    class ArmController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
        {
            // Get the joint handles (throws on failure).
            this->joints_handles[TOE_FOOT_JOINT] = hw->getHandle("toe_foot_joint");
            this->joints_handles[FOOT_LEG_JOINT] = hw->getHandle("foot_leg_joint");
            this->joints_handles[LEG_ARM_JOINT] = hw->getHandle("leg_arm_joint");
            this->joints_handles[ARM_HAND_JOINT] = hw->getHandle("arm_hand_joint");
            this->joints_handles[HAND_FINGER_JOINT] = hw->getHandle("hand_finger_joint");

            return true;
        }

        void update(const ros::Time &time, const ros::Duration &period)
        {
            for (JointHandleMap::iterator it=joints_handles.begin(); it!=joints_handles.end(); ++it) {
                hardware_interface::JointHandle joint = it->second;

                double error = (M_PI/4) - joint.getPosition();
                joint.setCommand(error*100);
            }
        }

        void starting(const ros::Time &time)
        {}

        void stopping(const ros::Time &time)
        {}

    private:
        enum JointID
        {
            TOE_FOOT_JOINT,
            FOOT_LEG_JOINT,
            LEG_ARM_JOINT,
            ARM_HAND_JOINT,
            HAND_FINGER_JOINT,
        };

        // Joints handles.
        typedef std::map<JointID,hardware_interface::JointHandle> JointHandleMap;
        JointHandleMap joints_handles;
    };

    PLUGINLIB_EXPORT_CLASS(arm_control::ArmController, controller_interface::ControllerBase);
} // namespace