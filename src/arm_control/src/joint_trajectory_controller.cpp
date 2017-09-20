#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
#include <controller_interface/controller_base.h>

namespace arm_control{

    class ArmJointTrajectoryController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
    {
    public:
        bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &n)
        {


            return true;
        }

        void update(const ros::Time &time, const ros::Duration &period)
        {}

        void starting(const ros::Time &time)
        {}

        void stopping(const ros::Time &time)
        {}

    private:

    };
}

PLUGINLIB_EXPORT_CLASS(arm_control::ArmJointTrajectoryController, controller_interface::ControllerBase);