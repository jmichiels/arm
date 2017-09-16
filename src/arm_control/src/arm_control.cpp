#include "ros/ros.h"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace arm_control
{
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

            // Get the urdf from the parameter server.
            std::string robot_description_urdf;
            if (!n.getParam("/robot_description", robot_description_urdf)) {
                ROS_ERROR("'robot description' not found");
                return false;
            }

            // Compute the KDL tree of the robot from the urdf.
            KDL::Tree tree;
            if (!kdl_parser::treeFromString(robot_description_urdf, tree)) {
                ROS_ERROR("Failed to construct kdl tree");
                return false;
            }

            // Extract chain for KDL tree.
            KDL::Chain chain;
            if (!tree.getChain("toe", "finger", chain)) {
                ROS_ERROR("Failed to extract kdl chain from tree");
                return false;
            }

            // Init the joint state.
            if (chain.getNrOfJoints() != 5) {
                ROS_ERROR("Invalid number of joints");
                return false;
            }
            state_position = new KDL::JntArray(chain.getNrOfJoints());
            state_velocity = new KDL::JntArray(chain.getNrOfJoints());

            // Init inverse dynamics solver.
            KDL::Vector gravity(0, 0, -9.81);
            solver = new KDL::ChainIdSolver_RNE(chain, gravity);

            return true;
        }

        void update(const ros::Time &time, const ros::Duration &period)
        {

            KDL::Wrenches wrenches(5);
            KDL::JntArray zero(5);
            KDL::JntArray target(5);
            target.data[TOE_FOOT_JOINT] = 0;
            target.data[FOOT_LEG_JOINT] = M_PI / 3;
            target.data[LEG_ARM_JOINT] = M_PI / 3;
            target.data[ARM_HAND_JOINT] = 0;
            target.data[HAND_FINGER_JOINT] = M_PI / 3;
            KDL::JntArray gravity_torques(5);

            // Iterate over the joints to update the state.
            for (JointHandleMap::iterator it = joints_handles.begin(); it != joints_handles.end(); ++it) {
                hardware_interface::JointHandle joint = it->second;

                state_position->data[it->first] = it->second.getPosition();
                state_velocity->data[it->first] = it->second.getVelocity();

                // double error = (M_PI / 4) - joint.getPosition();
                // joint.setCommand(error * 100);
            }

            // Compute the gravity term.
            if (solver->CartToJnt(*state_position, zero, zero, wrenches, gravity_torques) != 0) {
                ROS_ERROR("error computing the gravity term");
            };

            // Iterate over the joints to update the command.
            for (JointHandleMap::iterator it = joints_handles.begin(); it != joints_handles.end(); ++it) {
                hardware_interface::JointHandle joint = it->second;

                double error = (M_PI / 4) - state_position->data[it->first];
                ROS_ERROR("torque: %f\n",gravity_torques.data[it->first]);
                joint.setCommand(gravity_torques.data[it->first] + error * 10);
            }
        }

        void starting(const ros::Time &time)
        {}

        void stopping(const ros::Time &time)
        {}

    private:

        //Joints IDs.
        enum JointID
        {
            TOE_FOOT_JOINT,
            FOOT_LEG_JOINT,
            LEG_ARM_JOINT,
            ARM_HAND_JOINT,
            HAND_FINGER_JOINT,
        };

        // Joints handles.
        typedef std::map<JointID, hardware_interface::JointHandle> JointHandleMap;
        JointHandleMap joints_handles;

        // Inverse Dynamics Solver.
        KDL::ChainIdSolver_RNE *solver;

        // Current joint state.
        KDL::JntArray *state_position, *state_velocity;

    };

    PLUGINLIB_EXPORT_CLASS(arm_control::ArmController, controller_interface::ControllerBase);
} // namespace