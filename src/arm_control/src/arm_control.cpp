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

        ArmController() :
                state_position(5),
                state_velocity(5),
                target_position(5),
                target_velocity(5),
                inner_loop_control(5),
                outer_loop_control(5)
        {}

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

            // Check the number of joints.
            unsigned int n_joints = chain.getNrOfJoints();
            if (n_joints != 5) {
                ROS_ERROR("Invalid number of joints");
                return false;
            }

            // Init inverse dynamics solver.
            KDL::Vector gravity(0, 0, -9.81);
            solver = new KDL::ChainIdSolver_RNE(chain, gravity);

            return true;
        }

        void update(const ros::Time &time, const ros::Duration &period)
        {
            target_position.data[TOE_FOOT_JOINT] = 0;
            target_position.data[FOOT_LEG_JOINT] = M_PI / 4;
            target_position.data[LEG_ARM_JOINT] = M_PI / 4;
            target_position.data[ARM_HAND_JOINT] = 0;
            target_position.data[HAND_FINGER_JOINT] = M_PI / 2;

            // Iterate over the joints to update the current state.
            for (JointHandleMap::iterator it = joints_handles.begin(); it != joints_handles.end(); ++it) {
                hardware_interface::JointHandle joint = it->second;

                state_position.data[it->first] = joint.getPosition();
                state_velocity.data[it->first] = joint.getVelocity();

                // Compute outer loop control (just a PD)
                outer_loop_control.data[it->first] =
                        - (state_position.data[it->first] - target_position.data[it->first]) * 100
                        - (state_velocity.data[it->first] - target_velocity.data[it->first]) * 10;
            }

            // No external forces (except gravity).
            KDL::Wrenches external_forces(5);

            // Solve inverse dynamics (inner loop control).
            if (solver->CartToJnt(state_position, state_velocity, outer_loop_control, external_forces, inner_loop_control) != 0) {
                ROS_ERROR("error solving inverse dynamics");
            };

            // Apply the computed torques command to each joint.
            for (JointHandleMap::iterator it = joints_handles.begin(); it != joints_handles.end(); ++it) {
                hardware_interface::JointHandle joint = it->second;

                joint.setCommand(inner_loop_control.data[it->first]);
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
        KDL::JntArray state_position, target_position, state_velocity, target_velocity, inner_loop_control, outer_loop_control;
    };

    PLUGINLIB_EXPORT_CLASS(arm_control::ArmController, controller_interface::ControllerBase);
} // namespace