#include "ros/ros.h"
#include <boost/scoped_ptr.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
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
                joints_positions(5),
                joints_velocities(5),
                target_joints_positions(5),
                target_joints_velocities(5),
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
            id_solver.reset(new KDL::ChainIdSolver_RNE(chain, KDL::Vector(0, 0, -9.81)));

            return true;
        }

        void update(const ros::Time &time, const ros::Duration &period)
        {
            // Iterate over the joints to update the current state.
            for (JointHandleMap::iterator it = joints_handles.begin(); it != joints_handles.end(); ++it) {

                joints_positions.data[it->first] = it->second.getPosition();
                joints_velocities.data[it->first] = it->second.getVelocity();
            }

            // Iterate over the joints to compute outer loop control (just a PD)
            for (JointHandleMap::iterator it = joints_handles.begin(); it != joints_handles.end(); ++it) {

                outer_loop_control.data[it->first] =
                        (target_joints_positions.data[it->first] - joints_positions.data[it->first]) * 100
                        + (target_joints_velocities.data[it->first] - joints_velocities.data[it->first]) * 10;
            }

            // No external forces (except gravity).
            KDL::Wrenches external_forces(5);

            // Solve inverse dynamics (inner loop control).dd
            if (id_solver->CartToJnt(
                    joints_positions,
                    joints_velocities,
                    outer_loop_control,
                    external_forces,
                    inner_loop_control) != 0) {
                ROS_ERROR("error solving inverse dynamics");
                return;
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
        boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver;

        // Joints state.
        KDL::JntArray
                joints_positions, target_joints_positions,
                joints_velocities, target_joints_velocities,
                inner_loop_control,
                outer_loop_control;
    };

    PLUGINLIB_EXPORT_CLASS(arm_control::ArmController, controller_interface::ControllerBase);
} // namespace