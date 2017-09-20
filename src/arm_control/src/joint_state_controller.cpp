#include <arm_control/joint_state_controller.hpp>
#include <pluginlib/class_list_macros.h>

namespace arm_control
{
    bool JointStateController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
    {
        // Get the joint handles (throws on failure).
        joints_handles[TOE_FOOT_JOINT] = hw->getHandle("toe_foot_joint");
        joints_handles[FOOT_LEG_JOINT] = hw->getHandle("foot_leg_joint");
        joints_handles[LEG_ARM_JOINT] = hw->getHandle("leg_arm_joint");
        joints_handles[ARM_HAND_JOINT] = hw->getHandle("arm_hand_joint");
        joints_handles[HAND_FINGER_JOINT] = hw->getHandle("hand_finger_joint");

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

        // Reset and resize joint states.
        joints_positions.resize(n_joints);
        joints_velocities.resize(n_joints);
        target_joints_positions.resize(n_joints);
        target_joints_velocities.resize(n_joints);
        inner_loop_control.resize(n_joints);
        outer_loop_control.resize(n_joints);

        // Init inverse dynamics solver.
        id_solver.reset(new KDL::ChainIdSolver_RNE(chain, KDL::Vector(0, 0, -9.81)));

        return true;
    }

    void JointStateController::update(const ros::Time &time, const ros::Duration &period)
    {
        // Iterate over the joints to update the current state.
        for (JointHandleMap::iterator it = joints_handles.begin(); it != joints_handles.end(); ++it) {

            // todo: external control.
            target_joints_positions.data[it->first] = M_PI/4;

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

        // Solve inverse dynamics (inner loop control).
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

            it->second.setCommand(inner_loop_control.data[it->first]);
        }
    }

} // namespace

PLUGINLIB_EXPORT_CLASS(arm_control::JointStateController, controller_interface::ControllerBase);
