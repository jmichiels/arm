#include "ros/ros.h"
#include <kdl/jntarrayvel.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace joint_trajectory_controller
{
    typedef trajectory_interface::QuinticSplineSegment<double> SegmentImpl;
    typedef JointTrajectorySegment<SegmentImpl> Segment;
    typedef typename Segment::State State;
}

template <>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, joint_trajectory_controller::State>
{
public:
    HardwareInterfaceAdapter() :
            joint_handles_ptr(0)
    {}

    bool init(std::vector<hardware_interface::JointHandle> &joint_handles, ros::NodeHandle &nh)
    {
        ROS_INFO("Init inverse dynamics interface");

        // Store pointer to joint handles
        joint_handles_ptr = &joint_handles;

        // Get the URDF from the parameter server.
        std::string robot_description_urdf;
        if (!nh.getParam("/robot_description", robot_description_urdf)) {
            ROS_ERROR("'robot description' not found");
            return false;
        }

        // Compute the KDL tree of the robot from the URDF.
        KDL::Tree tree;
        if (!kdl_parser::treeFromString(robot_description_urdf, tree)) {
            ROS_ERROR("failed to construct kdl tree");
            return false;
        }

        // Extract chain from KDL tree.
        KDL::Chain chain;
        if (!tree.getChain("toe", "finger", chain)) {
            ROS_ERROR("failed to extract kdl chain from tree between");
            return false;
        }

        // Init inverse dynamics solver.
        id_solver.reset(new KDL::ChainIdSolver_RNE(chain, KDL::Vector(0, 0, -9.81)));

        // Reset and resize joint states.
        unsigned int n_joints = chain.getNrOfJoints();
        joints_state.resize(n_joints);
        joints_target.resize(n_joints);
        inner_loop_control.resize(n_joints);
        outer_loop_control.resize(n_joints);

        return true;
    }

    void starting(const ros::Time & /*time*/)
    {
        if (!joint_handles_ptr) { return; }

        for (unsigned int idx = 0; idx < joint_handles_ptr->size(); ++idx)
        {
            // Write joint effort command.
            (*joint_handles_ptr)[idx].setCommand(0);
        }
    }

    void stopping(const ros::Time & /*time*/)
    {}

    void updateCommand(const ros::Time &     /*time*/,
                       const ros::Duration & /*period*/,
                       const joint_trajectory_controller::State &desired_state,
                       const joint_trajectory_controller::State &state_error)
    {
        if (!joint_handles_ptr) { return; }

        for (size_t idx = 0; idx < joint_handles_ptr->size(); ++idx) {

            // Update joint state with current position (q) and velocity (qdot).
            joints_state.q.data[idx] = (*joint_handles_ptr)[idx].getPosition();
            joints_state.qdot.data[idx] = (*joint_handles_ptr)[idx].getVelocity();

            // Compute outer loop control.
            // todo: dynamic reconfigure parameters.
            outer_loop_control.data[idx] = 100 * state_error.position[idx] + 10 * state_error.velocity[idx];
        }

        // No external forces (except gravity).
        KDL::Wrenches external_forces(joint_handles_ptr->size());

        // Solve inverse dynamics (inner loop control).
        if (id_solver->CartToJnt(
                joints_state.q,
                joints_state.qdot,
                outer_loop_control,
                external_forces,
                inner_loop_control) != 0) {
            ROS_ERROR("error solving inverse dynamics");
            return;
        };

        for (unsigned int idx = 0; idx < joint_handles_ptr->size(); ++idx)
        {
            // Write joint effort command.
            (*joint_handles_ptr)[idx].setCommand(inner_loop_control.data[idx]);
        }
    }

private:

    // Joints handles.
    std::vector<hardware_interface::JointHandle> *joint_handles_ptr;

    // Inverse Dynamics Solver.
    boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver;

    // Joints state.
    KDL::JntArrayVel
            joints_state,
            joints_target;

    // Joints commands.
    KDL::JntArray
            inner_loop_control,
            outer_loop_control;
};
