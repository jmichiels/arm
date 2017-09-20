#ifndef ARM_CONTROL__JOINT_STATE_CONTROLLER_H
#define ARM_CONTROL__JOINT_STATE_CONTROLLER_H

#include "ros/ros.h"
#include <boost/scoped_ptr.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace arm_control
{
    /*!
    * \brief defines a joint ID.
    */
    enum JointID
    {
        TOE_FOOT_JOINT,
        FOOT_LEG_JOINT,
        LEG_ARM_JOINT,
        ARM_HAND_JOINT,
        HAND_FINGER_JOINT,
    };

    /*!
     * \brief holds joint handles.
     */
    typedef std::map<JointID, hardware_interface::JointHandle> JointHandleMap;

    class JointStateController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:

        /*!
         * \brief  initialize the controller.
         */
        bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

        /*!
         * \brief Issues commands to the joint. Should be called at regular intervals.
         */
        void update(const ros::Time &time, const ros::Duration &period);

    private:

        // Joints handles.
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

} // namespace

#endif