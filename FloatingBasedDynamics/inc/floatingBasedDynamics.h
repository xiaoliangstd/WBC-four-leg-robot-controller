#ifndef _FLOATING_BASED_DYNAMICS_H_
#define _FLOATING_BASED_DYNAMICS_H_

#include <Eigen>
#include <vector>

const double MASS[5] = {4.713,
                        0.696, 1.013, 0.166, 0.06};

const double CENTER_OF_MASS[5][3] = {{0.012731, 0.002186, 0.000515},
                                     {-0.003311, -0.000635, 3.1e-05},
                                     {-0.003237, 0.022327, -0.027326},
                                     {0.006435, 0.0, -0.107388},
                                     {0.0, 0.0, 0.0}};

const double INERTIAL[5][6] = {{0.01683993, 8.3902e-05, 0.000597679, 0.056579028, 2.5134e-05, 0.064713601},
                               {0.000469246, 9.409e-06, -3.42e-07, 0.00080749, 4.66e-07, 0.000552929},
                               {0.005529065, -4.825e-06, 0.000343869, 0.005139339, -2.2448e-05, 0.001367788},
                               {0.002997972, 0.0, -0.000141163, 0.003014022, 0.0, 3.2426e-05},
                               {9.6e-06, 0.0, 0.0, 9.6e-06, 0.0, 9.6e-06}};

const int PARRENT_BODY_INDEX[12] = {0, 1, 2, 0, 4, 5, 0, 7, 8, 0, 10, 11};

using Mat6d = Eigen::Matrix<double, 6, 6>;
using Vec6d = Eigen::Matrix<double, 6, 1>;

enum
{
    NO_FLIP = 0,
    FLIP_X_AXIS,
    FLIP_Y_AXIS,
    FLIP_XY_AXIS
};

const double leg_x_offset = 0.183;
const double leg_y_offset = 0.047;
const double hip_offset = 0.08505;
const double thigh_length = 0.2;
const double calf_length = 0.2;

class FloatingBasedDynamics
{
private:
    std::vector<Eigen::Matrix3d> body_inertial;
    std::vector<Eigen::Vector3d> center_of_mass;
    std::vector<double> mass;
    std::vector<Mat6d> spatial_inertial;
    std::vector<Mat6d> composite_spatial_inertial;
    std::vector<Mat6d> screw_transl_tree;
    std::vector<Mat6d> screw_rot_joint;
    std::vector<Mat6d> screw_parrent_body_to_child_body;
    std::vector<int> parrent_body_index;
    std::vector<Vec6d> joint_motion_screw;

    std::vector<Vec6d> link_vel_in_link_frame;
    std::vector<Vec6d> link_acc_vp_in_link_frame;
    std::vector<Vec6d> link_force_in_link_frame;

    std::vector<Eigen::Matrix<double, 4, 4>> leg_initial_configuration_mat;
    std::vector<Vec6d> initial_joint_axis_screw;

public:
    FloatingBasedDynamics();
    void forwardKinematics(Eigen::VectorXd joints_pos);
    Eigen::MatrixXd getMassMatrix(Eigen::VectorXd joints_pos);
    Eigen::VectorXd getCoriolisPlusGravityTerm(Eigen::Vector3d body_euler_angle,
                                               Vec6d body_velocity_in_body_frame,
                                               Eigen::VectorXd joint_pos,
                                               Eigen::VectorXd joint_velocity);
    Eigen::Matrix3d getFootPostureToBodyFrame(int leg_id, Eigen::Vector3d single_leg_joint_pos);
    Eigen::Vector3d getFootPosInGravityFrame(int leg_id, Eigen::Vector3d single_leg_joint_pos,
                                             Eigen::Vector3d body_euler_angle);
    Eigen::Vector3d getFootVelInGravityFrame(int leg_id, Eigen::Vector3d single_leg_joint_pos,
                                             Eigen::Vector3d single_leg_joint_vel,
                                             Eigen::Vector3d body_euler_angle);
    Eigen::MatrixXd getContactJacobianInGravityFrame(int leg_id, Eigen::Vector3d body_euler_angle,
                                                     Eigen::Vector3d single_leg_joint_pos);
    Eigen::Vector3d getJcDotQDot(int leg_id, Eigen::Vector3d body_euler_angle,
                                 Eigen::Vector3d single_leg_joint_pos,
                                 Vec6d body_velocity_in_body_frame,
                                 Eigen::Vector3d single_leg_joint_vel);

    Eigen::Vector3d getJointAngleFromFootPosInGravityFrame(int leg_id, Eigen::Vector3d desired_foot_pos,
                                                           Eigen::Vector3d body_euler_angle);
};

#endif