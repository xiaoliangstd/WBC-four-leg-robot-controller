#include "WBC.h"
#include "tools.h"
#include "Goldfarb_Optimizer/QuadProg++.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

WBC::WBC(double Kp_ori, double Kd_ori,
         double Kp_pos, double Kd_pos,
         double Kp_foot, double Kd_foot,
         std::vector<double> floating_degree_weights,
         double contact_force_weights,
         double coef_friction)
    : joint_pos_(12),
      joint_vel_(12),
      Kp_ori_(Kp_ori),
      Kd_ori_(Kd_ori),
      Kp_pos_(Kp_pos),
      Kd_pos_(Kd_pos),
      Kp_foot_(Kp_foot),
      Kd_foot_(Kd_foot),
      floating_degree_weights_(6),
      contact_force_weights_(contact_force_weights),
      coef_friction_(coef_friction)
{
    desired_euler_.setZero();
    desired_pos_in_gravity_frame_.setZero();

    desired_angular_velocity_in_gravity_frame_.setZero();
    desired_linear_velocity_in_gravity_frame_.setZero();

    desired_angular_acc_in_gravity_frame_.setZero();
    desired_linear_acc_in_gravity_frame_.setZero();

    leg_contact_state_.setZero();

    euler_.setZero();
    pos_in_gravity_frame_.setZero();

    angular_velocity_in_gravity_frame_.setZero();
    linear_velocity_in_gravity_frame_.setZero();

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        desired_foot_pos_in_gravity_frame_[leg_id].setZero();
        desired_foot_vel_in_gravity_frame_[leg_id].setZero();
        desired_foot_acc_in_gravity_frame_[leg_id].setZero();
        foot_pos_in_gravity_frame_[leg_id].setZero();
        foot_vel_in_gravity_frame_[leg_id].setZero();
    }

    joint_pos_.setZero();
    joint_vel_.setZero();

    for (int i = 0; i < 6; i++)
    {
        floating_degree_weights_(i) = floating_degree_weights[i];
    }
}

void WBC::updateInfo(std::vector<double> desired_euler,
                     std::vector<double> desired_pos_in_gravity_frame,
                     std::vector<double> desired_foot_pos_in_gravity_frame,

                     std::vector<double> desired_angular_velocity_in_gravity_frame,
                     std::vector<double> desired_linear_velocity_in_gravity_frame,
                     std::vector<double> desired_foot_vel_in_gravity_frame,

                     std::vector<double> desired_angular_acc_in_gravity_frame,
                     std::vector<double> desired_linear_acc_in_gravity_frame,
                     std::vector<double> desired_foot_acc_in_gravity_frame,

                     std::vector<int> leg_contact_state,

                     std::vector<double> euler,

                     std::vector<double> angular_velocity_in_gravity_frame,
                     std::vector<double> linear_velocity_in_gravity_frame,

                     std::vector<double> joint_pos,
                     std::vector<double> joint_vel)
{

    for (int i = 0; i < 12; i++)
    {
        joint_pos_(i) = joint_pos[i];
        joint_vel_(i) = joint_vel[i];
    }

    for (int i = 0; i < 3; i++)
    {
        desired_euler_(i) = desired_euler[i];
        desired_pos_in_gravity_frame_(i) = desired_pos_in_gravity_frame[i];
        desired_angular_velocity_in_gravity_frame_(i) =
            desired_angular_velocity_in_gravity_frame[i];
        desired_linear_velocity_in_gravity_frame_(i) =
            desired_linear_velocity_in_gravity_frame[i];

        desired_angular_acc_in_gravity_frame_(i) =
            desired_angular_acc_in_gravity_frame[i];

        desired_linear_acc_in_gravity_frame_(i) =
            desired_linear_acc_in_gravity_frame[i];

        euler_(i) = euler[i];

        angular_velocity_in_gravity_frame_(i) =
            angular_velocity_in_gravity_frame[i];

        linear_velocity_in_gravity_frame_(i) = linear_velocity_in_gravity_frame[i];
    }

    for (int i = 0; i < 4; i++)
    {
        leg_contact_state_(i) = leg_contact_state[i];
        foot_pos_in_gravity_frame_[i] = robot_dynamics.getFootPosInGravityFrame(i, joint_pos_.segment(3 * i, 3),
                                                                                euler_);
        foot_vel_in_gravity_frame_[i] = robot_dynamics.getFootVelInGravityFrame(i, joint_pos_.segment(3 * i, 3),
                                                                                joint_vel_.segment(3 * i, 3),
                                                                                euler_);

        for (int j = 0; j < 3; j++)
        {
            desired_foot_pos_in_gravity_frame_[i](j) =
                desired_foot_pos_in_gravity_frame[3 * i + j];

            desired_foot_vel_in_gravity_frame_[i](j) =
                desired_foot_vel_in_gravity_frame[3 * i + j];

            desired_foot_acc_in_gravity_frame_[i](j) =
                desired_foot_acc_in_gravity_frame[3 * i + j];
        }
    }

    double sum = 0.0;
    int num_of_leg_on_ground = 0;
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state_(leg_id) == 1)
        {
            sum += foot_pos_in_gravity_frame_[leg_id](2);
            num_of_leg_on_ground++;
        }
    }

    pos_in_gravity_frame_.setZero();
    if (num_of_leg_on_ground != 0)
    {
        pos_in_gravity_frame_(2) = fabs(sum) / num_of_leg_on_ground;
    }
    else
    {
        pos_in_gravity_frame_(2) = 0.3;
    }
}

std::vector<double> WBC::getJointCmd(std::vector<double> desired_euler,
                                     std::vector<double> desired_pos_in_gravity_frame,
                                     std::vector<double> desired_foot_pos_in_gravity_frame,

                                     std::vector<double> desired_angular_velocity_in_gravity_frame,
                                     std::vector<double> desired_linear_velocity_in_gravity_frame,
                                     std::vector<double> desired_foot_vel_in_gravity_frame,

                                     std::vector<double> desired_angular_acc_in_gravity_frame,
                                     std::vector<double> desired_linear_acc_in_gravity_frame,
                                     std::vector<double> desired_foot_acc_in_gravity_frame,

                                     std::vector<int> leg_contact_state,

                                     std::vector<double> euler,

                                     std::vector<double> angular_velocity_in_gravity_frame,
                                     std::vector<double> linear_velocity_in_gravity_frame,

                                     std::vector<double> joint_pos,
                                     std::vector<double> joint_vel)
{
    updateInfo(desired_euler,
               desired_pos_in_gravity_frame,
               desired_foot_pos_in_gravity_frame,

               desired_angular_velocity_in_gravity_frame,
               desired_linear_velocity_in_gravity_frame,
               desired_foot_vel_in_gravity_frame,

               desired_angular_acc_in_gravity_frame,
               desired_linear_acc_in_gravity_frame,
               desired_foot_acc_in_gravity_frame,

               leg_contact_state,

               euler,

               angular_velocity_in_gravity_frame,
               linear_velocity_in_gravity_frame,

               joint_pos,
               joint_vel);

    // for (int leg_id = 0; leg_id < 4; leg_id++)
    // {
    //     std::cout << desired_foot_pos_in_gravity_frame_[leg_id].transpose() << "\n";
    // }
    // std::cout << "\n\n";

    int num_of_leg_on_ground = 0;
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        num_of_leg_on_ground += leg_contact_state_(leg_id);
    }

    // std::cout << num_of_leg_on_ground << "\n\n";

    Eigen::MatrixXd J1 = getBodyOrientationTaskJacobian();
    Eigen::MatrixXd J2 = getBodyPositionTaskJacobian();
    Eigen::MatrixXd J3 = getSwingFootPositionTaskJacobian();
    Eigen::MatrixXd Jc(3 * num_of_leg_on_ground, 18);

    // std::cout << J3 << "\n\n";

    Eigen::VectorXd JdotQdot_3 = getSwingFootPositionJdotQdot();

    // std::cout << JdotQdot_3 << "\n\n";

    Eigen::VectorXd desired_delta_q(18);
    Eigen::VectorXd desired_qd(18);
    Eigen::VectorXd desired_qdd(18);

    desired_delta_q.setZero();
    desired_qd.setZero();
    desired_qdd.setZero();

    Eigen::MatrixXd mass_mat = robot_dynamics.getMassMatrix(joint_pos_);

    // std::cout << mass_mat.topLeftCorner(6, 6) << "\n\n";

    // Eigen::MatrixXd Jc_pinv = pseudoInverse(Jc);
    Eigen::MatrixXd Nc(18, 18);
    Nc.setIdentity();
    Eigen::MatrixXd Nc_dyn(18, 18);
    Nc_dyn.setIdentity();
    if (num_of_leg_on_ground > 0)
    {
        Jc = getStanceFootPositionTaskJacobian();
        // std::cout << Jc.middleRows(3 * 3, 3).middleCols(15, 3) << "\n\n";
        Eigen::VectorXd JcdotQdot = getStanceFootPositionJdotQdot();
        // std::cout << JcdotQdot << "\n\n";
        Eigen::MatrixXd Jc_dyn_pinv = getDynJacobianPinv(Jc, mass_mat);
        // std::cout << Jc_dyn_pinv << "\n\n";
        Nc = getNullProjectMatrix(Jc);
        // std::cout << Nc.rows() << "\t" << Nc.cols() << "\n\n";
        Nc_dyn = getDynNullProjectMatrix(Jc, mass_mat);
        // std::cout << Nc_dyn.rows() << "\t" << Nc_dyn.cols() << "\n\n";
        desired_qdd = Jc_dyn_pinv * (-JcdotQdot);
        // std::cout << desired_qdd << "\n\n";
    }

    // std::cout << Jc * desired_qdd << "\n\n";

    Eigen::MatrixXd J10 = J1 * Nc;
    Eigen::MatrixXd J10_dyn = J1 * Nc_dyn;
    Eigen::MatrixXd J10_pinv = pseudoInverse(J10);
    Eigen::MatrixXd J10_dyn_pinv = getDynJacobianPinv(J10_dyn, mass_mat);
    Eigen::MatrixXd N1 = Nc * getNullProjectMatrix(J10);
    Eigen::MatrixXd N1_dyn = Nc_dyn * getDynNullProjectMatrix(J10_dyn, mass_mat);

    Eigen::Vector3d so3_error = getBodyOrientationError();
    desired_delta_q += J10_pinv * so3_error;
    desired_qd += J10_pinv * desired_angular_velocity_in_gravity_frame_;
    Eigen::Vector3d cmd_angular_acc = desired_angular_acc_in_gravity_frame_ +
                                      Kp_ori_ * so3_error +
                                      Kd_ori_ * getBodyAngularVelocityError();
    desired_qdd += J10_dyn_pinv * (cmd_angular_acc - J1 * desired_qdd);

    // std::cout << J1 * desired_qdd << "\n\n";

    // std::cout << J1 * desired_delta_q << "\n\n";

    // std::cout << J1 * desired_qd << "\n\n";

    // std::cout << Jc * desired_qdd << "\n\n";

    Eigen::MatrixXd J21 = J2 * N1;
    Eigen::MatrixXd J21_dyn = J2 * N1_dyn;
    Eigen::MatrixXd J21_pinv = pseudoInverse(J21);
    Eigen::MatrixXd J21_dyn_pinv = getDynJacobianPinv(J21_dyn, mass_mat);
    Eigen::MatrixXd N2 = N1 * getNullProjectMatrix(J21);
    Eigen::MatrixXd N2_dyn = N1_dyn * getDynNullProjectMatrix(J21_dyn, mass_mat);

    Eigen::Vector3d pos_error = getBodyPosError();
    desired_delta_q += J21_pinv * (pos_error - J2 * desired_delta_q);
    desired_qd += J21_pinv * (desired_linear_velocity_in_gravity_frame_ - J2 * desired_qd);
    Eigen::Vector3d cmd_linear_acc = desired_linear_acc_in_gravity_frame_ +
                                     Kp_pos_ * pos_error +
                                     Kd_pos_ * getBodyLinearVelocityError();
    desired_qdd += J21_dyn_pinv * (cmd_linear_acc - J2 * desired_qdd);

    // std::cout << J2 * desired_qdd << "\n\n";

    // std::cout << J1 * desired_qdd << "\n\n";

    // std::cout << Jc * desired_qdd << "\n\n";

    // std::cout << J2 * desired_qd << "\n\n";

    // std::cout << J1 * desired_qd << "\n\n";

    // std::cout << J2 * desired_delta_q << "\n\n";

    // std::cout << J1 * desired_delta_q << "\n\n";
    // std::cout << J21_dyn_pinv * cmd_linear_acc << "\n\n";

    int num_of_leg_on_air = 4 - num_of_leg_on_ground;

    if (num_of_leg_on_air > 0)
    {
        Eigen::MatrixXd J32 = J3 * N2;
        Eigen::MatrixXd J32_dyn = J3 * N2_dyn;
        Eigen::MatrixXd J32_pinv = pseudoInverse(J32);
        Eigen::MatrixXd J32_dyn_pinv = getDynJacobianPinv(J32_dyn, mass_mat);
        // Eigen::MatrixXd N3 = N2 * getNullProjectMatrix(J32);
        // Eigen::MatrixXd N3_dyn = N2 * getDynNullProjectMatrix(J32_dyn, mass_mat);

        Eigen::VectorXd foot_pos_error = getSwingFootPosError();
        desired_delta_q += J32_pinv * (foot_pos_error - J3 * desired_delta_q);
        desired_qd += J32_pinv * (getSwingFootDesiredVel() - J3 * desired_qd);
        Eigen::VectorXd cmd_foot_acc = getSwingFootDesiredAcc() +
                                       Kp_foot_ * foot_pos_error +
                                       Kd_foot_ * getSwingFootVelError();
        desired_qdd += J32_dyn_pinv * (cmd_foot_acc - JdotQdot_3 - J3 * desired_qdd);
        // std::cout << desired_qdd << "\n\n";
    }

    // std::cout << J3 * desired_qdd + JdotQdot_3 << "\n\n";

    // std::cout << J2 * desired_qdd << "\n\n";

    // std::cout << J1 * desired_qdd << "\n\n";

    // std::cout << Jc * desired_qdd << "\n\n";

    // std::cout << J2 * desired_qd << "\n\n";
    // std::cout << J1 * desired_qd << "\n\n";
    // std::cout << J1 * desired_delta_q << "\n\n";
    // std::cout << J2 * desired_delta_q << "\n\n";

    Eigen::Matrix3d trans_world2body = roty(euler_(1)) * rotx(euler_(0));

    Eigen::VectorXd body_velocity_in_body_frame(6);

    body_velocity_in_body_frame.head(3) = trans_world2body.transpose() * angular_velocity_in_gravity_frame_;
    body_velocity_in_body_frame.tail(3) = trans_world2body.transpose() * linear_velocity_in_gravity_frame_;

    Eigen::VectorXd coriolis_gravity = robot_dynamics.getCoriolisPlusGravityTerm(euler_,
                                                                                 body_velocity_in_body_frame,
                                                                                 joint_pos_,
                                                                                 joint_vel_);

    // std::cout << coriolis_gravity << "\n\n";

    Eigen::MatrixXd Qf(6, 6);
    Qf.setZero();

    for (int i = 0; i < 6; i++)
    {
        Qf(i, i) = floating_degree_weights_[i];
    }

    // std::cout << Qf << "\n\n";

    Eigen::MatrixXd all_leg_jacobian = getAllFourLegJacobian();

    // std::cout << all_leg_jacobian << "\n\n";

    Eigen::MatrixXd G(18, 18);
    Eigen::VectorXd g0(18);
    Eigen::MatrixXd CE(6 + 3 * num_of_leg_on_air, 18);
    Eigen::VectorXd ce0(6 + 3 * num_of_leg_on_air);
    Eigen::MatrixXd CI(5 * num_of_leg_on_ground, 18);
    Eigen::VectorXd ci0(5 * num_of_leg_on_ground);
    Eigen::VectorXd x(18);

    G.setZero();

    G.topLeftCorner(6, 6) = 2.0 * Qf;
    G.bottomRightCorner(12, 12) = 2.0 * Eigen::MatrixXd::Identity(12, 12) * contact_force_weights_;

    // std::cout << G.bottomRightCorner(12, 12) << "\n\n";

    g0.setZero();

    g0.head(6) = -2.0 * Qf * desired_qdd.head(6);

    // std::cout << desired_qdd << "\n\n";

    CE.setZero();

    CE.block(0, 0, 6, 6) = mass_mat.block(0, 0, 6, 6);
    CE.block(0, 6, 6, 12) = -all_leg_jacobian.transpose().topRows(6);

    // std::cout << CE.rows() << "\t" << CE.cols() << "\n\n";

    int index_of_leg_on_air = 0;
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state_(leg_id) == 0)
        {
            CE.block(6 + 3 * index_of_leg_on_air, 6 + 3 * leg_id, 3, 3).setIdentity();
            index_of_leg_on_air++;
            // std::cout << "Done!\n";
        }
    }

    // std::cout << CE << "\n\n";

    ce0.setZero();
    ce0.head(6) = mass_mat.topRightCorner(6, 12) * desired_qdd.tail(12) +
                  coriolis_gravity.head(6);

    // std::cout << ce0 << "\n\n";

    CI.setZero();

    int index_of_leg_on_ground = 0;
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state_(leg_id) == 1)
        {
            CI.block(5 * index_of_leg_on_ground, 6 + 3 * leg_id, 5, 3) << 1.0, 0.0, coef_friction_,
                -1.0, 0.0, coef_friction_,
                0.0, 1.0, coef_friction_,
                0.0, -1.0, coef_friction_,
                0.0, 0.0, 1.0;
            index_of_leg_on_ground++;
        }
    }
    ci0.setZero();

    // std::cout << CI << "\n\n";

    double f = solve_quadprog(G, g0, CE.transpose(), ce0, CI.transpose(), ci0, x);

    Eigen::VectorXd optimized_qdd = desired_qdd;
    optimized_qdd.head(6) = x.head(6);
    Eigen::VectorXd optimized_force = x.tail(12);

    Eigen::VectorXd desired_torque = mass_mat * optimized_qdd + coriolis_gravity -
                                     all_leg_jacobian.transpose() * optimized_force;

    std::vector<double> joint_cmd;

    for (int joint_id = 0; joint_id < 12; joint_id++)
    {
        joint_cmd.push_back(desired_torque(6 + joint_id));
        joint_cmd.push_back(joint_pos_(joint_id) + desired_delta_q(6 + joint_id));
        joint_cmd.push_back(desired_qd(6 + joint_id));
    }

    // std::cout << "desired_qdd:\n"
    //           << desired_qdd.head(6) << "\n\n";

    // std::cout << "optimized_qdd:\n"
    //           << optimized_qdd.head(6) << "\n\n";

    // std::cout << "desired_joint_qdd: \n"
    //           << desired_qdd.tail(12) << "\n\n";

    // std::cout << "desired_joint_qd:\n"
    //           << desired_qd.tail(12) << "\n\n";

    // std::cout << "desired_joint_delta_q:\n"
    //           << desired_delta_q.tail(12) << "\n\n";

    // std::cout << "desired_torque: \n"
    //           << desired_torque << "\n\n";

    // for (int leg_id = 0; leg_id < 4; leg_id++)
    // {
    //     std::cout << optimized_force.segment(3 * leg_id, 3).transpose() << "\n\n";
    // }

    return joint_cmd;
}

Eigen::MatrixXd WBC::getBodyOrientationTaskJacobian()
{
    Eigen::MatrixXd task_jacobian(3, 18);
    task_jacobian.setZero();

    task_jacobian.block(0, 0, 3, 3) = rotz(0.0) * roty(euler_(1)) * rotx(euler_(0));

    return task_jacobian;
}

Eigen::MatrixXd WBC::getBodyPositionTaskJacobian()
{
    Eigen::MatrixXd task_jacobian(3, 18);
    task_jacobian.setZero();

    task_jacobian.block(0, 3, 3, 3) = rotz(0.0) * roty(euler_(1)) * rotx(euler_(0));

    return task_jacobian;
}

Eigen::MatrixXd WBC::getStanceFootPositionTaskJacobian()
{
    int num_of_leg_on_ground = 0;

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        num_of_leg_on_ground += leg_contact_state_(leg_id);
    }

    Eigen::MatrixXd task_jacobian(3 * num_of_leg_on_ground, 18);

    int index_of_leg_on_ground = 0;
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state_(leg_id) == 1)
        {
            task_jacobian.middleRows(3 * index_of_leg_on_ground, 3) =
                robot_dynamics.getContactJacobianInGravityFrame(leg_id, euler_, joint_pos_.segment(3 * leg_id, 3));
            index_of_leg_on_ground++;
        }
    }

    return task_jacobian;
}

Eigen::MatrixXd WBC::getSwingFootPositionTaskJacobian()
{
    int num_of_leg_on_ground = 0;

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        num_of_leg_on_ground += leg_contact_state_(leg_id);
    }

    Eigen::MatrixXd task_jacobian(3 * (4 - num_of_leg_on_ground), 18);

    int index_of_leg_on_air = 0;
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state_(leg_id) == 0)
        {
            task_jacobian.middleRows(3 * index_of_leg_on_air, 3) =
                robot_dynamics.getContactJacobianInGravityFrame(leg_id, euler_, joint_pos_.segment(3 * leg_id, 3));

            task_jacobian.middleRows(3 * index_of_leg_on_air, 3).leftCols(6).setZero();
            index_of_leg_on_air++;
        }
    }

    return task_jacobian;
}

// Eigen::Vector3d WBC::getBodyOrientationJdotQdot()
// {
//     return Eigen::Vector3d::Zero();
// }

// Eigen::Vector3d WBC::getBodyPositionJdotQdot()
// {
//     return Eigen::Vector3d::Zero();
// }
Eigen::VectorXd WBC::getStanceFootPositionJdotQdot()
{
    int num_of_leg_on_ground = 0;

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        num_of_leg_on_ground += leg_contact_state_(leg_id);
    }

    Eigen::VectorXd jdqd(3 * num_of_leg_on_ground);

    Eigen::Matrix3d trans_body2gravity = rotx(-euler_(0)) * roty(-euler_(1));
    Eigen::Vector3d linear_vel_in_body_frame = trans_body2gravity * linear_velocity_in_gravity_frame_;
    Eigen::Vector3d angular_velocity_in_body_frame = trans_body2gravity * angular_velocity_in_gravity_frame_;

    Eigen::VectorXd body_velocity_in_body_frame(6);

    body_velocity_in_body_frame.head(3) = angular_velocity_in_body_frame;
    body_velocity_in_body_frame.tail(3) = linear_vel_in_body_frame;

    int index_of_leg_on_ground = 0;
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state_(leg_id) == 1)
        {
            jdqd.segment(3 * index_of_leg_on_ground, 3) = robot_dynamics.getJcDotQDot(leg_id, euler_,
                                                                                      joint_pos_.segment(3 * leg_id, 3),
                                                                                      body_velocity_in_body_frame,
                                                                                      joint_vel_.segment(3 * leg_id, 3));

            index_of_leg_on_ground++;
        }
    }

    return jdqd;
}

Eigen::VectorXd WBC::getSwingFootPositionJdotQdot()
{
    int num_of_leg_on_ground = 0;

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        num_of_leg_on_ground += leg_contact_state_(leg_id);
    }

    Eigen::VectorXd jdqd(3 * (4 - num_of_leg_on_ground));

    // Eigen::Matrix3d trans_body2gravity = rotx(-euler_(0)) * roty(-euler_(1));
    // Eigen::Vector3d linear_vel_in_body_frame = trans_body2gravity * linear_velocity_in_gravity_frame_;
    // Eigen::Vector3d angular_velocity_in_body_frame = trans_body2gravity * angular_velocity_in_gravity_frame_;

    Eigen::VectorXd body_velocity_in_body_frame(6);

    body_velocity_in_body_frame.setZero();

    // body_velocity_in_body_frame.head(3) = angular_velocity_in_body_frame;
    // body_velocity_in_body_frame.tail(3) = linear_vel_in_body_frame;

    int index_of_leg_on_air = 0;
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state_(leg_id) == 0)
        {
            jdqd.segment(3 * index_of_leg_on_air, 3) = robot_dynamics.getJcDotQDot(leg_id, euler_,
                                                                                   joint_pos_.segment(3 * leg_id, 3),
                                                                                   body_velocity_in_body_frame,
                                                                                   joint_vel_.segment(3 * leg_id, 3));

            index_of_leg_on_air++;
        }
    }

    return jdqd;
}

Eigen::Vector3d WBC::getBodyOrientationError()
{
    Eigen::Matrix3d desired_rot_mat = roty(desired_euler_(1)) * rotx(desired_euler_(0));

    Eigen::Matrix3d rot_mat = roty(euler_(1)) * rotx(euler_(0));

    Eigen::Matrix3d error_mat = rot_mat.transpose() * desired_rot_mat;

    Eigen::AngleAxisd error_angle_axis(error_mat);

    Eigen::Vector3d error_so3 = error_angle_axis.angle() * error_angle_axis.axis();

    error_so3 = rot_mat * error_so3;

    return error_so3;
}

Eigen::Vector3d WBC::getBodyPosError()
{
    return desired_pos_in_gravity_frame_ - pos_in_gravity_frame_;
}
Eigen::VectorXd WBC::getSwingFootPosError()
{
    int num_of_leg_on_air = 0;

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state_(leg_id) == 0)
        {
            num_of_leg_on_air++;
        }
    }
    Eigen::VectorXd swing_foot_error(3 * num_of_leg_on_air);

    int index_of_leg_on_air = 0;
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state_(leg_id) == 0)
        {
            swing_foot_error.segment(3 * index_of_leg_on_air, 3) =
                desired_foot_pos_in_gravity_frame_[leg_id] - foot_pos_in_gravity_frame_[leg_id];
            index_of_leg_on_air++;
        }
    }

    return swing_foot_error;
}

Eigen::Vector3d WBC::getBodyAngularVelocityError()
{
    return desired_angular_velocity_in_gravity_frame_ - angular_velocity_in_gravity_frame_;
}

Eigen::Vector3d WBC::getBodyLinearVelocityError()
{
    return desired_linear_velocity_in_gravity_frame_ - linear_velocity_in_gravity_frame_;
}

Eigen::VectorXd WBC::getSwingFootVelError()
{
    int num_of_leg_on_air = 0;

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state_(leg_id) == 0)
        {
            num_of_leg_on_air++;
        }
    }
    Eigen::VectorXd swing_foot_error(3 * num_of_leg_on_air);

    int index_of_leg_on_air = 0;
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state_(leg_id) == 0)
        {
            swing_foot_error.segment(3 * index_of_leg_on_air, 3) =
                desired_foot_vel_in_gravity_frame_[leg_id] - foot_vel_in_gravity_frame_[leg_id];
            index_of_leg_on_air++;
        }
    }

    return swing_foot_error;
}

Eigen::VectorXd WBC::getSwingFootDesiredVel()
{
    int num_of_leg_on_air = 0;

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state_(leg_id) == 0)
        {
            num_of_leg_on_air++;
        }
    }
    Eigen::VectorXd swing_foot_desired_vel(3 * num_of_leg_on_air);

    int index_of_leg_on_air = 0;
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state_(leg_id) == 0)
        {
            swing_foot_desired_vel.segment(3 * index_of_leg_on_air, 3) =
                desired_foot_vel_in_gravity_frame_[leg_id];
            index_of_leg_on_air++;
        }
    }

    return swing_foot_desired_vel;
}

Eigen::VectorXd WBC::getSwingFootDesiredAcc()
{
    int num_of_leg_on_air = 0;

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state_(leg_id) == 0)
        {
            num_of_leg_on_air++;
        }
    }
    Eigen::VectorXd swing_foot_desired_acc(3 * num_of_leg_on_air);

    int index_of_leg_on_air = 0;
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state_(leg_id) == 0)
        {
            swing_foot_desired_acc.segment(3 * index_of_leg_on_air, 3) =
                desired_foot_acc_in_gravity_frame_[leg_id];
            index_of_leg_on_air++;
        }
    }

    return swing_foot_desired_acc;
}

Eigen::MatrixXd WBC::getAllFourLegJacobian()
{
    Eigen::MatrixXd jacobian(12, 18);

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        jacobian.middleRows(3 * leg_id, 3) = robot_dynamics.getContactJacobianInGravityFrame(leg_id, euler_,
                                                                                             joint_pos_.segment(3 * leg_id, 3));
    }

    return jacobian;
}

PYBIND11_MODULE(WBC_CONTROLLER, m)
{

    py::class_<WBC>(m, "WBC")
        .def(py::init<double, double, double, double, double, double, std::vector<double>, double, double>())
        .def("getJointCmd", &WBC::getJointCmd);
}