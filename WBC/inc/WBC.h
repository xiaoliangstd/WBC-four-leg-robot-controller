#ifndef _WBC_H_
#define _WBC_H_

#include "floatingBasedDynamics.h"
#include <vector>
#include <Eigen>

class WBC
{
private:
    FloatingBasedDynamics robot_dynamics;

    Eigen::Vector3d desired_euler_;
    Eigen::Vector3d desired_pos_in_gravity_frame_;
    Eigen::Vector3d desired_foot_pos_in_gravity_frame_[4];

    Eigen::Vector3d desired_angular_velocity_in_gravity_frame_;
    Eigen::Vector3d desired_linear_velocity_in_gravity_frame_;
    Eigen::Vector3d desired_foot_vel_in_gravity_frame_[4];

    Eigen::Vector3d desired_angular_acc_in_gravity_frame_;
    Eigen::Vector3d desired_linear_acc_in_gravity_frame_;
    Eigen::Vector3d desired_foot_acc_in_gravity_frame_[4];

    Eigen::Vector4i leg_contact_state_;

    Eigen::Vector3d euler_;
    Eigen::Vector3d pos_in_gravity_frame_;
    Eigen::Vector3d foot_pos_in_gravity_frame_[4];

    Eigen::Vector3d angular_velocity_in_gravity_frame_;
    Eigen::Vector3d linear_velocity_in_gravity_frame_;
    Eigen::Vector3d foot_vel_in_gravity_frame_[4];

    Eigen::VectorXd joint_pos_;
    Eigen::VectorXd joint_vel_;

    double Kp_ori_;
    double Kd_ori_;

    double Kp_pos_;
    double Kd_pos_;

    double Kp_foot_;
    double Kd_foot_;

    Eigen::VectorXd floating_degree_weights_;
    double contact_force_weights_;

    double coef_friction_;

public:
    WBC(double Kp_ori, double Kd_ori,
        double Kp_pos, double Kd_pos,
        double Kp_foot, double Kd_foot,
        std::vector<double> floating_degree_weights,
        double contact_force_weights,
        double coef_friction);
    void updateInfo(std::vector<double> desired_euler,
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
                    std::vector<double> joint_vel);
    std::vector<double> getJointCmd(std::vector<double> desired_euler,
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
                                    std::vector<double> joint_vel);

    Eigen::MatrixXd getBodyOrientationTaskJacobian();
    Eigen::MatrixXd getBodyPositionTaskJacobian();
    Eigen::MatrixXd getStanceFootPositionTaskJacobian();
    Eigen::MatrixXd getSwingFootPositionTaskJacobian();

    // Eigen::Vector3d getBodyOrientationJdotQdot();
    // Eigen::Vector3d getBodyPositionJdotQdot();
    Eigen::VectorXd getStanceFootPositionJdotQdot();
    Eigen::VectorXd getSwingFootPositionJdotQdot();

    Eigen::Vector3d getBodyOrientationError();
    Eigen::Vector3d getBodyPosError();
    Eigen::VectorXd getSwingFootPosError();

    Eigen::Vector3d getBodyAngularVelocityError();
    Eigen::Vector3d getBodyLinearVelocityError();
    Eigen::VectorXd getSwingFootVelError();

    Eigen::VectorXd getSwingFootDesiredVel();
    Eigen::VectorXd getSwingFootDesiredAcc();

    Eigen::MatrixXd getAllFourLegJacobian();
};

#endif