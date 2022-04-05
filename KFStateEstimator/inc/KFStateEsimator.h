#ifndef _KF_STATE_ESTIMATOR_H_

#define _KF_STATE_ESTIMATOR_H_

#include <Eigen>
#include <vector>

enum
{
    ON_GROUND = 1,
    ON_AIR = 0
};

class KFStateEstimator
{
private:
    Eigen::MatrixXd A;
    Eigen::MatrixXd C;
    Eigen::VectorXd Xs;
    Eigen::VectorXd Yo;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd P;

    double dt_;

    double estimated_height = 0.3;

    double swing_leg_vel_noise_;
    double stance_leg_vel_noise_;

public:
    KFStateEstimator(double dt, double swing_leg_vel_noise, double stance_leg_vel_noise);
    void run(std::vector<double> foot_pos_in_gravity_frame,
             std::vector<double> foot_velocity_in_gravity_frame,
             std::vector<double> angular_velocity_in_gravity_frame,
             std::vector<int> leg_contact_state);
    std::vector<double> getEstimatedVelocity();
    double getEstimatedHeight();
};

#endif