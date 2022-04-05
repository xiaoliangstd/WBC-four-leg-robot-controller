#include "KFStateEsimator.h"
#include <iostream>

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

KFStateEstimator::KFStateEstimator(double dt, double swing_leg_vel_noise, double stance_leg_vel_noise)
    : A(18, 18),
      C(24, 18),
      Xs(18),
      Yo(24),
      Q(18, 18),
      R(24, 24),
      P(18, 18),
      dt_(dt),
      swing_leg_vel_noise_(swing_leg_vel_noise),
      stance_leg_vel_noise_(stance_leg_vel_noise)
{
    Xs.setZero();
    Yo.setZero();
    A.setIdentity();
    C.setZero();
    P.setZero();
    Q.setZero();
    R.setZero();

    Xs.head(3) << 0.0, 0.0, 0.3;
    Xs.tail(12) << 0.17, -0.13, 0.0,
        0.17, 0.13, 0.0,
        -0.17, -0.13, 0.0,
        -0.17, 0.13, 0.0;

    Yo.head(12) << 0.17, -0.13, -0.3,
        0.17, 0.13, -0.3,
        -0.17, -0.13, -0.3,
        -0.17, 0.13, -0.3;

    A.block(0, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * dt;

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        C.block(3 * leg_id, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * (-1);
        C.block(3 * leg_id, 6, 3, 3).setIdentity();
        C.block(3 * leg_id + 12, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * (-1);
    }

    P = Eigen::MatrixXd::Identity(18, 18) * 1e-4;
    Q = Eigen::MatrixXd::Identity(18, 18) * 1e-4;
    R = Eigen::MatrixXd::Identity(24, 24) * stance_leg_vel_noise_;
}

void KFStateEstimator::run(std::vector<double> foot_pos_in_gravity_frame,
                           std::vector<double> foot_velocity_in_gravity_frame,
                           std::vector<double> angular_velocity_in_gravity_frame,
                           std::vector<int> leg_contact_state)
{
    double sum = 0.0;
    int num_of_leg_on_ground = 0;
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        if (leg_contact_state[leg_id] == ON_AIR)
        {
            R.block(3 * leg_id + 12, 3 * leg_id + 12, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * swing_leg_vel_noise_;
        }
        else if (leg_contact_state[leg_id] == ON_GROUND)
        {
            R.block(3 * leg_id + 12, 3 * leg_id + 12, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * stance_leg_vel_noise_;
        }

        Yo.segment(3 * leg_id, 3) << foot_pos_in_gravity_frame[3 * leg_id],
            foot_pos_in_gravity_frame[3 * leg_id + 1],
            foot_pos_in_gravity_frame[3 * leg_id + 2];

        Eigen::Vector3d angular_vel(angular_velocity_in_gravity_frame[0],
                                    angular_velocity_in_gravity_frame[1],
                                    angular_velocity_in_gravity_frame[2]);
        Eigen::Vector3d foot_pos(foot_pos_in_gravity_frame[3 * leg_id],
                                 foot_pos_in_gravity_frame[3 * leg_id + 1],
                                 foot_pos_in_gravity_frame[3 * leg_id + 2]);
        Eigen::Vector3d foot_vel(foot_velocity_in_gravity_frame[3 * leg_id],
                                 foot_velocity_in_gravity_frame[3 * leg_id + 1],
                                 foot_velocity_in_gravity_frame[3 * leg_id + 2]);

        Yo.segment(3 * leg_id + 12, 3) = foot_vel + angular_vel.cross(foot_pos);

        sum += foot_pos(2);
    }

    if (num_of_leg_on_ground != 0)
    {
        estimated_height = fabs(sum) / num_of_leg_on_ground;
    }

    Eigen::VectorXd Xs_prediction = A * Xs;
    Eigen::VectorXd Yo_prediction = C * Xs_prediction;

    Eigen::MatrixXd P_prediction = A * P * A.transpose() + Q;

    Eigen::MatrixXd K = P_prediction * C.transpose();

    Eigen::MatrixXd temp = C * K + R;

    Xs = Xs_prediction + K * temp.lu().solve(Yo - Yo_prediction);

    P = (Eigen::MatrixXd::Identity(18, 18) - K * temp.lu().solve(C)) * P_prediction;
}

std::vector<double> KFStateEstimator::getEstimatedVelocity()
{
    std::vector<double> estimated_vel;

    for (int i = 0; i < 3; i++)
    {
        estimated_vel.push_back(Xs(3 + i));
    }

    return estimated_vel;
}

double KFStateEstimator::getEstimatedHeight()
{
    return estimated_height;
}

PYBIND11_MODULE(KFStateEstimator, m)
{

    py::class_<KFStateEstimator>(m, "KFStateEstimator")
        .def(py::init<double, double, double>())
        .def("run", &KFStateEstimator::run)
        .def("getEstimatedVelocity", &KFStateEstimator::getEstimatedVelocity)
        .def("getEstimatedHeight", &KFStateEstimator::getEstimatedHeight);
}