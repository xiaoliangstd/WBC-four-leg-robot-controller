#include "tools.h"

Eigen::Matrix3d skew(Eigen::Vector3d vec)
{
    Eigen::Matrix3d mat;

    mat << 0.0, -vec(2), vec(1),
        vec(2), 0.0, -vec(0),
        -vec(1), vec(0), 0.0;

    return mat;
}

Eigen::VectorXd convertVector2VectorXd(std::vector<double> vec)
{
    int len_of_vector = vec.size();

    Eigen::VectorXd vectorXd(len_of_vector);

    for (int i = 0; i < len_of_vector; i++)
    {
        vectorXd(i) = vec[i];
    }

    return vectorXd;
}

Eigen::Vector3d getSo3FromEulerAngle(std::vector<double> desired_euler_angle,
                                     std::vector<double> euler_angle)
{
    Eigen::Matrix3d desired_rot_mat = (Eigen::AngleAxisd(desired_euler_angle[2], Eigen::Vector3d(0, 0, 1)) *
                                       Eigen::AngleAxisd(desired_euler_angle[1], Eigen::Vector3d(0, 1, 0)) *
                                       Eigen::AngleAxisd(desired_euler_angle[0], Eigen::Vector3d(1, 0, 0)))
                                          .matrix();

    Eigen::Matrix3d rot_mat = (Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d(0, 0, 1)) *
                               Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d(0, 1, 0)) *
                               Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d(1, 0, 0)))
                                  .matrix();

    Eigen::Matrix3d error_rot_mat = rot_mat.transpose() * desired_rot_mat;

    Eigen::AngleAxisd error_angle_axis;
    error_angle_axis = error_angle_axis.fromRotationMatrix(error_rot_mat);

    Eigen::Vector3d error_so3 = error_angle_axis.angle() * error_angle_axis.axis();

    error_so3 = rot_mat * error_so3;

    return error_so3;
}

Eigen::Matrix3d getInertialInGravityFrame(Eigen::Matrix3d inertial, std::vector<double> euler_angle)
{
    Eigen::Matrix3d rot_mat = (Eigen::AngleAxisd(0.0, Eigen::Vector3d(0, 0, 1)) *
                               Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d(0, 1, 0)) *
                               Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d(1, 0, 0)))
                                  .matrix();

    return rot_mat * inertial * rot_mat.transpose();
}

Eigen::Matrix3d expW(Eigen::Vector3d w, double theta)
{
    Eigen::Matrix3d w_skew = skew(w);
    double s = sin(theta);
    double c = cos(theta);

    Eigen::Matrix3d exp_w = Eigen::MatrixXd::Identity(3, 3) + s * w_skew + (1 - c) * w_skew * w_skew;

    return exp_w;
}

Eigen::Matrix4d expS(Eigen::VectorXd S, double theta)
{
    Eigen::Vector3d w(S(0), S(1), S(2));
    Eigen::Vector3d v(S(3), S(4), S(5));

    Eigen::Matrix3d w_skew = skew(w);

    double c = cos(theta);
    double s = sin(theta);

    Eigen::Matrix4d exp_s;
    exp_s.setZero();

    exp_s.block(0, 0, 3, 3) = expW(w, theta);

    exp_s.block(0, 3, 3, 1) = (Eigen::MatrixXd::Identity(3, 3) * theta + (1 - c) * w_skew + (theta - s) * w_skew * w_skew) * v;

    exp_s(3, 3) = 1.0;

    return exp_s;
}

Eigen::MatrixXd screwTransl(double x, double y, double z)
{
    Eigen::MatrixXd screw_transl = Eigen::MatrixXd::Identity(6, 6);

    Eigen::Vector3d vec(x, y, z);

    Eigen::Matrix3d r_skew = skew(vec);

    screw_transl.block(3, 0, 3, 3) = -r_skew;

    return screw_transl;
}

Eigen::MatrixXd screwRotx(double theta)
{
    double c = cos(theta);
    double s = sin(theta);

    Eigen::MatrixXd screw_rot(6, 6);
    screw_rot.setZero();

    Eigen::Matrix3d E;
    E << 1.0, 0.0, 0.0,
        0.0, c, s,
        0.0, -s, c;

    screw_rot.block(0, 0, 3, 3) = E;
    screw_rot.block(3, 3, 3, 3) = E;

    return screw_rot;
}

Eigen::MatrixXd screwRoty(double theta)
{
    double c = cos(theta);
    double s = sin(theta);

    Eigen::MatrixXd screw_rot(6, 6);
    screw_rot.setZero();

    Eigen::Matrix3d E;
    E << c, 0.0, -s,
        0.0, 1.0, 0.0,
        s, 0.0, c;

    screw_rot.block(0, 0, 3, 3) = E;
    screw_rot.block(3, 3, 3, 3) = E;

    return screw_rot;
}

Eigen::MatrixXd screwRotz(double theta)
{
    double c = cos(theta);
    double s = sin(theta);

    Eigen::MatrixXd screw_rot(6, 6);
    screw_rot.setZero();

    Eigen::Matrix3d E;
    E << c, s, 0.0,
        -s, c, 0.0,
        0.0, 0.0, 1.0;

    screw_rot.block(0, 0, 3, 3) = E;
    screw_rot.block(3, 3, 3, 3) = E;

    return screw_rot;
}

Mat6d mcI(double mass, Eigen::Vector3d center_of_mass, Eigen::Matrix3d body_inertial)
{
    Mat6d spatial_inertial;

    Eigen::Matrix3d c_skew = skew(center_of_mass);

    spatial_inertial.block(0, 0, 3, 3) = body_inertial - mass * c_skew * c_skew;
    spatial_inertial.block(0, 3, 3, 3) = mass * c_skew;
    spatial_inertial.block(3, 0, 3, 3) = -mass * c_skew;
    spatial_inertial.block(3, 3, 3, 3) = mass * Eigen::MatrixXd::Identity(3, 3);

    return spatial_inertial;
}

Mat6d crm(Eigen::VectorXd vel)
{
    Mat6d vel_cross;
    vel_cross.setZero();

    vel_cross.block(0, 0, 3, 3) = skew(vel.head(3));
    vel_cross.block(3, 3, 3, 3) = skew(vel.head(3));
    vel_cross.block(3, 0, 3, 3) = skew(vel.tail(3));

    return vel_cross;
}

Mat6d crf(Eigen::VectorXd vel)
{
    return -crm(vel).transpose();
}

Eigen::Matrix3d rotx(double x)
{
    Eigen::Matrix3d rot_mat;

    double s = sin(x);
    double c = cos(x);

    rot_mat << 1.0, 0.0, 0.0,
        0.0, c, -s,
        0.0, s, c;

    return rot_mat;
}

Eigen::Matrix3d roty(double y)
{
    Eigen::Matrix3d rot_mat;

    double s = sin(y);
    double c = cos(y);

    rot_mat << c, 0.0, s,
        0.0, 1.0, 0.0,
        -s, 0.0, c;

    return rot_mat;
}

Eigen::Matrix3d rotz(double z)
{
    Eigen::Matrix3d rot_mat;

    double s = sin(z);
    double c = cos(z);

    rot_mat << c, -s, 0.0,
        s, c, 0.0,
        0.0, 0.0, 1;

    return rot_mat;
}

Eigen::MatrixXd getDynJacobianPinv(Eigen::MatrixXd jacobian, Eigen::MatrixXd mass_mat)
{
    Eigen::MatrixXd temp = mass_mat.inverse() * jacobian.transpose();

    Eigen::MatrixXd dyn_jac_pinv = temp * (jacobian * temp).inverse();

    return dyn_jac_pinv;
}

// template <typename T>
Eigen::MatrixXd pseudoInverse(Eigen::MatrixXd matrix)
{

    double sigmaThreshold = 0.001;

    // if ((1 == matrix.rows()) && (1 == matrix.cols()))
    // {
    //     invMatrix.resize(1, 1);
    //     if (matrix.coeff(0, 0) > sigmaThreshold)
    //     {
    //         invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
    //     }
    //     else
    //     {
    //         invMatrix.coeffRef(0, 0) = 0.0;
    //     }
    //     return;
    // }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix,
                                          Eigen::ComputeThinU | Eigen::ComputeThinV);
    // not sure if we need to svd.sort()... probably not
    int const nrows(svd.singularValues().rows());
    Eigen::MatrixXd invS = Eigen::MatrixXd::Zero(nrows, nrows);
    for (int ii(0); ii < nrows; ++ii)
    {
        if (svd.singularValues().coeff(ii) > sigmaThreshold)
        {
            invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
        }
        else
        {
            // invS.coeffRef(ii, ii) = 1.0/ sigmaThreshold;
            // printf("sigular value is too small: %f\n",
            // svd.singularValues().coeff(ii));
        }
    }
    Eigen::MatrixXd invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();

    return invMatrix;
}

Eigen::MatrixXd getNullProjectMatrix(Eigen::MatrixXd jacobian)
{
    Eigen::MatrixXd jac_pinv = pseudoInverse(jacobian);

    return Eigen::MatrixXd::Identity(jacobian.cols(), jacobian.cols()) - jac_pinv * jacobian;
}

Eigen::MatrixXd getDynNullProjectMatrix(Eigen::MatrixXd jacobian, Eigen::MatrixXd mass_mat)
{
    Eigen::MatrixXd jac_pinv = getDynJacobianPinv(jacobian, mass_mat);

    return Eigen::MatrixXd::Identity(jacobian.cols(), jacobian.cols()) - jac_pinv * jacobian;
}
