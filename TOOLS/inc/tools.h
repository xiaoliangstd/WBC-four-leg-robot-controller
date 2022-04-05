#ifndef _TOOLS_H_
#define _TOOLS_H_

#include <Eigen>
#include <cmath>

Eigen::Matrix3d skew(Eigen::Vector3d vec);

Eigen::VectorXd convertVector2VectorXd(std::vector<double> vec);
Eigen::Vector3d getSo3FromEulerAngle(std::vector<double> desired_euler_angle,
                                     std::vector<double> euler_angle);
Eigen::Matrix3d getInertialInGravityFrame(Eigen::Matrix3d inertial, std::vector<double> euler_angle);

Eigen::Matrix3d skew(Eigen::Vector3d vec);
Eigen::Matrix3d expW(Eigen::Vector3d w, double theta);
Eigen::Matrix4d expS(Eigen::VectorXd S, double theta);

Eigen::MatrixXd screwRotx(double theta);
Eigen::MatrixXd screwRoty(double theta);
Eigen::MatrixXd screwRotz(double theta);
Eigen::MatrixXd screwTransl(double x, double y, double z);

using Mat6d = Eigen::Matrix<double, 6, 6>;

Mat6d mcI(double mass, Eigen::Vector3d center_of_mass, Eigen::Matrix3d body_inertial);
Mat6d crm(Eigen::VectorXd vel);
Mat6d crf(Eigen::VectorXd vel);

Eigen::Matrix3d rotx(double x);
Eigen::Matrix3d roty(double y);
Eigen::Matrix3d rotz(double z);

Eigen::Vector4d getAngleAxisFromMat(Eigen::Matrix3d mat);

Eigen::MatrixXd getDynJacobianPinv(Eigen::MatrixXd jacobian, Eigen::MatrixXd mass_mat);
Eigen::MatrixXd pseudoInverse(Eigen::MatrixXd matrix);

Eigen::MatrixXd getNullProjectMatrix(Eigen::MatrixXd jacobian);
Eigen::MatrixXd getDynNullProjectMatrix(Eigen::MatrixXd jacobian, Eigen::MatrixXd mass_mat);

#endif