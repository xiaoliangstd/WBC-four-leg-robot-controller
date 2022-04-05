#include "floatingBasedDynamics.h"
#include "tools.h"
#include <iostream>

Eigen::Matrix3d inertialAssignment(const double inertial[], int flip_which_axis)
{
    Eigen::Matrix3d inertial_mat;

    if (flip_which_axis == NO_FLIP)
    {
        inertial_mat << inertial[0], -inertial[1], -inertial[2],
            -inertial[1], inertial[3], -inertial[4],
            -inertial[2], -inertial[4], inertial[5];
    }
    else if (flip_which_axis == FLIP_X_AXIS)
    {
        inertial_mat << inertial[0], inertial[1], inertial[2],
            inertial[1], inertial[3], -inertial[4],
            inertial[2], -inertial[4], inertial[5];
    }
    else if (flip_which_axis == FLIP_Y_AXIS)
    {
        inertial_mat << inertial[0], inertial[1], -inertial[2],
            inertial[1], inertial[3], inertial[4],
            -inertial[2], inertial[4], inertial[5];
    }
    else if (flip_which_axis == FLIP_XY_AXIS)
    {
        inertial_mat << inertial[0], -inertial[1], inertial[2],
            -inertial[1], inertial[3], inertial[4],
            inertial[2], inertial[4], inertial[5];
    }

    return inertial_mat;
}

Eigen::Vector3d centerOfMassAssignment(const double center_of_mass[], int flip_which_axis)
{
    Eigen::Vector3d center_of_mass_vec;

    if (flip_which_axis == NO_FLIP)
    {
        center_of_mass_vec << center_of_mass[0], center_of_mass[1], center_of_mass[2];
    }
    else if (flip_which_axis == FLIP_X_AXIS)
    {
        center_of_mass_vec << -center_of_mass[0], center_of_mass[1], center_of_mass[2];
    }
    else if (flip_which_axis == FLIP_Y_AXIS)
    {
        center_of_mass_vec << center_of_mass[0], -center_of_mass[1], center_of_mass[2];
    }
    else if (flip_which_axis == FLIP_XY_AXIS)
    {
        center_of_mass_vec << -center_of_mass[0], -center_of_mass[1], center_of_mass[2];
    }

    return center_of_mass_vec;
}

FloatingBasedDynamics::FloatingBasedDynamics()
    : body_inertial(13),
      center_of_mass(13),
      mass(13),
      spatial_inertial(13),
      composite_spatial_inertial(13),
      screw_transl_tree(12),
      screw_rot_joint(12),
      parrent_body_index(12),
      screw_parrent_body_to_child_body(12),
      joint_motion_screw(12),
      link_vel_in_link_frame(13),
      link_acc_vp_in_link_frame(13),
      link_force_in_link_frame(13),
      leg_initial_configuration_mat(4),
      initial_joint_axis_screw(12)
{
    body_inertial[0] = inertialAssignment(INERTIAL[0], NO_FLIP);

    body_inertial[1] = inertialAssignment(INERTIAL[1], NO_FLIP);
    body_inertial[2] = inertialAssignment(INERTIAL[2], NO_FLIP);
    body_inertial[3] = inertialAssignment(INERTIAL[3], NO_FLIP);

    body_inertial[4] = inertialAssignment(INERTIAL[1], FLIP_Y_AXIS);
    body_inertial[5] = inertialAssignment(INERTIAL[2], FLIP_Y_AXIS);
    body_inertial[6] = inertialAssignment(INERTIAL[3], FLIP_Y_AXIS);

    body_inertial[7] = inertialAssignment(INERTIAL[1], FLIP_X_AXIS);
    body_inertial[8] = inertialAssignment(INERTIAL[2], NO_FLIP);
    body_inertial[9] = inertialAssignment(INERTIAL[3], NO_FLIP);

    body_inertial[10] = inertialAssignment(INERTIAL[1], FLIP_XY_AXIS);
    body_inertial[11] = inertialAssignment(INERTIAL[2], FLIP_Y_AXIS);
    body_inertial[12] = inertialAssignment(INERTIAL[3], FLIP_Y_AXIS);

    // for (int i = 0; i < 13; i++)
    // {
    //     std::cout << i << ":\n"
    //               << body_inertial[i] << "\n\n";
    // }

    center_of_mass[0] = centerOfMassAssignment(CENTER_OF_MASS[0], NO_FLIP);

    center_of_mass[1] = centerOfMassAssignment(CENTER_OF_MASS[1], NO_FLIP);
    center_of_mass[2] = centerOfMassAssignment(CENTER_OF_MASS[2], NO_FLIP);
    center_of_mass[3] = centerOfMassAssignment(CENTER_OF_MASS[3], NO_FLIP);

    center_of_mass[4] = centerOfMassAssignment(CENTER_OF_MASS[1], FLIP_Y_AXIS);
    center_of_mass[5] = centerOfMassAssignment(CENTER_OF_MASS[2], FLIP_Y_AXIS);
    center_of_mass[6] = centerOfMassAssignment(CENTER_OF_MASS[3], FLIP_Y_AXIS);

    center_of_mass[7] = centerOfMassAssignment(CENTER_OF_MASS[1], FLIP_X_AXIS);
    center_of_mass[8] = centerOfMassAssignment(CENTER_OF_MASS[2], NO_FLIP);
    center_of_mass[9] = centerOfMassAssignment(CENTER_OF_MASS[3], NO_FLIP);

    center_of_mass[10] = centerOfMassAssignment(CENTER_OF_MASS[1], FLIP_XY_AXIS);
    center_of_mass[11] = centerOfMassAssignment(CENTER_OF_MASS[2], FLIP_Y_AXIS);
    center_of_mass[12] = centerOfMassAssignment(CENTER_OF_MASS[3], FLIP_Y_AXIS);

    // for (int i = 0; i < 13; i++)
    // {
    //     std::cout << i << ":\n"
    //               << center_of_mass[i] << "\n\n";
    // }

    mass[0] = MASS[0];

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        mass[3 * leg_id + 1] = MASS[1];
        mass[3 * leg_id + 2] = MASS[2];
        mass[3 * leg_id + 3] = MASS[3];
    }

    // for (int i = 0; i < 13; i++)
    // {
    //     std::cout << mass[i] << "\n";
    // }

    for (int i = 0; i < 13; i++)
    {
        spatial_inertial[i] = mcI(mass[i], center_of_mass[i], body_inertial[i]);
    }

    // for (int i = 0; i < 13; i++)
    // {
    //     std::cout << i << ":\n"
    //               << spatial_inertial[i] << "\n\n";
    // }

    Mat6d lower_leg_to_toe_tree = screwTransl(0.0, 0.0, -calf_length);

    // std::cout << lower_leg_to_toe_tree << "\n\n";

    for (int toe = 0; toe < 4; toe++)
    {
        spatial_inertial[3 * toe + 3] += lower_leg_to_toe_tree.transpose() *
                                         mcI(MASS[4], Eigen::Vector3d(0.0, 0.0, 0.0), inertialAssignment(INERTIAL[4], NO_FLIP)) *
                                         lower_leg_to_toe_tree;
    }

    double imu_mass = 0.001;
    double imu_inertial[6] = {0.0001, 0.0, 0.0, 0.0001, 0.0, 0.0001};
    spatial_inertial[0] += mcI(imu_mass, Eigen::Vector3d(0.0, 0.0, 0.0), inertialAssignment(imu_inertial, NO_FLIP));

    Mat6d FR_leg_should_offset_trans = screwTransl(0.0, -0.081, 0.0);
    Mat6d FL_leg_should_offset_trans = screwTransl(0.0, 0.081, 0.0);
    Mat6d RR_leg_should_offset_trans = screwTransl(0.0, -0.081, 0.0);
    Mat6d RL_leg_should_offset_trans = screwTransl(0.0, 0.081, 0.0);

    spatial_inertial[1] += FR_leg_should_offset_trans.transpose() *
                           mcI(imu_mass, Eigen::Vector3d(0.0, 0.0, 0.0), inertialAssignment(imu_inertial, NO_FLIP)) *
                           FR_leg_should_offset_trans;
    spatial_inertial[4] += FL_leg_should_offset_trans.transpose() *
                           mcI(imu_mass, Eigen::Vector3d(0.0, 0.0, 0.0), inertialAssignment(imu_inertial, NO_FLIP)) *
                           FL_leg_should_offset_trans;
    spatial_inertial[7] += RR_leg_should_offset_trans.transpose() *
                           mcI(imu_mass, Eigen::Vector3d(0.0, 0.0, 0.0), inertialAssignment(imu_inertial, NO_FLIP)) *
                           RR_leg_should_offset_trans;
    spatial_inertial[9] += RL_leg_should_offset_trans.transpose() *
                           mcI(imu_mass, Eigen::Vector3d(0.0, 0.0, 0.0), inertialAssignment(imu_inertial, NO_FLIP)) *
                           RL_leg_should_offset_trans;

    // std::cout << mcI(MASS[4], Eigen::Vector3d(0.0, 0.0, 0.0), inertialAssignment(INERTIAL[4], NO_FLIP)) << "\n\n";
    // for (int i = 0; i < 13; i++)
    // {
    //     std::cout << i << ":\n"
    //               << spatial_inertial[i] << "\n\n";
    // }

    screw_transl_tree[0] = screwTransl(leg_x_offset, -leg_y_offset, 0.0);
    screw_transl_tree[1] = screwTransl(0.0, -hip_offset, 0.0);
    screw_transl_tree[2] = screwTransl(0.0, 0.0, -thigh_length);

    screw_transl_tree[3] = screwTransl(leg_x_offset, leg_y_offset, 0.0);
    screw_transl_tree[4] = screwTransl(0.0, hip_offset, 0.0);
    screw_transl_tree[5] = screwTransl(0.0, 0.0, -thigh_length);

    screw_transl_tree[6] = screwTransl(-leg_x_offset, -leg_y_offset, 0.0);
    screw_transl_tree[7] = screwTransl(0.0, -hip_offset, 0.0);
    screw_transl_tree[8] = screwTransl(0.0, 0.0, -thigh_length);

    screw_transl_tree[9] = screwTransl(-leg_x_offset, leg_y_offset, 0.0);
    screw_transl_tree[10] = screwTransl(0.0, hip_offset, 0.0);
    screw_transl_tree[11] = screwTransl(0.0, 0.0, -thigh_length);

    for (int i = 0; i < 12; i++)
    {
        parrent_body_index[i] = PARRENT_BODY_INDEX[i];
        screw_rot_joint[i].setIdentity();
    }

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        for (int joint = 0; joint < 3; joint++)
        {
            if (joint == 0)
            {
                joint_motion_screw[3 * leg_id + joint] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            }
            else
            {
                joint_motion_screw[3 * leg_id + joint] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
            }
        }
    }

    for (int i = 0; i < 4; i++)
    {
        leg_initial_configuration_mat[i].setIdentity();
    }

    leg_initial_configuration_mat[0].block(0, 3, 3, 1) << leg_x_offset, -leg_y_offset - hip_offset, -thigh_length - calf_length;
    leg_initial_configuration_mat[1].block(0, 3, 3, 1) << leg_x_offset, leg_y_offset + hip_offset, -thigh_length - calf_length;
    leg_initial_configuration_mat[2].block(0, 3, 3, 1) << -leg_x_offset, -leg_y_offset - hip_offset, -thigh_length - calf_length;
    leg_initial_configuration_mat[3].block(0, 3, 3, 1) << -leg_x_offset, leg_y_offset + hip_offset, -thigh_length - calf_length;

    initial_joint_axis_screw[0] << 1.0, 0.0, 0.0, 0.0, 0.0, leg_y_offset;
    initial_joint_axis_screw[1] << 0.0, 1.0, 0.0, 0.0, 0.0, leg_x_offset;
    initial_joint_axis_screw[2] << 0, 1, 0, thigh_length, 0, leg_x_offset;

    initial_joint_axis_screw[3] << 1, 0, 0, 0, 0, -leg_y_offset;
    initial_joint_axis_screw[4] << 0, 1, 0, 0, 0, leg_x_offset;
    initial_joint_axis_screw[5] << 0, 1, 0, thigh_length, 0, leg_x_offset;

    initial_joint_axis_screw[6] << 1, 0, 0, 0, 0, leg_y_offset;
    initial_joint_axis_screw[7] << 0, 1, 0, 0, 0, -leg_x_offset;
    initial_joint_axis_screw[8] << 0, 1, 0, thigh_length, 0, -leg_x_offset;

    initial_joint_axis_screw[9] << 1, 0, 0, 0, 0, -leg_y_offset;
    initial_joint_axis_screw[10] << 0, 1, 0, 0, 0, -leg_x_offset;
    initial_joint_axis_screw[11] << 0, 1, 0, thigh_length, 0, -leg_x_offset;
}

void FloatingBasedDynamics::forwardKinematics(Eigen::VectorXd joint_pos)
{
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        for (int joint = 0; joint < 3; joint++)
        {
            if (joint == 0)
            {
                screw_rot_joint[3 * leg_id + joint] = screwRotx(joint_pos[3 * leg_id + joint]);
            }
            else
            {
                screw_rot_joint[3 * leg_id + joint] = screwRoty(joint_pos[3 * leg_id + joint]);
            }
        }
    }

    for (int i = 0; i < 12; i++)
    {
        screw_parrent_body_to_child_body[i] = screw_rot_joint[i] * screw_transl_tree[i];
        // std::cout << i + 1 << ":\n"
        //           << screw_parrent_body_to_child_body[i] << "\n\n";
    }
}

Eigen::MatrixXd FloatingBasedDynamics::getMassMatrix(Eigen::VectorXd joints_pos)
{
    Eigen::MatrixXd mass_matrix(18, 18);
    Eigen::MatrixXd H(12, 12);
    H.setZero();
    Eigen::MatrixXd F(6, 12);

    forwardKinematics(joints_pos);

    for (int i = 0; i < 13; i++)
    {
        composite_spatial_inertial[i] = spatial_inertial[i];
    }

    for (int i = 12; i >= 1; i--)
    {
        composite_spatial_inertial[parrent_body_index[i - 1]] +=
            screw_parrent_body_to_child_body[i - 1].transpose() *
            composite_spatial_inertial[i] * screw_parrent_body_to_child_body[i - 1];

        F.col(i - 1) = composite_spatial_inertial[i] * joint_motion_screw[i - 1];
        H(i - 1, i - 1) = joint_motion_screw[i - 1].transpose() * F.col(i - 1);

        int j = i;
        while (parrent_body_index[j - 1] != 0)
        {
            F.col(i - 1) = screw_parrent_body_to_child_body[j - 1].transpose() * F.col(i - 1);
            j = parrent_body_index[j - 1];
            // std::cout << j << "\n";
            H(i - 1, j - 1) = F.col(i - 1).transpose() * joint_motion_screw[j - 1];
            H(j - 1, i - 1) = H(i - 1, j - 1);
        }
        F.col(i - 1) = screw_parrent_body_to_child_body[j - 1].transpose() * F.col(i - 1);
    }

    mass_matrix.block(0, 0, 6, 6) = composite_spatial_inertial[0];
    mass_matrix.block(0, 6, 6, 12) = F;
    mass_matrix.block(6, 0, 12, 6) = F.transpose();
    mass_matrix.block(6, 6, 12, 12) = H;

    return mass_matrix;
}

Eigen::VectorXd FloatingBasedDynamics::getCoriolisPlusGravityTerm(Eigen::Vector3d body_euler_angle,
                                                                  Vec6d body_velocity_in_body_frame,
                                                                  Eigen::VectorXd joint_pos,
                                                                  Eigen::VectorXd joint_velocity)
{
    forwardKinematics(joint_pos);

    Eigen::VectorXd coriolis_plus_gravity_term(18);

    Eigen::Matrix3d body_posture_mat = rotz(0.0) * roty(body_euler_angle(1)) * rotx(body_euler_angle(0));

    Mat6d body_posture_mat_for_screw;
    body_posture_mat_for_screw.setZero();
    body_posture_mat_for_screw.block(0, 0, 3, 3) = body_posture_mat.transpose();
    body_posture_mat_for_screw.block(3, 3, 3, 3) = body_posture_mat.transpose();

    Vec6d acc_gravity;
    acc_gravity << 0.0, 0.0, 0.0, 0.0, 0.0, -9.8;

    link_acc_vp_in_link_frame[0] = -body_posture_mat_for_screw * acc_gravity;
    link_vel_in_link_frame[0] = body_velocity_in_body_frame;

    Vec6d single_joint_vel;
    for (int i = 1; i < 13; i++)
    {
        single_joint_vel = joint_motion_screw[i - 1] * joint_velocity(i - 1);
        link_vel_in_link_frame[i] =
            screw_parrent_body_to_child_body[i - 1] * link_vel_in_link_frame[parrent_body_index[i - 1]] + single_joint_vel;

        link_acc_vp_in_link_frame[i] =
            screw_parrent_body_to_child_body[i - 1] * link_acc_vp_in_link_frame[parrent_body_index[i - 1]] +
            crm(link_vel_in_link_frame[i]) * single_joint_vel;

        link_force_in_link_frame[i] = spatial_inertial[i] * link_acc_vp_in_link_frame[i] +
                                      crf(link_vel_in_link_frame[i]) * spatial_inertial[i] * link_vel_in_link_frame[i];
    }
    link_force_in_link_frame[0] = spatial_inertial[0] * link_acc_vp_in_link_frame[0] +
                                  crf(link_vel_in_link_frame[0]) * spatial_inertial[0] * link_vel_in_link_frame[0];

    for (int i = 12; i >= 1; i--)
    {
        coriolis_plus_gravity_term(i + 5) = joint_motion_screw[i - 1].transpose() * link_force_in_link_frame[i];

        link_force_in_link_frame[parrent_body_index[i - 1]] +=
            screw_parrent_body_to_child_body[i - 1].transpose() * link_force_in_link_frame[i];
    }

    coriolis_plus_gravity_term.head(6) = link_force_in_link_frame[0];

    return coriolis_plus_gravity_term;
}

Eigen::Matrix3d FloatingBasedDynamics::getFootPostureToBodyFrame(int leg_id, Eigen::Vector3d single_leg_joint_pos)
{
    Eigen::Matrix4d foot_end_posture_and_pos_mat =
        expS(initial_joint_axis_screw[3 * leg_id], single_leg_joint_pos(0)) *
        expS(initial_joint_axis_screw[3 * leg_id + 1], single_leg_joint_pos(1)) *
        expS(initial_joint_axis_screw[3 * leg_id + 2], single_leg_joint_pos(2)) * leg_initial_configuration_mat[leg_id];

    Eigen::Matrix3d foot_posture_to_body_frame = foot_end_posture_and_pos_mat.block(0, 0, 3, 3);

    return foot_posture_to_body_frame;
}

Eigen::MatrixXd FloatingBasedDynamics::getContactJacobianInGravityFrame(int leg_id, Eigen::Vector3d body_euler_angle,
                                                                        Eigen::Vector3d single_leg_joint_pos)
{
    Eigen::MatrixXd jacobian(3, 18);
    jacobian.setZero();

    Eigen::MatrixXd single_leg_jacobian(6, 3);

    Mat6d T_01, T_12, T_23, T_34;

    Eigen::Matrix3d rot_gravity_to_body_frame = rotz(0.0) * roty(body_euler_angle(1)) * rotx(body_euler_angle(0));
    Eigen::Matrix3d rot_body_to_feet_frame = getFootPostureToBodyFrame(leg_id, single_leg_joint_pos);

    Mat6d rot_gravity_to_body_frame_for_screw;
    rot_gravity_to_body_frame_for_screw.setZero();

    rot_gravity_to_body_frame_for_screw.block(0, 0, 3, 3) = rot_gravity_to_body_frame;
    rot_gravity_to_body_frame_for_screw.block(3, 3, 3, 3) = rot_gravity_to_body_frame;

    Mat6d rot_body_to_feet_frame_for_screw;
    rot_body_to_feet_frame_for_screw.setZero();

    rot_body_to_feet_frame_for_screw.block(0, 0, 3, 3) = rot_body_to_feet_frame;
    rot_body_to_feet_frame_for_screw.block(3, 3, 3, 3) = rot_body_to_feet_frame;

    if (leg_id == 0)
    {
        T_01 = screwRotx(single_leg_joint_pos(0)) * screwTransl(leg_x_offset, -leg_y_offset, 0.0);
        T_12 = screwRoty(single_leg_joint_pos(1)) * screwTransl(0.0, -hip_offset, 0.0);
    }
    else if (leg_id == 1)
    {
        T_01 = screwRotx(single_leg_joint_pos(0)) * screwTransl(leg_x_offset, leg_y_offset, 0.0);
        T_12 = screwRoty(single_leg_joint_pos(1)) * screwTransl(0.0, hip_offset, 0.0);
    }
    else if (leg_id == 2)
    {
        T_01 = screwRotx(single_leg_joint_pos(0)) * screwTransl(-leg_x_offset, -leg_y_offset, 0.0);
        T_12 = screwRoty(single_leg_joint_pos(1)) * screwTransl(0.0, -hip_offset, 0.0);
    }
    else if (leg_id == 3)
    {
        T_01 = screwRotx(single_leg_joint_pos(0)) * screwTransl(-leg_x_offset, leg_y_offset, 0.0);
        T_12 = screwRoty(single_leg_joint_pos(1)) * screwTransl(0.0, hip_offset, 0.0);
    }
    T_23 = screwRoty(single_leg_joint_pos(2)) * screwTransl(0.0, 0.0, -thigh_length);
    T_34 = screwTransl(0.0, 0.0, -calf_length);

    Mat6d T_24 = T_34 * T_23;
    Mat6d T_14 = T_24 * T_12;
    Mat6d T_04 = T_14 * T_01;

    single_leg_jacobian.col(0) = T_14 * joint_motion_screw[3 * leg_id];
    single_leg_jacobian.col(1) = T_24 * joint_motion_screw[3 * leg_id + 1];
    single_leg_jacobian.col(2) = T_34 * joint_motion_screw[3 * leg_id + 2];

    single_leg_jacobian =
        rot_gravity_to_body_frame_for_screw * rot_body_to_feet_frame_for_screw * single_leg_jacobian;

    jacobian.block(0, 0, 3, 6) = (rot_gravity_to_body_frame_for_screw * rot_body_to_feet_frame_for_screw * T_04).block(3, 0, 3, 6);
    jacobian.block(0, 3 * leg_id + 6, 3, 3) = single_leg_jacobian.block(3, 0, 3, 3);

    return jacobian;
}

Eigen::Vector3d FloatingBasedDynamics::getJcDotQDot(int leg_id, Eigen::Vector3d body_euler_angle,
                                                    Eigen::Vector3d single_leg_joint_pos,
                                                    Vec6d body_velocity_in_body_frame,
                                                    Eigen::Vector3d single_leg_joint_vel)
{
    std::vector<Vec6d> link_velocity(4);
    std::vector<Vec6d> link_acc_vp(4);
    std::vector<Mat6d> trans_from_parrent_to_child_link(3);
    std::vector<Vec6d> joint_axis(3);

    joint_axis[0] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    joint_axis[1] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    joint_axis[2] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;

    link_velocity[0] = body_velocity_in_body_frame;
    link_acc_vp[0].setZero();

    if (leg_id == 0)
    {
        trans_from_parrent_to_child_link[0] =
            screwRotx(single_leg_joint_pos(0)) * screwTransl(leg_x_offset, -leg_y_offset, 0.0);
        trans_from_parrent_to_child_link[1] =
            screwRoty(single_leg_joint_pos(1)) * screwTransl(0.0, -hip_offset, 0.0);
        trans_from_parrent_to_child_link[2] =
            screwRoty(single_leg_joint_pos(2)) * screwTransl(0.0, 0.0, -thigh_length);
    }
    else if (leg_id == 1)
    {
        trans_from_parrent_to_child_link[0] =
            screwRotx(single_leg_joint_pos(0)) * screwTransl(leg_x_offset, leg_y_offset, 0.0);
        trans_from_parrent_to_child_link[1] =
            screwRoty(single_leg_joint_pos(1)) * screwTransl(0.0, hip_offset, 0.0);
        trans_from_parrent_to_child_link[2] =
            screwRoty(single_leg_joint_pos(2)) * screwTransl(0.0, 0.0, -thigh_length);
    }
    else if (leg_id == 2)
    {
        trans_from_parrent_to_child_link[0] =
            screwRotx(single_leg_joint_pos(0)) * screwTransl(-leg_x_offset, -leg_y_offset, 0.0);
        trans_from_parrent_to_child_link[1] =
            screwRoty(single_leg_joint_pos(1)) * screwTransl(0.0, -hip_offset, 0.0);
        trans_from_parrent_to_child_link[2] =
            screwRoty(single_leg_joint_pos(2)) * screwTransl(0.0, 0.0, -thigh_length);
    }
    else if (leg_id == 3)
    {
        trans_from_parrent_to_child_link[0] =
            screwRotx(single_leg_joint_pos(0)) * screwTransl(-leg_x_offset, leg_y_offset, 0.0);
        trans_from_parrent_to_child_link[1] =
            screwRoty(single_leg_joint_pos(1)) * screwTransl(0.0, hip_offset, 0.0);
        trans_from_parrent_to_child_link[2] =
            screwRoty(single_leg_joint_pos(2)) * screwTransl(0.0, 0.0, -thigh_length);
    }

    Vec6d joint_vel;
    for (int i = 1; i < 4; i++)
    {
        joint_vel = joint_axis[i - 1] * single_leg_joint_vel(i - 1);
        link_velocity[i] = trans_from_parrent_to_child_link[i - 1] * link_velocity[i - 1] + joint_vel;
        link_acc_vp[i] = trans_from_parrent_to_child_link[i - 1] * link_acc_vp[i - 1] +
                         crm(link_velocity[i]) * joint_vel;
    }

    Eigen::Matrix3d rot_gravity_to_body_frame = rotz(0.0) * roty(body_euler_angle(1)) * rotx(body_euler_angle(0));
    Eigen::Matrix3d rot_body_to_feet_frame = getFootPostureToBodyFrame(leg_id, single_leg_joint_pos);

    Mat6d rot_gravity_to_body_frame_for_screw;
    rot_gravity_to_body_frame_for_screw.setZero();

    rot_gravity_to_body_frame_for_screw.block(0, 0, 3, 3) = rot_gravity_to_body_frame;
    rot_gravity_to_body_frame_for_screw.block(3, 3, 3, 3) = rot_gravity_to_body_frame;

    Mat6d rot_body_to_feet_frame_for_screw;
    rot_body_to_feet_frame_for_screw.setZero();

    rot_body_to_feet_frame_for_screw.block(0, 0, 3, 3) = rot_body_to_feet_frame;
    rot_body_to_feet_frame_for_screw.block(3, 3, 3, 3) = rot_body_to_feet_frame;

    Vec6d end_point_screw_acc = rot_gravity_to_body_frame_for_screw *
                                rot_body_to_feet_frame_for_screw * screwTransl(0.0, 0.0, -calf_length) *
                                link_acc_vp[3];
    Vec6d end_point_screw_vel = rot_gravity_to_body_frame_for_screw *
                                rot_body_to_feet_frame_for_screw * screwTransl(0.0, 0.0, -calf_length) *
                                link_velocity[3];

    Eigen::Vector3d end_linear_velocity = end_point_screw_vel.tail(3);
    Eigen::Vector3d end_angular_velocity = end_point_screw_vel.head(3);

    Eigen::Vector3d classic_linear_acc = end_point_screw_acc.tail(3) +
                                         (end_angular_velocity).cross(end_linear_velocity);

    return classic_linear_acc;
}

Eigen::Vector3d FloatingBasedDynamics::getFootPosInGravityFrame(int leg_id, Eigen::Vector3d single_leg_joint_pos,
                                                                Eigen::Vector3d body_euler_angle)
{
    Eigen::Matrix4d foot_end_posture_and_pos_mat =
        expS(initial_joint_axis_screw[3 * leg_id], single_leg_joint_pos(0)) *
        expS(initial_joint_axis_screw[3 * leg_id + 1], single_leg_joint_pos(1)) *
        expS(initial_joint_axis_screw[3 * leg_id + 2], single_leg_joint_pos(2)) * leg_initial_configuration_mat[leg_id];

    Eigen::Vector3d foot_pos_in_body_frame = foot_end_posture_and_pos_mat.topRightCorner(3, 1);

    Eigen::Matrix3d trans_world2body = roty(body_euler_angle(1)) * rotx(body_euler_angle(0));

    return trans_world2body * foot_pos_in_body_frame;
}

Eigen::Vector3d FloatingBasedDynamics::getFootVelInGravityFrame(int leg_id, Eigen::Vector3d single_leg_joint_pos,
                                                                Eigen::Vector3d single_leg_joint_vel,
                                                                Eigen::Vector3d body_euler_angle)
{
    Eigen::Matrix3d jacobian = getContactJacobianInGravityFrame(leg_id, body_euler_angle,
                                                                single_leg_joint_pos);

    return jacobian * single_leg_joint_vel;
}

Eigen::Vector3d FloatingBasedDynamics::getJointAngleFromFootPosInGravityFrame(int leg_id,
                                                                              Eigen::Vector3d desired_foot_pos,
                                                                              Eigen::Vector3d body_euler_angle)
{
    Eigen::Vector3d init_joint_pos(0.0,
                                   (-1.0471975512 + 4.18879020479) / 2.0,
                                   (-2.69653369433 + -0.916297857297) / 2.0);

    // init_joint_pos.setZero();

    // std::cout << init_joint_pos << "\n\n";

    Eigen::Vector3d foot_pos = getFootPosInGravityFrame(leg_id, init_joint_pos, body_euler_angle);

    Eigen::Vector3d error_pos = desired_foot_pos - foot_pos;

    int loop_count = 0;
    while (error_pos.norm() > 1e-10 && loop_count < 100)
    {
        Eigen::Matrix3d jacobian =
            getContactJacobianInGravityFrame(leg_id, body_euler_angle, init_joint_pos).middleCols(6 + 3 * leg_id, 3);

        // std::cout << jacobian << "\n\n";

        // std::cout << loop_count << ":\t" << jacobian.inverse() << "\n\n";

        Eigen::Vector3d delta_q = jacobian.lu().solve(error_pos);
        init_joint_pos += delta_q;

        foot_pos = getFootPosInGravityFrame(leg_id, init_joint_pos, body_euler_angle);
        error_pos = desired_foot_pos - foot_pos;
        loop_count++;
    }

    return init_joint_pos;
}
