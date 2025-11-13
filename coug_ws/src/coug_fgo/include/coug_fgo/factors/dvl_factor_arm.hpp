#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Lie.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>

using gtsam::symbol_shorthand::B; // Bias (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V; // Velocity (x,y,z)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

class CustomDVLFactorArm : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>
{
    gtsam::Vector3 measured_velocity_sensor_; // $v_{m,s}$ (Measured velocity in sensor frame)
    gtsam::Vector3 measured_gyro_;            // $\omega_m$ (Measured gyro in body frame)
    gtsam::Pose3 T_base_sensor_;              // $T_{bs}$ (Sensor pose in base frame)

public:
    CustomDVLFactorArm(gtsam::Key poseKey, gtsam::Key velKey, gtsam::Key biasKey,
              const gtsam::Vector3 &measured_velocity_sensor,
              const gtsam::Vector3 &measured_gyro,
              const gtsam::Pose3 &T_base_sensor, const gtsam::SharedNoiseModel &model)
        : NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>(model, poseKey, velKey, biasKey),
          measured_velocity_sensor_(measured_velocity_sensor),
          measured_gyro_(measured_gyro),
          T_base_sensor_(T_base_sensor) {}

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,                 // $T_{wb}$
                                const gtsam::Vector3 &vel_world,          // $v_w$
                                const gtsam::imuBias::ConstantBias &bias, // $b$
                                boost::optional<gtsam::Matrix &> H_pose = boost::none,
                                boost::optional<gtsam::Matrix &> H_vel = boost::none,
                                boost::optional<gtsam::Matrix &> H_bias = boost::none) const override
    {
        // $R_{bw} = R_{wb}^T$
        gtsam::Rot3 R_bw = pose.rotation().inverse();
        // $p_{bs}$
        gtsam::Vector3 p_bs = T_base_sensor_.translation();

        // $v_{b,origin} = R_{bw} v_w$
        gtsam::Vector3 predicted_vel_at_origin = R_bw.rotate(vel_world);

        // $\omega_b = \omega_m - b_g$
        gtsam::Vector3 w_body = measured_gyro_ - bias.gyroscope();

        // $v_{rot} = \omega_b \times p_{bs}$
        gtsam::Vector3 rotational_velocity = w_body.cross(p_bs);

        // $v_{b,sensor} = v_{b,origin} + v_{rot}$
        gtsam::Vector3 predicted_velocity_at_sensor = predicted_vel_at_origin + rotational_velocity;

        // $v_{m,b} = R_{bs} v_{m,s}$
        gtsam::Vector3 measured_velocity_body = T_base_sensor_.rotation().rotate(measured_velocity_sensor_);

        // $e = v_{b,sensor} - v_{m,b}$
        gtsam::Vector3 error = predicted_velocity_at_sensor - measured_velocity_body;

        if (H_pose)
        {
            // $\frac{\partial e}{\partial \mathbf{x}}$ (3x6)
            // $\frac{\partial e}{\partial \boldsymbol{\theta}_{wb}} = [v_{b,origin}]_{\times} = [R_{bw} v_w]_{\times}$
            gtsam::Matrix3 H_rot = gtsam::skewSymmetric(predicted_vel_at_origin);
            // $\frac{\partial e}{\partial p_{wb}} = \mathbf{0}_{3x3}$
            gtsam::Matrix3 H_trans = gtsam::Matrix3::Zero();
            *H_pose = (gtsam::Matrix(3, 6) << H_rot, H_trans).finished();
        }

        if (H_vel)
        {
            // $\frac{\partial e}{\partial v_w} = R_{bw}$
            *H_vel = R_bw.matrix();
        }

        if (H_bias)
        {
            // $\frac{\partial e}{\partial b}$ (3x6)
            // $\frac{\partial e}{\partial b_a} = \mathbf{0}_{3x3}$
            gtsam::Matrix3 H_bias_accel = gtsam::Matrix3::Zero();
            // $\frac{\partial e}{\partial b_g} = [p_{bs}]_{\times}$
            gtsam::Matrix3 H_bias_gyro = gtsam::skewSymmetric(p_bs);
            *H_bias = (gtsam::Matrix(3, 6) << H_bias_accel, H_bias_gyro).finished();
        }

        return error;
    }
};