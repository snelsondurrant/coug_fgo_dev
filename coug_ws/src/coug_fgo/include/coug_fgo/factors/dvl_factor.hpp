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

class CustomDVLFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>
{
    gtsam::Vector3 measured_velocity_body_; // $v_{m,b}$ (Measured velocity in body frame)

public:
    CustomDVLFactor(gtsam::Key poseKey, gtsam::Key velKey,
              const gtsam::Vector3 &measured_velocity_body,
              const gtsam::SharedNoiseModel &model)
        : NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>(model, poseKey, velKey),
          measured_velocity_body_(measured_velocity_body) {}

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,                 // $T_{wb}$
                                const gtsam::Vector3 &vel_world,          // $v_w$
                                boost::optional<gtsam::Matrix &> H_pose = boost::none,
                                boost::optional<gtsam::Matrix &> H_vel = boost::none) const override
    {
        // $R_{bw} = R_{wb}^T$
        gtsam::Rot3 R_bw = pose.rotation().inverse();

        // $v_b = R_{bw} v_w$
        gtsam::Vector3 predicted_vel_body = R_bw.rotate(vel_world);

        // $e = v_b - v_{m,b}$
        gtsam::Vector3 error = predicted_vel_body - measured_velocity_body_;

        if (H_pose)
        {
            // $\frac{\partial e}{\partial \mathbf{x}}$ (3x6)
            // $\frac{\partial e}{\partial \boldsymbol{\theta}_{wb}} = [R_{bw} v_w]_{\times}$
            gtsam::Matrix3 H_rot = gtsam::skewSymmetric(predicted_vel_body);
            // $\frac{\partial e}{\partial p_{wb}} = \mathbf{0}_{3x3}$
            gtsam::Matrix3 H_trans = gtsam::Matrix3::Zero();
            *H_pose = (gtsam::Matrix(3, 6) << H_rot, H_trans).finished();
        }

        if (H_vel)
        {
            // $\frac{\partial e}{\partial v_w} = R_{bw}$
            *H_vel = R_bw.matrix();
        }

        return error;
    }
};