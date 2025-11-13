#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

class CustomHeadingFactorArm : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
    gtsam::Rot3 measured_rot_sensor_; // $R_{ws}$ (Measured world-to-sensor rotation)
    gtsam::Rot3 R_base_sensor_;       // $R_{sb}$ (Body-to-sensor rotation)

public:
    CustomHeadingFactorArm(gtsam::Key poseKey, const gtsam::Rot3 &measured_rot_sensor, const gtsam::Rot3 &R_base_sensor, const gtsam::SharedNoiseModel &model)
        : NoiseModelFactor1<gtsam::Pose3>(model, poseKey), measured_rot_sensor_(measured_rot_sensor), R_base_sensor_(R_base_sensor) {}

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose, // $T_{wb}$
                                boost::optional<gtsam::Matrix &> H = boost::none) const override
    {
        // $R_{wb}$
        const gtsam::Rot3 &pose_rot = pose.rotation();

        // $R_{m,b} = R_{ws} \cdot R_{sb}^{-1}$
        gtsam::Rot3 measured_rot_body = measured_rot_sensor_ * R_base_sensor_.inverse();

        // $R_e = R_{m,b}^{-1} \cdot R_{wb}$
        gtsam::Rot3 error_rot = measured_rot_body.inverse() * pose_rot;

        // $e = \log(R_e)$
        gtsam::Vector3 error = gtsam::Rot3::Logmap(error_rot);

        if (H)
        {
            // Jacobian $H = \frac{\partial e}{\partial \mathbf{x}}$ (3x6)
            // $\frac{\partial e}{\partial \boldsymbol{\xi}_{wb}} = \text{dexp}^{-1}_{e}$
            gtsam::Matrix3 H_rot = gtsam::Rot3::LogmapDerivative(error);
            // $\frac{\partial e}{\partial p_{wb}} = \mathbf{0}_{3x3}$
            gtsam::Matrix3 H_trans = gtsam::Matrix3::Zero();
            *H = (gtsam::Matrix(3, 6) << H_rot, H_trans).finished();
        }

        return error;
    }
};