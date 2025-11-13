#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

class CustomGPSFactorArm : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
    gtsam::Point3 measured_position_; // $p_m$ (Measured position in world frame)
    gtsam::Pose3 T_base_sensor_;      // $T_{bs}$ (Sensor pose in base frame)

public:
    CustomGPSFactorArm(gtsam::Key poseKey, const gtsam::Point3 &measured_position, const gtsam::Pose3 &T_base_sensor, const gtsam::SharedNoiseModel &model)
        : NoiseModelFactor1<gtsam::Pose3>(model, poseKey), measured_position_(measured_position), T_base_sensor_(T_base_sensor) {}

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose, // $T_{wb}$
                                boost::optional<gtsam::Matrix &> H = boost::none) const override
    {
        // $T_{ws} = T_{wb} \cdot T_{bs}$
        gtsam::Pose3 T_ws = pose.compose(T_base_sensor_);

        // $p_{ws}$
        gtsam::Point3 predicted_position = T_ws.translation();

        // $e = p_{ws} - p_m$
        gtsam::Vector3 error = predicted_position - measured_position_;

        if (H)
        {
            // Jacobian $H = \frac{\partial e}{\partial \mathbf{x}}$ (3x6)
            gtsam::Matrix H_matrix = gtsam::Matrix::Zero(3, 6);

            gtsam::Matrix R_wb = pose.rotation().matrix();        // $R_{wb}$
            gtsam::Vector3 p_bs = T_base_sensor_.translation();   // $p_{bs}$
            gtsam::Matrix p_bs_skew = gtsam::skewSymmetric(p_bs); // $[p_{bs}]_{\times}$

            // $\frac{\partial e}{\partial \boldsymbol{\theta}_{wb}} = -R_{wb} [p_{bs}]_{\times}$
            H_matrix.block<3, 3>(0, 0) = -R_wb * p_bs_skew;
            // $\frac{\partial e}{\partial p_{wv}} = R_{wv}$
            H_matrix.block<3, 3>(0, 3) = R_wb;

            *H = H_matrix;
        }

        return error;
    }
};