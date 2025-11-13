#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

class CustomDepthFactorArm : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
    double measured_depth_;      // $z_m$ (Measured Z in world frame)
    gtsam::Pose3 T_base_sensor_; // $T_{bs}$ (Sensor pose in base frame)

public:
    CustomDepthFactorArm(gtsam::Key poseKey, double measured_depth, const gtsam::Pose3 &T_base_sensor, const gtsam::SharedNoiseModel &model)
        : NoiseModelFactor1<gtsam::Pose3>(model, poseKey), measured_depth_(measured_depth), T_base_sensor_(T_base_sensor) {}

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose, // $T_{wb}$
                                boost::optional<gtsam::Matrix &> H = boost::none) const override
    {
        // $T_{ws} = T_{wb} \cdot T_{bs}$
        gtsam::Pose3 T_ws = pose.compose(T_base_sensor_);

        double world_z_sensor = T_ws.z(); // $z_{ws}$

        // $e = z_{ws} - z_m$
        double error = world_z_sensor - measured_depth_;

        if (H)
        {
            // Jacobian $H = \frac{\partial e}{\partial \mathbf{x}}$ (1x6)
            gtsam::Matrix H_matrix = gtsam::Matrix::Zero(1, 6);

            gtsam::Matrix R_wb = pose.rotation().matrix();        // $R_{wb}$
            gtsam::Vector3 p_bs = T_base_sensor_.translation();   // $p_{bs}$
            gtsam::Matrix p_bs_skew = gtsam::skewSymmetric(p_bs); // $[p_{bs}]_{\times}$
            gtsam::Matrix R_row2 = R_wb.row(2);                   // Z-axis row of $R_{wb}$

            // $\frac{\partial e}{\partial \boldsymbol{\theta}_{wb}} = -R_{wb}(2,:) [p_{bs}]_{\times}$
            H_matrix.block<1, 3>(0, 0) = -R_row2 * p_bs_skew;
            // $\frac{\partial e}{\partial p_{wb}} = R_{wb}(2,:)$
            H_matrix.block<1, 3>(0, 3) = R_row2;

            *H = H_matrix;
        }

        return gtsam::Vector1(error);
    }
};