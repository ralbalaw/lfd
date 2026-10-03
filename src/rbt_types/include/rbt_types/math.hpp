#pragma once

/**
 * @file math.hpp
 * @brief Shared Eigen aliases used across `rbt_*` packages.
 */

#include <Eigen/Core>

namespace rbt_types
{

/// @brief Dynamic-size column vector (`Eigen::VectorXd`).
using Vec = Eigen::VectorXd;

/// @brief Dynamic-size row vector (`Eigen::RowVectorXd`).
using RowVec = Eigen::RowVectorXd;

/// @brief Dynamic-size matrix (`Eigen::MatrixXd`).
using Mat = Eigen::MatrixXd;

/// @brief Waypoint times from trajectory start [s], size = num_waypoints.
using TimeVec = Vec;

/// @brief Joint-position vector [rad], size = dof.
using JointVec = Vec;

/// @brief Read-only view of a joint-position vector [rad], size = dof.
using JointVecView = Eigen::Ref<const JointVec>;

/// @brief Row vector of joint positions [rad], size = dof.
using JointRowVec = RowVec;

/// @brief View of a row vector of joint positions, size = dof.
using JointRowVecView = Eigen::Ref<const JointRowVec>;

/// @brief Reward/IRL parameter vector, size = num_features.
using WeightVec = Vec;

/// @brief Feature-count/value vector, size = num_features.
using FeatureVec = Vec;

/// @brief Row-major joint-path matrix [rad], shape = (num_waypoints, dof).
using JointPathMat =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

/// @brief Read-only view of a row-major joint-path matrix [rad].
using JointPathView = Eigen::Ref<const JointPathMat>;

}  // namespace rbt_types
