//
// Created by hannes on 2018-02-05.
//

#ifndef TASERV2_TRAJECTORY_DEFS_H
#define TASERV2_TRAJECTORY_DEFS_H

#include "trajectories/linear_trajectory.h"
#include "trajectories/uniform_r3_spline_trajectory.h"
#include "trajectories/uniform_so3_spline_trajectory.h"
#include "trajectories/uniform_se3_spline_trajectory.h"
#include "trajectories/split_trajectory.h"

namespace TT = taser::trajectories;

static constexpr auto trajectory_types = hana::tuple_t<
    TT::LinearTrajectory,
    TT::UniformR3SplineTrajectory,
    TT::UniformSO3SplineTrajectory,
    TT::UniformSE3SplineTrajectory,
    TT::SplitTrajectory
>;

#endif //TASERV2_TRAJECTORY_DEFS_H
