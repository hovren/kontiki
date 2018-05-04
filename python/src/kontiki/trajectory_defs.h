//
// Created by hannes on 2018-02-05.
//

#ifndef KONTIKIV2_TRAJECTORY_DEFS_H
#define KONTIKIV2_TRAJECTORY_DEFS_H

#include <kontiki/trajectories/uniform_r3_spline_trajectory.h>
#include <kontiki/trajectories/uniform_so3_spline_trajectory.h>
#include <kontiki/trajectories/uniform_se3_spline_trajectory.h>
#include <kontiki/trajectories/split_trajectory.h>

namespace TT = kontiki::trajectories;

static constexpr auto trajectory_types = hana::tuple_t<
    TT::UniformR3SplineTrajectory,
    TT::UniformSO3SplineTrajectory,
    TT::UniformSE3SplineTrajectory,
    TT::SplitTrajectory
>;

#endif //KONTIKIV2_TRAJECTORY_DEFS_H
