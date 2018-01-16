//
// Created by hannes on 2017-12-07.
//

#ifndef TASERV2_TYPE_HELPERS_H
#define TASERV2_TYPE_HELPERS_H

#include <boost/hana.hpp>
namespace hana = boost::hana;

#include "trajectories/linear_trajectory.h"
#include "trajectories/simple_multi_trajectory.h"
#include "trajectories/uniform_r3_spline_trajectory.h"
//
#include "cameras/pinhole.h"
#include "cameras/atan.h"

#include "measurements/static_rscamera_measurement.h"
#include "measurements/position_measurement.h"

// Add template-of-template support to hana
// Thanks to Jason Rice!
#if 0
template <template <template <typename...> class> class F>
struct template_template_t
{
  template <template <typename...> class G>
  constexpr auto operator()(hana::basic_type<hana::template_t<G>>) const
  -> hana::type<F<G>>
  { return {}; }
};

template <template <template <typename> class> class F>
constexpr auto template_template = template_template_t<F>{};
#else
// non-ellipsis versions which actually work
template <template <template <typename> class> class F>
struct template_template_t
{
  template <template <typename> class G>
  constexpr auto operator()(hana::basic_type<hana::template_t<G>>) const
  -> hana::type<F<G>>
  { return {}; }
};

template <template <template <typename> class> class F>
constexpr auto template_template = template_template_t<F>{};
#endif

// Begin actual
namespace TT = taser::trajectories;
namespace TM = taser::measurements;
namespace TC = taser::cameras;

static constexpr auto trajectory_types = hana::tuple_t<
    TT::SimpleMultiTrajectory,
    TT::LinearTrajectory,
    TT::UniformR3SplineTrajectory
>;

// Define valid camera types
static constexpr auto camera_types = hana::tuple_t<
  TC::AtanCamera,
  TC::PinholeCamera
>;

// Define Camera measurements
static auto cam_meas_templates = hana::tuple_t<
    hana::template_t<TM::StaticRsCameraMeasurement>
>;


static auto make_cam_meas = [](auto mtype, auto ctype) {
  using MeasType = typename decltype(mtype)::type;
  auto mclass = MeasType();
  return mclass(ctype);
};

static auto camera_measurements = hana::ap(
    hana::make_tuple(make_cam_meas),
    cam_meas_templates,
    camera_types
    );

// Final list of measurement types
static auto measurement_types = hana::concat(
  camera_measurements,
  hana::tuple_t<TM::PositionMeasurement>
);

#endif //TASERV2_TYPE_HELPERS_H
