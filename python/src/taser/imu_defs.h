//
// Created by hannes on 2018-02-05.
//

#ifndef TASERV2_IMU_DEFS_H
#define TASERV2_IMU_DEFS_H

#include <boost/hana.hpp>
namespace hana = boost::hana;

#include <taser/sensors/basic_imu.h>
#include <taser/sensors/constant_bias_imu.h>

namespace TS = taser::sensors;

// Define IMU types
static constexpr auto imu_types = hana::tuple_t<
    TS::ConstantBiasImu,
    TS::BasicImu
>;

#endif //TASERV2_IMU_DEFS_H
