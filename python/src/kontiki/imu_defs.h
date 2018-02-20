//
// Created by hannes on 2018-02-05.
//

#ifndef KONTIKIV2_IMU_DEFS_H
#define KONTIKIV2_IMU_DEFS_H

#include <boost/hana.hpp>
namespace hana = boost::hana;

#include <kontiki/sensors/basic_imu.h>
#include <kontiki/sensors/constant_bias_imu.h>

namespace TS = kontiki::sensors;

// Define IMU types
static constexpr auto imu_types = hana::tuple_t<
    TS::ConstantBiasImu,
    TS::BasicImu
>;

#endif //KONTIKIV2_IMU_DEFS_H
