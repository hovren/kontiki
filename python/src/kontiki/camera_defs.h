//
// Created by hannes on 2018-02-05.
//

#ifndef KONTIKIV2_CAMERA_DEFS_H
#define KONTIKIV2_CAMERA_DEFS_H

#include <boost/hana.hpp>
namespace hana = boost::hana;

#include "kontiki/sensors/pinhole_camera.h"
#include "kontiki/sensors/atan_camera.h"

namespace TC = kontiki::sensors;

// Define valid camera types
static constexpr auto camera_types = hana::tuple_t<
    TC::AtanCamera,
    TC::PinholeCamera
>;


#endif //KONTIKIV2_CAMERA_DEFS_H
