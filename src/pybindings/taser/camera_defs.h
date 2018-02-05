//
// Created by hannes on 2018-02-05.
//

#ifndef TASERV2_CAMERA_DEFS_H
#define TASERV2_CAMERA_DEFS_H

#include <boost/hana.hpp>
namespace hana = boost::hana;

#include "cameras/pinhole.h"
#include "cameras/atan.h"

namespace TC = taser::cameras;

// Define valid camera types
static constexpr auto camera_types = hana::tuple_t<
    TC::AtanCamera,
    TC::PinholeCamera
>;


#endif //TASERV2_CAMERA_DEFS_H
