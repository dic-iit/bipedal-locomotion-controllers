# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_TSID)

  set(H_PREFIX include/BipedalLocomotion/TSID)

  add_bipedal_locomotion_library(
    NAME                  TSID
    PUBLIC_HEADERS        ${H_PREFIX}/Task.h ${H_PREFIX}/SE3Task.h
    SOURCES               src/Task.cpp src/SE3Task.cpp
    PUBLIC_LINK_LIBRARIES Eigen3::Eigen BipedalLocomotion::ParametersHandler BipedalLocomotion::System LieGroupControllers::LieGroupControllers MANIF::manif iDynTree::idyntree-high-level iDynTree::idyntree-model
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::ManifConversions
    SUBDIRECTORIES         tests)

endif()