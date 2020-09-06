# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_YarpImplementation)

  add_bipedal_locomotion_library(
    NAME                   RobotInterfaceYarpImplementation
    SOURCES                src/YarpImplementation.cpp
    PUBLIC_HEADERS         include/BipedalLocomotion/RobotInterface/YarpImplementation.h 
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::RobotInterface YARP::YARP_dev
    INSTALLATION_FOLDER ParametersHandler)

endif()