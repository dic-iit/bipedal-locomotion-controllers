# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_Planners)

  set(H_PREFIX include/BipedalLocomotion/bindings/Planners)

  add_bipedal_locomotion_python_module(
    NAME PlannersBindings
    SOURCES src/DCMPlanner.cpp src/TimeVaryingDCMPlanner.cpp src/QuinticSpline.cpp src/SwingFootPlanner.cpp src/Module.cpp
    HEADERS ${H_PREFIX}/DCMPlanner.h ${H_PREFIX}/TimeVaryingDCMPlanner.h ${H_PREFIX}/QuinticSpline.h ${H_PREFIX}/SwingFootPlanner.h ${H_PREFIX}/Module.h
    LINK_LIBRARIES BipedalLocomotion::Planners
    TESTS tests/test_quintic_spline.py tests/test_swing_foot_planner.py tests/test_time_verying_dcm_planner.py
    )

endif()
