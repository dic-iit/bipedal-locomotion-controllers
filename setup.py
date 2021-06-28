# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import sys

from cmake_build_extension import BuildExtension, CMakeExtension
from setuptools import setup

if "CIBUILDWHEEL" in os.environ and os.environ["CIBUILDWHEEL"] == "1":
    CIBW_CMAKE_OPTIONS = ["-DCMAKE_INSTALL_LIBDIR=lib"]
else:
    CIBW_CMAKE_OPTIONS = []


setup(
    cmdclass=dict(build_ext=BuildExtension),
    ext_modules=[
        CMakeExtension(
            name="BlfCMakeProject",
            install_prefix="bipedal_locomotion_framework",
            cmake_depends_on=["idyntree", "pybind11", "yarp", "manifpy", "casadi"],
            disable_editable=True,
            cmake_configure_options=[
                f"-DPython3_EXECUTABLE:PATH={sys.executable}",
                "-DFRAMEWORK_PACKAGE_FOR_PYPI:BOOL=ON",
                "-DFRAMEWORK_USE_LieGroupControllers:BOOL=OFF",
                "-DFRAMEWORK_USE_OpenCV:BOOL=OFF",
                "-DFRAMEWORK_USE_OsqpEigen:BOOL=ON",
                "-DFRAMEWORK_USE_PCL:BOOL=OFF",
                "-DFRAMEWORK_USE_Python3:BOOL=ON",
                "-DFRAMEWORK_USE_Qhull:BOOL=ON",
                "-DFRAMEWORK_USE_VALGRIND:BOOL=OFF",
                "-DFRAMEWORK_USE_YARP:BOOL=ON",
                "-DFRAMEWORK_USE_casadi:BOOL=ON",
                "-DFRAMEWORK_USE_cppad:BOOL=OFF",
                "-DFRAMEWORK_USE_manif:BOOL=ON",
                "-DFRAMEWORK_USE_matioCpp:BOOL=OFF",
                "-DFRAMEWORK_USE_pybind11:BOOL=ON",
                "-DFRAMEWORK_USE_realsense2:BOOL=OFF",
                "-DFRAMEWORK_COMPILE_System:BOOL=ON",
                "-DFRAMEWORK_COMPILE_Planners:BOOL=ON",
                "-DFRAMEWORK_COMPILE_RobotInterface:BOOL=ON",
                "-DFRAMEWORK_COMPILE_YarpUtilities:BOOL=ON",
                "-DFRAMEWORK_COMPILE_YarpImplementation:BOOL=ON",
                "-DFRAMEWORK_COMPILE_PYTHON_BINDINGS:BOOL=ON",
            ]
            + CIBW_CMAKE_OPTIONS,
        )
    ],
)
