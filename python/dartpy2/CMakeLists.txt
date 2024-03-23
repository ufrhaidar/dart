# Copyright (c) 2011-2024, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
nanobind_add_module(dartpy2 dartpy2.cpp)
