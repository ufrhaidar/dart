/*
 * Copyright (c) The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <dart/config.hpp>

#include <memory>

// Define a typedef for const and non-const version of shared_ptr and weak_ptr
// for the class X
#define DART_DECL_CLASS_PTRS(X)                                                \
  class X;                                                                     \
  using X##Ptr = std::shared_ptr<X>;                                           \
  using Const##X##Ptr = std::shared_ptr<const X>;                              \
  using X##WeakPtr = std::weak_ptr<X>;                                         \
  using Const##X##WeakPtr = std::weak_ptr<const X>;                            \
  using X##UniquePtr = std::unique_ptr<X>;                                     \
  using Const##X##UniquePtr = std::unique_ptr<const X>;

namespace dart::v7 {

DART_DECL_CLASS_PTRS(Resource)
DART_DECL_CLASS_PTRS(LocalResource)
DART_DECL_CLASS_PTRS(ResourceLoader)

struct StepInfo;

class Context;
class World;

class RigidBody;

} // namespace dart::v7
