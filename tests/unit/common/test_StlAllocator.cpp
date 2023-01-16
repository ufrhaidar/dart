/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/common/common.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace common;

//==============================================================================
GTEST_TEST(StdAllocatorTest, Basics)
{
  auto a = StdAllocator<int>();
  auto o1 = a.allocate(1);
  auto o2 = a.allocate(1);
  EXPECT_TRUE(o1 != nullptr);
  EXPECT_TRUE(o2 != nullptr);
  a.deallocate(o1, 1);
  a.deallocate(o2, 1);
  a.print();
}

//==============================================================================
// GTEST_TEST(StdAllocatorTest, Basics2)
//{
//  AllocatorLinear base_allocator(sizeof(int) + 1);
//  static_assert(
//      sizeof(int) > 1,
//      "sizeof(int) should be greater than 1 to keep this test valid.");
//  auto a = StdAllocator<int>(base_allocator);
//  EXPECT_TRUE(a.allocate(1) != nullptr);
//  try {
//    EXPECT_TRUE(a.allocate(1) == nullptr);
//  } catch (std::bad_alloc& /*e*/) {
//    EXPECT_TRUE(true);
//  } catch (...) {
//    EXPECT_TRUE(false);
//  }
//  a.print();
//}
