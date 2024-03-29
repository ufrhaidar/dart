/*
 * Copyright (c) 2011-2024, The DART development contributors
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

#include "dart/dynamics/RigidBody.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/SphereShape.hpp"

namespace dart::dynamics {

std::shared_ptr<RigidBody> RigidBody::create(const Config& config)
{
  return std::make_shared<RigidBody>(config);
}

RigidBody::RigidBody(const Config& config)
{
  mSkeleton = Skeleton::create(config.name);
  DART_ASSERT(mSkeleton);

  std::tie(mParentJoint, mBodyNode)
      = mSkeleton->createJointAndBodyNodePair<FreeJoint>();
  DART_ASSERT(mParentJoint);
  DART_ASSERT(mBodyNode);

  // TODO: Add shape factory by the string type
  if (config.shapeType == BoxShape::getStaticType()) {
    mShape
        = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5));
  } else {
    mShape = std::make_shared<dynamics::SphereShape>(0.5);
  }
  DART_ASSERT(mShape);

  mShapeNode = mBodyNode->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(mShape);
  DART_ASSERT(mShapeNode);
  mShapeNode->setShape(nullptr);
}

RigidBody::~RigidBody()
{
  // Empty
}

} // namespace dart::dynamics
