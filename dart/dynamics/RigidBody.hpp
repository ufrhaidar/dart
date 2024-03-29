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

#pragma once

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>

namespace dart::dynamics {

struct RigidBodyConfig
{
  std::string name = "RigidBody";
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  std::string shapeType = BoxShape::getStaticType();

  RigidBodyConfig(const std::string& name = "RigidBody") : name(name)
  {
    // Empty
  }
};

class RigidBody
{
public:
  using Config = RigidBodyConfig;

  static std::shared_ptr<RigidBody> create(const Config& config = Config());

  /// Constructor
  explicit RigidBody(const Config& config = Config());

  /// Destructor
  virtual ~RigidBody();

  template <typename Derived>
  void setPosition(const Eigen::MatrixBase<Derived>& position)
  {
    auto tf = mBodyNode->getTransform();
    tf.translation() = position;
    FreeJoint::setTransformOf(mParentJoint, tf);
  }

  [[nodiscard]] Eigen::Vector3d getPosition() const
  {
    return mBodyNode->getTransform().translation();
  }

  template <typename Derived>
  void setOrientation(const Eigen::MatrixBase<Derived>& rotationMap)
  {
    auto tf = mBodyNode->getTransform();
    tf.linear() = rotationMap;
    FreeJoint::setTransformOf(mParentJoint, tf);
  }

  [[nodiscard]] Eigen::Matrix3d getOrientation() const
  {
    return mBodyNode->getTransform().linear();
  }

  void setInertia(const Eigen::Matrix3d& inertia)
  {
    const auto& inertial = mBodyNode->getInertia();
    mBodyNode->setInertia(
        Inertia(inertial.getMass(), inertial.getLocalCOM(), inertia));
  }

  ConstShapePtr getShape() const
  {
    return mShape;
  }

  ShapePtr getShape()
  {
    return mShape;
  }

  void setColor(const Eigen::Vector3d& rgb)
  {
    mShapeNode->getVisualAspect()->setColor(rgb);
  }

  SkeletonPtr getSkeleton()
  {
    return mSkeleton;
  }

  ConstSkeletonPtr getSkeleton() const
  {
    return mSkeleton;
  }

  void setStatic(bool value = true) {
    if (value) {
      mParentJoint->setActuatorType(Joint::LOCKED);
    } else {
      mParentJoint->setActuatorType(Joint::FORCE);
    }
  }

  [[nodiscard]] bool isStatic() const {
    return mParentJoint->isKinematic();
  }

private:
  SkeletonPtr mSkeleton;
  BodyNode* mBodyNode;
  FreeJoint* mParentJoint;
  ShapeNodePtr mShapeNode;
  ShapePtr mShape;
};

} // namespace dart::dynamics
