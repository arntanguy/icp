//  This file is part of the Icp Library,
//
//  Copyright (C) 2014-2015 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#include <algorithm>
#include "constraints.hpp"
#include "instanciate.hpp"
#include "logging.hpp"


namespace icp
{

int FixTranslationConstraint::numFixedAxes() const
{
  return std::count(std::begin(fixedAxes_), std::end(fixedAxes_), true);
}

template<typename Scalar, unsigned int DegreesOfFreedom>
void JacobianConstraints<Scalar, DegreesOfFreedom>::processJacobian(const JacobianMatrix &J, JacobianMatrix &Jconstrained) {
  LOG(INFO) << translationConstraint_.getFixedAxes()[0] << ", " << translationConstraint_.getFixedAxes()[1] << ", " << translationConstraint_.getFixedAxes()[2];
            Eigen::Matrix<Scalar, 3, DegreesOfFreedom> weights;
  weights.setOnes();
  int i = 0;
  for (bool axis : this->translationConstraint_.getFixedAxes()) {
    if (axis) {
      weights(i, i) = 0;
    } else {
      weights(i, i) = 1;
    }
    ++i;
  }
  LOG(INFO) << "W: " << weights;
  Jconstrained = J;
  for (i = 0; i < J.rows() / 3; i++) {
    Jconstrained.block(i, 0, 3, DegreesOfFreedom) = weights.array() * Jconstrained.block(i, 0, 3, DegreesOfFreedom).array();
  }
}

INSTANCIATE_CONSTRAINTS;

}  // namespace icp
