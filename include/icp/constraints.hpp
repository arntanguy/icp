//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#ifndef ICP_CONSTRAINTS_HPP
#define ICP_CONSTRAINTS_HPP

#include <Eigen/Core>
#include "eigentools.hpp"
#include "logging.hpp"

#define DEFINE_CONSTRAINT_TYPES(Scalar, DegreesOfFreedom, suffix) \
    typedef Constraints<Scalar, DegreesOfFreedom> Constraints##DegreesOfFreedom##suffix;
namespace icp
{

class FixTranslationConstraint
{
  protected:
    std::array<bool, 3> fixedAxes_;
  public:
    FixTranslationConstraint() : fixedAxes_({{false, false, false}}) {
    }

    FixTranslationConstraint(bool x, bool y, bool z) : fixedAxes_({x, y, z})
    {
    }

    int numFixedAxes() const;
    std::array<bool, 3> getFixedAxes() const {
      return fixedAxes_;
    }
};

template <typename Scalar, unsigned int DegreesOfFreedom>
class Constraints
{
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> JacobianMatrix;
    typedef typename Eigen::Matrix<Scalar, DegreesOfFreedom, 1> Twist;

  protected:
    FixTranslationConstraint translationConstraint_;


  public:
    Constraints () {
    }

    void setTranslationConstraint(const FixTranslationConstraint &translationConstraint) {
      translationConstraint_ = translationConstraint;
    }

    FixTranslationConstraint getTranslationConstraint() const {
      return translationConstraint_;
    }

    bool hasConstraints() const {
      return translationConstraint_.numFixedAxes() != 0;
    }

    void processJacobian(JacobianMatrix &J, JacobianMatrix& Jconstrained) {
      int numFixed = translationConstraint_.numFixedAxes();
      Jconstrained.resize(J.rows(), J.cols()-numFixed);
      unsigned int i = 0;
      for(unsigned int j = 0; j < J.cols(); j++) {
        if(j < 3) {
          if(!translationConstraint_.getFixedAxes()[j]) {
            Jconstrained.col(i) = J.col(j);
            i++;
          }
        } else {
            Jconstrained.col(i) = J.col(j);
            i++;
        }
      }
    }

    Twist getTwist(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &twist) {
      /*
       * Recreate from the missing parts of the lie algebra
       * Adds a zero translation to each fixed axis in the lie algebra
      */
      LOG(INFO) << "Get twist";
      Twist xc;
      int i = 0;
      int j = 0;
      int numfixed = 0;
      for (bool axis : translationConstraint_.getFixedAxes()) {
        if (axis) {
          xc(i) = 0; 
          numfixed++;
        } else {
          xc(i) = twist(j);
          j++;
        }
        ++i;
      }
      LOG(INFO) << "numfixed: " << numfixed;
      for (; j < twist.rows(); ++j)
      {
        LOG(INFO) << "xc(" << numfixed+j << ")";
        xc(numfixed + j) = twist(j);
      }
      return xc;
    }
};

DEFINE_CONSTRAINT_TYPES(float, 6, );
DEFINE_CONSTRAINT_TYPES(float, 6, f);
DEFINE_CONSTRAINT_TYPES(float, 7, );
DEFINE_CONSTRAINT_TYPES(float, 7, f);


}  // namespace icp

#endif
