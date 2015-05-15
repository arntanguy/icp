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
#include <boost/array.hpp>
#include "eigentools.hpp"
#include "logging.hpp"

#define DEFINE_CONSTRAINT_TYPES(Scalar, DegreesOfFreedom, suffix) \
    typedef JacobianConstraints<Scalar, DegreesOfFreedom> Constraints##DegreesOfFreedom##suffix;

namespace icp
{

class FixTranslationConstraint
{
  protected:
   typedef boost::array<bool, 3> FixedAxes;
   FixedAxes fixedAxes_;

  public:
    FixTranslationConstraint() {
      setFixedAxes(false, false, false);
    }

    FixTranslationConstraint(bool x, bool y, bool z)
    {
      setFixedAxes(x, y, z);
    }

    void setFixedAxes(bool x, bool y, bool z) {
      fixedAxes_[0] = x;
      fixedAxes_[1] = y;
      fixedAxes_[2] = z;
    }

    int numFixedAxes() const;
    FixedAxes getFixedAxes() const {
      return fixedAxes_;
    }
};

template <typename Scalar, unsigned int DegreesOfFreedom>
class Constraints_
{
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> JacobianMatrix;
    typedef typename Eigen::Matrix<Scalar, DegreesOfFreedom, 1> Twist;

  protected:
    FixTranslationConstraint translationConstraint_;


  public:
    Constraints_ () {
    }

    virtual void setTranslationConstraint(const FixTranslationConstraint &translationConstraint) {
      LOG(INFO) << translationConstraint.getFixedAxes()[0] << ", " << translationConstraint.getFixedAxes()[1] << ", " <<
                translationConstraint.getFixedAxes()[2];
      translationConstraint_ = translationConstraint;
      LOG(INFO) << translationConstraint_.getFixedAxes()[0] << ", " << translationConstraint_.getFixedAxes()[1] << ", " <<
                translationConstraint_.getFixedAxes()[2];
    }

    FixTranslationConstraint getTranslationConstraint() const {
      return translationConstraint_;
    }

    bool hasConstraints() const {
      return translationConstraint_.numFixedAxes() != 0;
    }

    virtual void processJacobian(const JacobianMatrix &J, JacobianMatrix &Jconstrained) {
      Jconstrained = J;
    }


    virtual Twist getTwist(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &twist) {
      return twist;
    }
};

template <typename Scalar, unsigned int DegreesOfFreedom>
class JacobianConstraints : public Constraints_<Scalar, DegreesOfFreedom>
{
  protected:
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> JacobianMatrix;
    typedef typename Eigen::Matrix<Scalar, DegreesOfFreedom, 1> Twist;
    using Constraints_<Scalar, DegreesOfFreedom>::translationConstraint_;

  public:
    void processJacobian(const JacobianMatrix &J, JacobianMatrix &Jconstrained);
};

DEFINE_CONSTRAINT_TYPES(float, 6, );
DEFINE_CONSTRAINT_TYPES(float, 6, f);
DEFINE_CONSTRAINT_TYPES(float, 7, );
DEFINE_CONSTRAINT_TYPES(float, 7, f);


}  // namespace icp

#endif
