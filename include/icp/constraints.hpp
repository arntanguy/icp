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
#include "linear_algebra.hpp"

#define DEFINE_CONSTRAINT_TYPES(Scalar, DegreesOfFreedom, suffix) \
    typedef JacobianConstraints<Scalar, DegreesOfFreedom> Constraints##DegreesOfFreedom##suffix;

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

    void setFixedAxes(bool x, bool y, bool z) {
      fixedAxes_ = {x, y, z};
    }

    int numFixedAxes() const;
    std::array<bool, 3> getFixedAxes() const {
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
      LOG(INFO) << translationConstraint.getFixedAxes()[0] << ", " << translationConstraint.getFixedAxes()[1] << ", " << translationConstraint.getFixedAxes()[2];
      translationConstraint_ = translationConstraint;
      LOG(INFO) << translationConstraint_.getFixedAxes()[0] << ", " << translationConstraint_.getFixedAxes()[1] << ", " << translationConstraint_.getFixedAxes()[2];
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
class HardConstraints : public Constraints_<Scalar, DegreesOfFreedom>
{
  protected:
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> JacobianMatrix;
    typedef typename Eigen::Matrix<Scalar, DegreesOfFreedom, 1> Twist;

  public:
    virtual void processJacobian(const JacobianMatrix &J, JacobianMatrix &Jconstrained) {
      int numFixed = this->translationConstraint_.numFixedAxes();
      Jconstrained.resize(J.rows(), J.cols() - numFixed);
      unsigned int i = 0;
      for (unsigned int j = 0; j < J.cols(); j++) {
        if (j < 3) {
          if (!this->translationConstraint_.getFixedAxes()[j]) {
            Jconstrained.col(i) = J.col(j);
            i++;
          }
        } else {
          Jconstrained.col(i) = J.col(j);
          i++;
        }
      }
    }

    /**
     * @brief
     *
     * @param twist
     *   Recreate from the missing parts of the lie algebra
     *   Adds a zero translation to each fixed axis in the lie algebra
     *
     * @return
     */
    virtual Twist getTwist(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &twist) {
      Twist xc;
      int i = 0;
      int j = 0;
      int numfixed = 0;
      for (bool axis : this->translationConstraint_.getFixedAxes()) {
        if (axis) {
          xc(i) = 0;
          numfixed++;
        } else {
          xc(i) = twist(j);
          j++;
        }
        ++i;
      }
      for (; j < twist.rows(); ++j)
      {
        xc(numfixed + j) = twist(j);
      }
      LOG(INFO) << xc;
      LOG(INFO) << la::expLie(xc);
      return xc;
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

    void processJacobian(const JacobianMatrix &J, JacobianMatrix &Jconstrained) {
      LOG(INFO) << translationConstraint_.getFixedAxes()[0] << ", " << translationConstraint_.getFixedAxes()[1] << ", " << translationConstraint_.getFixedAxes()[2];
      Eigen::Matrix<Scalar, 3, DegreesOfFreedom> weights;
      weights.setOnes();
      int i = 0;
      for (bool axis : this->translationConstraint_.getFixedAxes()) {
        if (axis) {
          weights(i, i) = 0;
          LOG(INFO) << "Set weight to 0";
        } else {
          LOG(INFO) << "Set weight to 1";
          weights(i, i) = 1;
        }
        ++i;
      }
      LOG(INFO) << "W: " << weights;
      Jconstrained = J;
      for (i = 0; i < J.rows() / 3; i++) {
        Jconstrained.block(i, 0, 3, DegreesOfFreedom) = weights.array() * Jconstrained.block(i, 0, 3, DegreesOfFreedom).array();
      }
      LOG(INFO) << "Jc: " << Jconstrained;
    }
};

DEFINE_CONSTRAINT_TYPES(float, 6, );
DEFINE_CONSTRAINT_TYPES(float, 6, f);
DEFINE_CONSTRAINT_TYPES(float, 7, );
DEFINE_CONSTRAINT_TYPES(float, 7, f);


}  // namespace icp

#endif
