#ifndef ICP_CONSTRAINTS_HPP
#define ICP_CONSTRAINTS_HPP

#include <Eigen/Core>
#include "eigentools.hpp"
#include "logging.hpp"

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

    bool hasConstraints() const {
      return translationConstraint_.numFixedAxes() != 0;
    }

    void processJacobian(JacobianMatrix &J) {
      unsigned int i = 0;
      for (bool axis : translationConstraint_.getFixedAxes()) {
        if (axis) {
          eigentools::removeColumn(J, i);
        }
        ++i;
      }
    }

    Twist getTwist(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &twist) {
      /*
       * Recreate from the missing parts of the lie algebra
       * Adds a zero translation to each fixed axis in the lie algebra
      */
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
      for (; j < twist.rows(); ++j)
      {
        LOG(INFO) << "xc(" << numfixed+j << ")";
        xc(numfixed + j) = twist(j);
      }
      return xc;
    }
};

}  // namespace icp

#endif
