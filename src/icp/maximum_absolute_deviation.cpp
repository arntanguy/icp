#include <pcl/point_types.h>
#include "maximum_absolute_deviation.hpp"
#include "pcltools.hpp"
#include "eigentools.hpp"
#include "instanciate.hpp"

namespace icp
{


template<typename Scalar>
using VectorX = typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

/**
 * @brief Computes MAD from data.
 */
template<typename Scalar>
void MaximumAbsoluteDeviationVector<Scalar>::compute() {
  median_ = eigentools::median(data_);
  // Median centered residual error
  r_ = (data_ - median_).cwiseAbs();
  maxResidualError_ = r_.maxCoeff();

  // robust standard deviation (MAD)
  //scale = 1.4826 * (1+5/(n-p)) * median(r);
  mad_ = eigentools::median(r_);
  scale_ =  1.4826 * mad_;

  //// If MAD is less that noise threshold
  //if (scale_ < noise_threshold_) {
  //  // scale = 1.4826 * (1+5/(n-p)) * 1;
  //  scale_ = 1.4826;
  //}
}

/**
 * @brief Computes MAD from data, relative to given median and residual error.
 * WARNING: this will modify the internal median and residual error, so
 * getMedian will return the specified median and not the data's one! 
 *
 * @param median
 * @param maxResidual
 */
template<typename Scalar>
void MaximumAbsoluteDeviationVector<Scalar>::compute(const Scalar median, const Scalar maxResidual) {
  // Median centered residual error
  median_ = median;
  maxResidualError_ = maxResidual;
  r_ = (data_ - median).cwiseAbs();
  scale_ =  maxResidual; // *scale_factor
}

/**
 * @brief Computes the Maximum absolute deviation of the model and reference cloud.
 * This is meant to be used when the model's initial position is close to the
 * object location. Do not use this with MEstimators if that's not the case, as
 * it will discard most points!
 *
 * WARNING: If the model is changed after the reference mad has been computed, it will
 * need to be recomputed to give correct results!
 */
template<typename Scalar, typename PointReference, typename PointSource>
void MaximumAbsoluteDeviation<Scalar, PointReference, PointSource>::computeMadModel() {
  VectorX vx, vy, vz;
  pcltools::getColumn<Scalar, PointSource>(modelCloud_, vx, 0);
  pcltools::getColumn<Scalar, PointSource>(modelCloud_, vy, 1);
  pcltools::getColumn<Scalar, PointSource>(modelCloud_, vz, 2);
  madModelX_.setVector(vx);
  madModelX_.compute();
  madModelY_.setVector(vy);
  madModelY_.compute();
  madModelZ_.setVector(vz);
  madModelZ_.compute();
}

/**
 * @brief Computes the median absolute deviation of the pointcloud 
 * This can be used to manually specify the
 */
template<typename Scalar, typename PointReference, typename PointSource>
void MaximumAbsoluteDeviation<Scalar, PointReference, PointSource>::computeMadReference() {
  VectorX vx, vy, vz;
  pcltools::getColumn<float, PointReference>(referenceCloud_, vx, 0);
  pcltools::getColumn<float, PointReference>(referenceCloud_, vy, 1);
  pcltools::getColumn<float, PointReference>(referenceCloud_, vz, 2);
  madReferenceX_.setVector(vx);
  madReferenceX_.compute();
  madReferenceY_.setVector(vy);
  madReferenceY_.compute();
  madReferenceZ_.setVector(vz);
  madReferenceZ_.compute();
}

/**
 * @brief Computes the median absolute deviation assuming a known median.
 * This can be used to manually specify the
 */
template<typename Scalar, typename PointReference, typename PointSource>
void MaximumAbsoluteDeviation<Scalar, PointReference, PointSource>::computeMadReference(const Eigen::Matrix<Scalar, 4, 4> &medianTransform) {
  VectorX vx, vy, vz;
  pcltools::getColumn<float, PointReference>(referenceCloud_, vx, 0);
  pcltools::getColumn<float, PointReference>(referenceCloud_, vy, 1);
  pcltools::getColumn<float, PointReference>(referenceCloud_, vz, 2);
  Eigen::Matrix<Scalar, 4, 1> madModel;
  madModel << madModelX_.getMedian(), madModelY_.getMedian(), madModelZ_.getMedian(), 1;

  // Compute with the median shifted from the model's center to the estimated
  // object position in worldspace
  Eigen::Matrix<Scalar, 4, 1> madModelTransformed = medianTransform * madModel;
  madReferenceX_.setVector(vx);
  madReferenceX_.compute(madModelTransformed(0), madModelX_.getMaxResidual());
  madReferenceY_.setVector(vy);
  madReferenceY_.compute(madModelTransformed(1), madModelY_.getMaxResidual());
  madReferenceZ_.setVector(vz);
  madReferenceZ_.compute(madModelTransformed(2), madModelZ_.getMaxResidual());
}


template<typename Scalar, typename PointReference, typename PointSource>
void MaximumAbsoluteDeviation<Scalar, PointReference, PointSource>::setModelCloud(PcPtr cloud) {
  modelCloud_ = cloud;
  computeMadModel();
}

/**
 * @brief
 *
 * @param cloud
 * @param medianTransform
 * Relative transformation moving the median of the model's cloud to the desired position in the
 * reference cloud. This should be place as close as possible to the position of
 * the object in the reference cloud so that the estimator is meaningful.
 */
template<typename Scalar, typename PointReference, typename PointSource>
void MaximumAbsoluteDeviation<Scalar, PointReference, PointSource>::setReferenceCloud(PrPtr cloud,
    const Eigen::Matrix<Scalar, 4, 4> &medianTransform) {
  referenceCloud_ = cloud;
  computeMadReference(medianTransform);
}

INSTANCIATE_MAD;

}  // namespace icp
