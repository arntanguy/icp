#include <pcl/point_types.h>
#include "maximum_absolute_deviation.hpp"
#include "pcltools.hpp"
#include "eigentools.hpp"
#include "instanciate.hpp"

namespace icp
{


template<typename Scalar>
using VectorX = typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

template<typename Scalar>
VectorX<Scalar> MaximumAbsoluteDeviationVector<Scalar>::compute(const VectorX &v) {
  median_ = eigentools::median(v);
  return computeWithMedian(v, median_);
}

template<typename Scalar>
VectorX<Scalar> MaximumAbsoluteDeviationVector<Scalar>::computeWithMedian(const VectorX &v, Scalar median) {
  median_ = median;

  // Median centered residual error
  VectorX r = (v - median).cwiseAbs();

  // robust standard deviation (MAD)
  //scale = 1.4826 * (1+5/(n-p)) * median(r);
  mad_ = eigentools::median(r);
  scale_ =  1.4826 * mad_;

  // If MAD is less that noise threshold
  if (scale_ < noise_threshold_) {
    // scale = 1.4826 * (1+5/(n-p)) * 1;
    scale_ = 1.4826;
  }

  return r;
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
template<typename Scalar, typename Point>
void MaximumAbsoluteDeviation<Scalar, Point>::computeMadModel() {
  VectorX vx, vy, vz;
  pcltools::getColumn<float, Point>(modelCloud_, vx, 0);
  pcltools::getColumn<float, Point>(modelCloud_, vy, 1);
  pcltools::getColumn<float, Point>(modelCloud_, vz, 2);
  madModelX_.compute(vx);
  madModelY_.compute(vy);
  madModelZ_.compute(vz);
}

/**
 * @brief Computes the median absolute deviation assuming a known median.
 * This can be used to manually specify the
 */
template<typename Scalar, typename Point>
void MaximumAbsoluteDeviation<Scalar, Point>::computeMadReference(const Eigen::Matrix<Scalar, 4, 4> &medianTransform) {
  VectorX vx, vy, vz;
  pcltools::getColumn<float, Point>(referenceCloud_, vx, 0);
  pcltools::getColumn<float, Point>(referenceCloud_, vy, 1);
  pcltools::getColumn<float, Point>(referenceCloud_, vz, 2);
  Eigen::Matrix<Scalar, 4, 1> madModel;
  madModel << madModelX_.getMedian(), madModelY_.getMedian(), madModelZ_.getMedian(), 1;
  
  // Compute with the median shifted from the model's center to the estimated
  // object position in worldspace
  Eigen::Matrix<Scalar, 4, 1> madModelTransformed = medianTransform * madModel;
  madReferenceX_.computeWithMedian(vx, madModelTransformed(0));
  madReferenceY_.computeWithMedian(vy, madModelTransformed(1));
  madReferenceZ_.computeWithMedian(vz, madModelTransformed(2));
}

template<typename Scalar, typename Point>
void MaximumAbsoluteDeviation<Scalar, Point>::setModelCloud(PcPtr cloud) {
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
template<typename Scalar, typename Point>
void MaximumAbsoluteDeviation<Scalar, Point>::setReferenceCloud(PcPtr cloud,
    const Eigen::Matrix<Scalar, 4, 4> &medianTransform) {
  referenceCloud_ = cloud;
  computeMadReference(medianTransform);
}

INSTANCIATE_MAD;

}  // namespace icp
