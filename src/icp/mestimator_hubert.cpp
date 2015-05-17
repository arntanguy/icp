#include "mestimator_hubert.hpp"
#include "instanciate.hpp"
#include "logging.hpp"

namespace icp {

template<typename Scalar, typename Point>
typename MEstimatorHubert<Scalar, Point>::VectorX
MEstimatorHubert<Scalar, Point>::weightsHuber(Scalar scale, VectorX rectified) {
  VectorX result(rectified.rows());

  const Scalar c = 1.2107 * scale; // originally was 1.345
  for (int i = 0; i < rectified.size(); ++i)
  {
    const Scalar v = rectified(i);
    if (v < c) {
      // Inlier
      result(i) = 1;
    } else {
      // Outlier
      result(i) = c / v;
    }
  }
  return result;
}

template <typename Scalar, typename Point>
void MEstimatorHubert<Scalar, Point>::computeWeights() {
  // FIXME: Without the logging, there is a corrupted unsorted chunk crash!
  LOG(INFO) <<
            "FIXME! When this line isn't printed, the MEstimator fails with a corrupted unsorted chunk crash!";
  MaximumAbsoluteDeviationVector<float> madx = mad_.getMadX();
  MaximumAbsoluteDeviationVector<float> mady = mad_.getMadY();
  MaximumAbsoluteDeviationVector<float> madz = mad_.getMadZ();
  VectorX rx = madx.getRectified();
  VectorX ry = mady.getRectified();
  VectorX rz = madz.getRectified();
  //  DLOG(INFO) << "Rows: " << m.rows() << ", cols: " << m.cols();
  //  DLOG(INFO) << "MAD x = " << madx.getMad();
  //  DLOG(INFO) << "MAD y = " << mady.getMad();
  //  DLOG(INFO) << "MAD z = " << madz.getMad();

  LOG(INFO) << "Median x, y, z: " << madx.getMedian() << ", " << mady.getMedian() << ", " << madz.getMedian();
  LOG(INFO) << "Max residual x, y, z: " << madx.getMaxResidual() << ", " << mady.getMaxResidual() << ", " << madz.getMaxResidual();
  LOG(INFO) << "Scale x, y, z: " << madx.getScale() << ", " << mady.getScale() << ", " << madz.getScale();

  VectorX wx = weightsHuber(madx.getScale(), rx);
  VectorX wy = weightsHuber(mady.getScale(), ry);
  VectorX wz = weightsHuber(madz.getScale(), rz);

  weights_.resize(rx.size(), 4);
  //  DLOG(INFO) << "weights_ size " << weights_.rows() << ", " << weights_.cols();
  for (int i = 0; i < wx.rows(); ++i)
  {
    weights_.row(i) << wx(i), wy(i), wz(i), Scalar(1);
  }
  //  DLOG(INFO) << "W: " << weights_;
};

INSTANCIATE_MESTIMATOR_HUBERT;

}  // namespace icp

