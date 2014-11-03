#include "mestimator_hubert.hpp"

namespace icp {

template <typename Scalar>
void MEstimatorHubert<Scalar>::computeWeights(const Pc::Ptr pc) {
  MaximumAbsoluteDeviation<float> mad;
  Eigen::MatrixXf m = pc->getMatrixXfMap().transpose();
  LOG(INFO) << "Rows: " << m.rows() << ", cols: " << m.cols();
  mad(m.col(0));
  LOG(INFO) << "MAD x = " << mad.getMad();
  mad(m.col(1));
  LOG(INFO) << "MAD y = " << mad.getMad();
  mad(m.col(2));
  LOG(INFO) << "MAD z = " << mad.getMad();

  Eigen::MatrixXf weights_(m.rows(), m.cols());
  LOG(FATAL) << "TODO";
};

}  // namespace icp

template class icp::MEstimatorHubert<float>;
