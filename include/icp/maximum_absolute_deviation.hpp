#ifndef ICP_MAXIMUM_ABSOLUTE_DEVIATION_HPP
#define ICP_MAXIMUM_ABSOLUTE_DEVIATION_HPP

#include <Eigen/Core>

#include <pcl/point_cloud.h>


namespace icp
{

/**
 * @brief Computes the maximum absolute deviation of a Vector
 */
template<typename Scalar>
class MaximumAbsoluteDeviationVector
{
  public:
    typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;

  protected:
    //! Median of the input cloud
    Scalar median_;
    //! Maximum absolute deviation
    /*! Median-centered residual error */
    Scalar mad_;
    Scalar maxResidualError_;
    Scalar scale_;
    Scalar noise_threshold_;
    //! Data vector on which the mad is computed
    VectorX data_;
    //! Median centered residual error
    VectorX r_;

  protected:

  public:
    MaximumAbsoluteDeviationVector(float noise_threshold = 0.01)
      : noise_threshold_(noise_threshold)
    {}

    void setVector(const VectorX &v) {
      data_ = v;
      compute();
    }

    void compute();
    void compute(const Scalar median, const Scalar maxResidual);

    Scalar getMaxResidual() const {
      return maxResidualError_;
    }

    Scalar getMedian() const {
      return median_;
    }

    Scalar getMaxResidualError() const {
      return maxResidualError_;
    }

    Scalar getMad() const {
      return mad_;
    }

    VectorX getRectified() const {
      return r_;
    }

    Scalar getScale() const {
      return scale_;
    }
};


/**
 * @brief Computes the Maximum Absolute Deviation along
 * each axis of a pointcloud
 */
template<typename Scalar, typename Point>
class MaximumAbsoluteDeviation
{
 public:
  typedef typename pcl::PointCloud<Point> Pc;
  typedef typename pcl::PointCloud<Point>::Ptr PcPtr;
  typedef typename MaximumAbsoluteDeviationVector<Scalar>::VectorX VectorX;

  PcPtr modelCloud_;
  PcPtr referenceCloud_;

 protected:
  MaximumAbsoluteDeviationVector<Scalar> madModelX_;
  MaximumAbsoluteDeviationVector<Scalar> madModelY_;
  MaximumAbsoluteDeviationVector<Scalar> madModelZ_;
  MaximumAbsoluteDeviationVector<Scalar> madReferenceX_;
  MaximumAbsoluteDeviationVector<Scalar> madReferenceY_;
  MaximumAbsoluteDeviationVector<Scalar> madReferenceZ_;

 protected:
  void computeMadModel(); 
  void computeMadReference(); 
  void computeMadReference(const Eigen::Matrix<Scalar, 4, 4>& medianTransform); 

 public:
  MaximumAbsoluteDeviation() {
  }

  void setModelCloud(PcPtr cloud);
  void setReferenceCloud(PcPtr cloud); 
  void setReferenceCloud(PcPtr cloud, const Eigen::Matrix<Scalar, 4, 4>& medianTransform); 

  MaximumAbsoluteDeviationVector<Scalar> getMadX() const {
    return madReferenceX_;
  } 
 MaximumAbsoluteDeviationVector<Scalar> getMadY() const {
    return madReferenceY_;
  } 
  MaximumAbsoluteDeviationVector<Scalar> getMadZ() const {
    return madReferenceZ_;
  } 
};


}  // namespace icp

#endif
