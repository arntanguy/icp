#ifndef LINEAR_ALGEBRA_HPP
#define LINEAR_ALGEBRA_HPP

#undef Success
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <vector>
#include "sophus/sim3.hpp"

namespace Eigen {
typedef Eigen::Matrix<float, 6, 1> Vector6f;
}  // namespace Eigen

namespace la {

// Function declarations //

template<class T> Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1> x);
template<class T> Eigen::Matrix<T, 4, 4> skew(const Eigen::Matrix<T, 6, 1> x);
template<class T> Eigen::Matrix<T, 3, 3> expSO3(const Eigen::Matrix<T, 3, 1> vector);
template<class T> Eigen::Matrix<T, 3, 1> lnSO3(const Eigen::Matrix<T, 3, 3> matrix);
template<class T> Eigen::Matrix<T, 4, 4> expSE3(const Eigen::Matrix<T, 6, 1> x);
template<class T> Eigen::Matrix<T, 4, 4> expSIM3(const Eigen::Matrix<T, 7, 1> vector);
// SE3
template<typename T> Eigen::Matrix<T, 4, 4> expLie(const Eigen::Matrix<T, 6, 1>& x);
// SIM3
template<typename T> Eigen::Matrix<T, 4, 4> expLie(const Eigen::Matrix<T, 7, 1>& x);
// SO3
template<typename T> Eigen::Matrix<T, 4, 4> expLie(const Eigen::Matrix<T, 3, 1>& x);

template<typename T>
Eigen::Matrix<T, 4, 4> expLie(const Eigen::Matrix<T, 6, 1>& x) {
  return expSE3(x);
}
template<typename T>
Eigen::Matrix<T, 4, 4> expLie(const Eigen::Matrix<T, 7, 1>& x) {
  return Sophus::Sim3Group<T>::exp(x).matrix();
}

template<typename T>
Eigen::Matrix<T, 4, 4> expLie(const Eigen::Matrix<T, 3, 1>& x) {
  Eigen::Matrix<T, 4, 4> trans = Eigen::Matrix<T, 4, 4>::Identity();
  Eigen::Matrix<T, 3, 3> rot = la::expSO3(x);
  trans(0, 0) = rot(0, 0); 
  trans(0, 1) = rot(0, 1); 
  trans(0, 2) = rot(0, 2); 
  trans(1, 0) = rot(1, 0); 
  trans(1, 1) = rot(1, 1); 
  trans(1, 2) = rot(1, 2); 
  trans(2, 0) = rot(2, 0); 
  trans(2, 1) = rot(2, 1); 
  trans(2, 2) = rot(2, 2); 
  return  trans;
}

template<class T> Eigen::Matrix<T, 3, 3> q_to_R(Eigen::Matrix<T, 4, 1> q) {
  T qx = q(0);
  T qy = q(1);
  T qz = q(2);
  T qw = q(3);

  Eigen::Matrix<T, 3, 3> R;

  R << qx *qx + qy *qy - qz *qz - qw *qw, 2 * (qy * qz - qx * qw),
  2 * (qy * qw + qx * qz),
  2 * (qy * qz + qx * qw), qx *qx - qy *qy + qz *qz - qw *qw,
  2 * (qz * qw - qx * qy),
  2 * (qy * qw - qx * qz), 2 * (qz * qw + qx * qy),
  qx *qx - qy *qy - qz *qz + qw *qw;

  return R;
}
template<class T> Eigen::Matrix<T, 4, 1> R_to_q(Eigen::Matrix<T, 3, 3> a) {
  Eigen::Matrix<T, 4, 1> q;

  T trace = a.trace();
  if ( trace > 0.0 ) {
    float s = 0.5f / sqrtf(trace + 1.0f);
    q(3) = 0.25f / s;
    q(0) = ( a(2, 1) - a(1, 2) ) * s;
    q(1) = ( a(0, 2) - a(2, 0) ) * s;
    q(2) = ( a(1, 0) - a(0, 1) ) * s;
  } else {
    if ( a(0, 0) > a(1, 1) && a(0, 0) > a(2, 2) ) {
      float s = 2.0f * sqrtf( 1.0f + a(0, 0) - a(1, 1) - a(2, 2));
      q(3) = (a(2, 1) - a(1, 2) ) / s;
      q(0) = 0.25f * s;
      q(1) = (a(0, 1) + a(1, 0) ) / s;
      q(2) = (a(0, 2) + a(2, 0) ) / s;
    } else if (a(1, 1) > a(2, 2)) {
      float s = 2.0f * sqrtf( 1.0f + a(1, 1) - a(0, 0) - a(2, 2));
      q(3) = (a(0, 2) - a(2, 0) ) / s;
      q(0) = (a(0, 1) + a(1, 0) ) / s;
      q(1) = 0.25f * s;
      q(2) = (a(1, 2) + a(2, 1) ) / s;
    } else {
      float s = 2.0f * sqrtf( 1.0f + a(2, 2) - a(0, 0) - a(1, 1) );
      q(3) = (a(1, 0) - a(0, 1) ) / s;
      q(0) = (a(0, 2) + a(2, 0) ) / s;
      q(1) = (a(1, 2) + a(2, 1) ) / s;
      q(2) = 0.25f * s;
    }
  }
  return q;
}

template<class T>
inline Eigen::Matrix<T, 6, 1> covariance_axes(const Eigen::Matrix<T, 6, 6>
    &_covariance) {
  Eigen::Matrix<T, 6, 1> axes = _covariance.eigenvalues().real();
  axes = axes.array().sqrt();
  // 99% ellipse : 3 times the standard deviation //
  return 3 * axes;
}


template<class T> Eigen::Matrix<T, 3, 1> normal(
  std::vector<Eigen::Matrix<T, 3, 1> > &list, Eigen::Matrix<T, 3, 1> &centroid);


template<class T>
void rodrigues_so3_exp(const Eigen::Matrix<T, 3, 1> &w, const T A, const T B,
                       Eigen::Matrix<T, 3, 3> &R) {
  {
    const T wx2 = (T)w(0) * w(0);
    const T wy2 = (T)w(1) * w(1);
    const T wz2 = (T)w(2) * w(2);

    R(0, 0) = 1.0 - B * (wy2 + wz2);
    R(1, 1) = 1.0 - B * (wx2 + wz2);
    R(2, 2) = 1.0 - B * (wx2 + wy2);
  }
  {
    const T a = A * w(2);
    const T b = B * (w(0) * w(1));
    R(0, 1) = b - a;
    R(1, 0) = b + a;
  }
  {
    const T a = A * w(1);
    const T b = B * (w(0) * w(2));
    R(0, 2) = b + a;
    R(2, 0) = b - a;
  }
  {
    const T a = A * w(0);
    const T b = B * (w(1) * w(2));
    R(1, 2) = b - a;
    R(2, 1) = b + a;
  }
}


template<class T>
inline Eigen::Matrix<T, 3, 3>
expSO3(const Eigen::Matrix<T, 3, 1> w) {
  using std::sqrt;
  using std::sin;
  using std::cos;
  static const T one_6th = 1.0 / 6.0;
  static const T one_20th = 1.0 / 20.0;

  Eigen::Matrix<T, 3, 3> R;

  const T theta_sq = w.dot(w);
  const T theta = sqrt(theta_sq);
  T A, B;
  //Use a Taylor series expansion near zero. This is required for
  //accuracy, since sin t / t and (1-cos t)/t^2 are both 0/0.
  if (theta_sq < 1e-8) {
    A = 1.0 - one_6th * theta_sq;
    B = 0.5;
  } else {
    if (theta_sq < 1e-6) {
      B = 0.5 - 0.25 * one_6th * theta_sq;
      A = 1.0 - theta_sq * one_6th * (1.0 - one_20th * theta_sq);
    } else {
      const T inv_theta = 1.0 / theta;
      A = sin(theta) * inv_theta;
      B = (1 - cos(theta)) * (inv_theta * inv_theta);
    }
  }
  rodrigues_so3_exp(w, A, B, R);
  return R;
}


template<class T>
inline Eigen::Matrix<T, 3, 1>
lnSO3(const Eigen::Matrix<T, 3, 3> R) {
  using std::sqrt;
  Eigen::Matrix<T, 3, 1> result;

  const T cos_angle = (R(0, 0) + R(1, 1) + R(2, 2) - 1.0) * 0.5;
  result(0) = (R(2, 1) - R(1, 2)) / 2;
  result(1) = (R(0, 2) - R(2, 0)) / 2;
  result(2) = (R(1, 0) - R(0, 1)) / 2;

  T sin_angle_abs = sqrt(result.dot(result));
  if (cos_angle > M_SQRT1_2) {            // [0 - Pi/4[ use asin
    if (sin_angle_abs > 0) {
      result *= asin(sin_angle_abs) / sin_angle_abs;
    }
  } else if ( cos_angle >
              -M_SQRT1_2) {   // [Pi/4 - 3Pi/4[ use acos, but antisymmetric part
    const T angle = acos(cos_angle);
    result *= angle / sin_angle_abs;
  } else {  // rest use symmetric part
    // antisymmetric part vanishes, but still large rotation, need information from symmetric part
    const T angle = M_PI - asin(sin_angle_abs);
    const T d0 = R(0, 0) - cos_angle,
            d1 = R(1, 1) - cos_angle,
            d2 = R(2, 2) - cos_angle;

    Eigen::Matrix<T, 3, 1> r2;

    if (d0 * d0 > d1 * d1 &&
        d0 * d0 > d2 * d2) { // first is largest, fill with first column
      r2(0) = d0;
      r2(1) = (R(1, 0) + R(0, 1)) / 2;
      r2(2) = (R(0, 2) + R(2, 0)) / 2;
    } else if (d1 * d1 > d2 *
               d2) { 			 // second is largest, fill with second column
      r2(0) = (R(1, 0) + R(0, 1)) / 2;
      r2(1) = d1;
      r2(2) = (R(2, 1) + R(1, 2)) / 2;
    } else {							    // third is largest, fill with third column
      r2[0] = (R(0, 2) + R(2, 0)) / 2;
      r2[1] = (R(2, 1) + R(1, 2)) / 2;
      r2[2] = d2;
    }
    // flip, if we point in the wrong direction!
    if (r2.dot(result) < 0)
      r2 *= -1;

    // r2 = r2/r2.norm();
    r2.normalize();
    result = angle * r2;
  }

  return result;
}




template<class T>
inline Eigen::Matrix<T, 4, 4> expSE3(const Eigen::Matrix<T, 6, 1> x) {
  Eigen::Matrix<T, 4, 4> P = Eigen::Matrix<T, 4, 4>::Identity();

  T one_6th = 1.0 / 6.0;
  T one_20th = 1.0 / 20.0;

  Eigen::Matrix<T, 3, 1> w = x.block(3, 0, 3, 1);
  Eigen::Matrix<T, 3, 1> t = x.block(0, 0, 3, 1);

  double theta_sq = w.dot(w);
  double theta = sqrt(theta_sq);
  Eigen::Matrix<T, 3, 1> cross = skew(w) * t;

  Eigen::Matrix<T, 3, 3> R_out;
  Eigen::Matrix<T, 3, 1> t_out;

  double A, B;

  if (theta_sq < 1e-8)
  {
    A = 1.0 - one_6th * theta_sq;
    B = 0.5;
    t_out = t + 0.5 * cross;
  }
  else
  {
    double C;
    if (theta_sq < 1e-6)
    {
      C = one_6th * (1.0 - one_20th * theta_sq);
      A = 1.0 - theta_sq * C;
      B = 0.5 - 0.25 * one_6th * theta_sq;
    }
    else
    {
      double inv_theta = 1.0 / theta;
      A = sin(theta) * inv_theta;
      B = (1 - cos(theta)) * (inv_theta * inv_theta);
      C = (1 - A) * (inv_theta * inv_theta);
    }
    t_out = t +  B * cross + C * skew(w) * cross;
  }
  R_out = expSO3(w);

  P.block(0, 0, 3, 3) = R_out;
  P.block(0, 3, 3, 1) = t_out;

  return P;
}

template<class T>
inline Eigen::Matrix<T, 4, 4> expSIM3(const Eigen::Matrix<T, 7, 1> x) {
  Eigen::Matrix<T, 4, 4> P = Eigen::Matrix<T, 4, 4>::Identity();
  T lambda = x(6);
  if(lambda == 0) return P;

  Eigen::Matrix<T, 3, 1> w = x.block(3, 0, 3, 1);
  Eigen::Matrix<T, 3, 1> t = x.block(0, 0, 3, 1);
  T lambda_sq = lambda*lambda;

  Eigen::Matrix<T, 3, 3> w_skew = skew(w);
  Eigen::Matrix<T, 3, 3> w_skew_sq = w_skew * w_skew;

  T theta_sq = w.transpose() * w;
  T theta = sqrt(theta_sq);
  T alpha = lambda_sq/(lambda_sq+theta_sq);
  T beta = (exp(-lambda)-1+lambda)/lambda_sq;

  T X, Y, Z, W;

  if(theta_sq < 10e-6) {
    X = 1. + theta_sq / 6.;
    Y = 0.5 - theta_sq / 24.;
    Z = 1./6. + theta_sq / 120.;
    W = 1./24. - theta_sq / 720.;
  } else {
    T inv_theta = 1.0/theta;
    T inv_theta_sq = inv_theta*inv_theta;
    
    X = sin(theta) * inv_theta;
    Y = (1-cos(theta))*inv_theta_sq;
    Z = (1-X)*inv_theta_sq;
    W = (0.5-Y)*inv_theta_sq;
  }

  T gamma = Y - lambda * Z;
  T mu = (1-lambda+0.25*lambda_sq-exp(-lambda))/lambda_sq;
  T nu = Z-lambda*W;
  T A = (1-exp(-lambda))/lambda;
  T B = alpha*(beta-gamma)+gamma;
  T C = alpha*(mu-nu)+nu;

  Eigen::Matrix<T, 3, 3> R = Eigen::Matrix<T, 3, 3>::Identity() + A * w_skew + B * w_skew_sq;
  Eigen::Matrix<T, 3, 3> V = A*Eigen::Matrix<T, 3, 3>::Identity() + B*w_skew+ C*w_skew_sq;
  P << exp(lambda)*R, V*t, 0, 0, 0, 1; 

  return P;
}

template<class T>
inline Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1> x) {
  Eigen::Matrix<T, 3, 3> A;

  A(0, 0) = 0.0; A(0, 1) = -x(2); A(0, 2) = x(1);
  A(1, 0) = x(2); A(1, 1) = 0.0; A(1, 2) = -x(0);
  A(2, 0) = -x(1); A(2, 1) = x(0); A(2, 2) = 0.0;

  return A;
}

template<class T>
inline Eigen::Matrix<T, 4, 4> skew(const Eigen::Matrix<T, 6, 1> x)
{
  Eigen::Matrix<T, 4, 4> A;

  A(0, 0) = 0.0; A(0, 1) = -x(5); A(0, 2) = x(4); A(0, 3) = x(0);
  A(1, 0) = x(5); A(1, 1) = 0.0; A(1, 2) = -x(3); A(1, 3) = x(1);
  A(2, 0) = -x(4); A(2, 1) = x(3); A(2, 2) = 0.0; A(2, 3) = x(2);
  A(3, 0) = 0.0; A(3, 1) = 0.0; A(3, 2) = 0.0; A(3, 3) = 0.0;

  return A;
}

//////////////////////// PCL STUFF //////////////////////////
/** \brief Compute the roots of a quadratic polynom x^2 + b*x + c = 0
  * \param[in] b linear parameter
  * \param[in] c constant parameter
  * \param[out] roots solutions of x^2 + b*x + c = 0
  */
template<typename Scalar, typename Roots> inline void
computeRoots2 (const Scalar &b, const Scalar &c, Roots &roots) {
  roots (0) = Scalar (0);
  Scalar d = Scalar (b * b - 4.0 * c);
  if (d < 0.0) // no real roots!!!! THIS SHOULD NOT HAPPEN!
    d = 0.0;

  Scalar sd = ::std::sqrt (d);

  roots (2) = 0.5f * (b + sd);
  roots (1) = 0.5f * (b - sd);
}


/** \brief computes the roots of the characteristic polynomial of the input matrix m, which are the eigenvalues
  * \param[in] m input matrix
  * \param[out] roots roots of the characteristic polynomial of the input matrix m, which are the eigenvalues
  */
template<typename Matrix, typename Roots> inline void
computeRoots (const Matrix &m, Roots &roots) {
  typedef typename Matrix::Scalar Scalar;

  // The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0.  The
  // eigenvalues are the roots to this equation, all guaranteed to be
  // real-valued, because the matrix is symmetric.
  Scalar c0 =            m (0, 0) * m (1, 1) * m (2, 2)
                         + Scalar (2) * m (0, 1) * m (0, 2) * m (1, 2)
                         - m (0, 0) * m (1, 2) * m (1, 2)
                         - m (1, 1) * m (0, 2) * m (0, 2)
                         - m (2, 2) * m (0, 1) * m (0, 1);
  Scalar c1 = m (0, 0) * m (1, 1) -
              m (0, 1) * m (0, 1) +
              m (0, 0) * m (2, 2) -
              m (0, 2) * m (0, 2) +
              m (1, 1) * m (2, 2) -
              m (1, 2) * m (1, 2);
  Scalar c2 = m (0, 0) + m (1, 1) + m (2, 2);


  if (fabs (c0) <
      Eigen::NumTraits<Scalar>::epsilon ())// one root is 0 -> quadratic equation
    computeRoots2 (c2, c1, roots);
  else
  {
    const Scalar s_inv3 = Scalar (1.0 / 3.0);
    const Scalar s_sqrt3 = std::sqrt (Scalar (3.0));
    // Construct the parameters used in classifying the roots of the equation
    // and in solving the equation for the roots in closed form.
    Scalar c2_over_3 = c2 * s_inv3;
    Scalar a_over_3 = (c1 - c2 * c2_over_3) * s_inv3;
    if (a_over_3 > Scalar (0))
      a_over_3 = Scalar (0);

    Scalar half_b = Scalar (0.5) * (c0 + c2_over_3 * (Scalar (
                                      2) * c2_over_3 * c2_over_3 - c1));

    Scalar q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;
    if (q > Scalar (0))
      q = Scalar (0);

    // Compute the eigenvalues by solving for the roots of the polynomial.
    Scalar rho = std::sqrt (-a_over_3);
    Scalar theta = std::atan2 (std::sqrt (-q), half_b) * s_inv3;
    Scalar cos_theta = std::cos (theta);
    Scalar sin_theta = std::sin (theta);
    roots (0) = c2_over_3 + Scalar (2) * rho * cos_theta;
    roots (1) = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
    roots (2) = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

    // Sort in increasing order.
    if (roots (0) >= roots (1))
      std::swap (roots (0), roots (1));
    if (roots (1) >= roots (2))
    {
      std::swap (roots (1), roots (2));
      if (roots (0) >= roots (1))
        std::swap (roots (0), roots (1));
    }

    if (roots (0) <=
        0) // eigenval for symetric positive semi-definite matrix can not be negative! Set it to 0
      computeRoots2 (c2, c1, roots);
  }
}


/** \brief determines the eigenvector and eigenvalue of the smallest eigenvalue of the symmetric positive semi definite input matrix
  * \param[in] mat symmetric positive semi definite input matrix
  * \param[out] eigenvalue smallest eigenvalue of the input matrix
  * \param[out] eigenvector the corresponding eigenvector for the input eigenvalue
  * \note if the smallest eigenvalue is not unique, this function may return any eigenvector that is consistent to the eigenvalue.
  * \ingroup common
  */
template<typename Matrix, typename Vector> inline void
eigen33 (const Matrix &mat, typename Matrix::Scalar &eigenvalue,
         Vector &eigenvector) {
  typedef typename Matrix::Scalar Scalar;
  // Scale the matrix so its entries are in [-1,1].  The scaling is applied
  // only when at least one matrix entry has magnitude larger than 1.

  Scalar scale = mat.cwiseAbs ().maxCoeff ();
  if (scale <= std::numeric_limits<Scalar>::min ())
    scale = Scalar (1.0);

  Matrix scaledMat = mat / scale;

  Vector eigenvalues;
  computeRoots (scaledMat, eigenvalues);

  eigenvalue = eigenvalues (0) * scale;

  scaledMat.diagonal ().array () -= eigenvalues (0);

  Vector vec1 = scaledMat.row (0).cross (scaledMat.row (1));
  Vector vec2 = scaledMat.row (0).cross (scaledMat.row (2));
  Vector vec3 = scaledMat.row (1).cross (scaledMat.row (2));

  Scalar len1 = vec1.squaredNorm ();
  Scalar len2 = vec2.squaredNorm ();
  Scalar len3 = vec3.squaredNorm ();

  if (len1 >= len2 && len1 >= len3)
    eigenvector = vec1 / std::sqrt (len1);
  else if (len2 >= len1 && len2 >= len3)
    eigenvector = vec2 / std::sqrt (len2);
  else
    eigenvector = vec3 / std::sqrt (len3);
}


template<typename Matrix, typename Vector> inline void
eigen33 (const Matrix &mat, Matrix &evecs, Vector &evals) {
  typedef typename Matrix::Scalar Scalar;
  // Scale the matrix so its entries are in [-1,1].  The scaling is applied
  // only when at least one matrix entry has magnitude larger than 1.

  Scalar scale = mat.cwiseAbs ().maxCoeff ();
  if (scale <= std::numeric_limits<Scalar>::min ())
    scale = Scalar (1.0);

  Matrix scaledMat = mat / scale;

  // Compute the eigenvalues
  computeRoots (scaledMat, evals);

  if ((evals (2) - evals (0)) <= Eigen::NumTraits<Scalar>::epsilon ())
  {
    // all three equal
    evecs.setIdentity ();
  }
  else if ((evals (1) - evals (0)) <= Eigen::NumTraits<Scalar>::epsilon () )
  {
    // first and second equal
    Matrix tmp;
    tmp = scaledMat;
    tmp.diagonal ().array () -= evals (2);

    Vector vec1 = tmp.row (0).cross (tmp.row (1));
    Vector vec2 = tmp.row (0).cross (tmp.row (2));
    Vector vec3 = tmp.row (1).cross (tmp.row (2));

    Scalar len1 = vec1.squaredNorm ();
    Scalar len2 = vec2.squaredNorm ();
    Scalar len3 = vec3.squaredNorm ();

    if (len1 >= len2 && len1 >= len3)
      evecs.col (2) = vec1 / std::sqrt (len1);
    else if (len2 >= len1 && len2 >= len3)
      evecs.col (2) = vec2 / std::sqrt (len2);
    else
      evecs.col (2) = vec3 / std::sqrt (len3);

    evecs.col (1) = evecs.col (2).unitOrthogonal ();
    evecs.col (0) = evecs.col (1).cross (evecs.col (2));
  }
  else if ((evals (2) - evals (1)) <= Eigen::NumTraits<Scalar>::epsilon () )
  {
    // second and third equal
    Matrix tmp;
    tmp = scaledMat;
    tmp.diagonal ().array () -= evals (0);

    Vector vec1 = tmp.row (0).cross (tmp.row (1));
    Vector vec2 = tmp.row (0).cross (tmp.row (2));
    Vector vec3 = tmp.row (1).cross (tmp.row (2));

    Scalar len1 = vec1.squaredNorm ();
    Scalar len2 = vec2.squaredNorm ();
    Scalar len3 = vec3.squaredNorm ();

    if (len1 >= len2 && len1 >= len3)
      evecs.col (0) = vec1 / std::sqrt (len1);
    else if (len2 >= len1 && len2 >= len3)
      evecs.col (0) = vec2 / std::sqrt (len2);
    else
      evecs.col (0) = vec3 / std::sqrt (len3);

    evecs.col (1) = evecs.col (0).unitOrthogonal ();
    evecs.col (2) = evecs.col (0).cross (evecs.col (1));
  }
  else
  {
    Matrix tmp;
    tmp = scaledMat;
    tmp.diagonal ().array () -= evals (2);

    Vector vec1 = tmp.row (0).cross (tmp.row (1));
    Vector vec2 = tmp.row (0).cross (tmp.row (2));
    Vector vec3 = tmp.row (1).cross (tmp.row (2));

    Scalar len1 = vec1.squaredNorm ();
    Scalar len2 = vec2.squaredNorm ();
    Scalar len3 = vec3.squaredNorm ();
#ifdef _WIN32
    Scalar *mmax = new Scalar[3];
#else
    Scalar mmax[3];
#endif
    unsigned int min_el = 2;
    unsigned int max_el = 2;
    if (len1 >= len2 && len1 >= len3)
    {
      mmax[2] = len1;
      evecs.col (2) = vec1 / std::sqrt (len1);
    }
    else if (len2 >= len1 && len2 >= len3)
    {
      mmax[2] = len2;
      evecs.col (2) = vec2 / std::sqrt (len2);
    }
    else
    {
      mmax[2] = len3;
      evecs.col (2) = vec3 / std::sqrt (len3);
    }

    tmp = scaledMat;
    tmp.diagonal ().array () -= evals (1);

    vec1 = tmp.row (0).cross (tmp.row (1));
    vec2 = tmp.row (0).cross (tmp.row (2));
    vec3 = tmp.row (1).cross (tmp.row (2));

    len1 = vec1.squaredNorm ();
    len2 = vec2.squaredNorm ();
    len3 = vec3.squaredNorm ();
    if (len1 >= len2 && len1 >= len3)
    {
      mmax[1] = len1;
      evecs.col (1) = vec1 / std::sqrt (len1);
      min_el = len1 <= mmax[min_el] ? 1 : min_el;
      max_el = len1 > mmax[max_el] ? 1 : max_el;
    }
    else if (len2 >= len1 && len2 >= len3)
    {
      mmax[1] = len2;
      evecs.col (1) = vec2 / std::sqrt (len2);
      min_el = len2 <= mmax[min_el] ? 1 : min_el;
      max_el = len2 > mmax[max_el] ? 1 : max_el;
    }
    else
    {
      mmax[1] = len3;
      evecs.col (1) = vec3 / std::sqrt (len3);
      min_el = len3 <= mmax[min_el] ? 1 : min_el;
      max_el = len3 > mmax[max_el] ? 1 : max_el;
    }

    tmp = scaledMat;
    tmp.diagonal ().array () -= evals (0);

    vec1 = tmp.row (0).cross (tmp.row (1));
    vec2 = tmp.row (0).cross (tmp.row (2));
    vec3 = tmp.row (1).cross (tmp.row (2));

    len1 = vec1.squaredNorm ();
    len2 = vec2.squaredNorm ();
    len3 = vec3.squaredNorm ();
    if (len1 >= len2 && len1 >= len3)
    {
      mmax[0] = len1;
      evecs.col (0) = vec1 / std::sqrt (len1);
      min_el = len3 <= mmax[min_el] ? 0 : min_el;
      max_el = len3 > mmax[max_el] ? 0 : max_el;
    }
    else if (len2 >= len1 && len2 >= len3)
    {
      mmax[0] = len2;
      evecs.col (0) = vec2 / std::sqrt (len2);
      min_el = len3 <= mmax[min_el] ? 0 : min_el;
      max_el = len3 > mmax[max_el] ? 0 : max_el;
    }
    else
    {
      mmax[0] = len3;
      evecs.col (0) = vec3 / std::sqrt (len3);
      min_el = len3 <= mmax[min_el] ? 0 : min_el;
      max_el = len3 > mmax[max_el] ? 0 : max_el;
    }

    unsigned mid_el = 3 - min_el - max_el;
    evecs.col (min_el) = evecs.col ((min_el + 1) % 3).cross ( evecs.col ((
                           min_el + 2) % 3) ).normalized ();
    evecs.col (mid_el) = evecs.col ((mid_el + 1) % 3).cross ( evecs.col ((
                           mid_el + 2) % 3) ).normalized ();
#ifdef _WIN32
    delete [] mmax;
#endif
  }
  // Rescale back to the original size.
  evals *= scale;
}
template <typename T> inline unsigned int
computeMeanAndCovarianceMatrix (const Eigen::Matrix<T, 3, 1> vertices [],
                                Eigen::Matrix<T, 3, 3> &covariance_matrix,
                                Eigen::Matrix<T, 3, 1> &centroid,
                                size_t num_vertices) {
  // create the buffer on the stack which is much faster than using cloud[indices[i]] and centroid as a buffer
  Eigen::Matrix<T, 1, 9, Eigen::RowMajor> accu =
    Eigen::Matrix<T, 1, 9, Eigen::RowMajor>::Zero ();

  // For each point
  for (size_t i = 0; i < num_vertices; ++i)
  {
    accu [0] += vertices[i](0) * vertices[i](0);
    accu [1] += vertices[i](0) * vertices[i](1);
    accu [2] += vertices[i](0) * vertices[i](2);
    accu [3] += vertices[i](1) * vertices[i](1); // 4
    accu [4] += vertices[i](1) * vertices[i](2); // 5
    accu [5] += vertices[i](2) * vertices[i](2); // 8
    accu [6] += vertices[i](0);
    accu [7] += vertices[i](1);
    accu [8] += vertices[i](2);
  }
  accu /= num_vertices;


  if (num_vertices != 0)
  {
    //centroid.head<3> () = accu.tail<3> ();    -- does not compile with Clang 3.0
    centroid[0] = accu[6]; centroid[1] = accu[7]; centroid[2] = accu[8];
    covariance_matrix.coeffRef (0) = accu [0] - accu [6] * accu [6];
    covariance_matrix.coeffRef (1) = accu [1] - accu [6] * accu [7];
    covariance_matrix.coeffRef (2) = accu [2] - accu [6] * accu [8];
    covariance_matrix.coeffRef (4) = accu [3] - accu [7] * accu [7];
    covariance_matrix.coeffRef (5) = accu [4] - accu [7] * accu [8];
    covariance_matrix.coeffRef (8) = accu [5] - accu [8] * accu [8];
    covariance_matrix.coeffRef (3) = covariance_matrix.coeff (1);
    covariance_matrix.coeffRef (6) = covariance_matrix.coeff (2);
    covariance_matrix.coeffRef (7) = covariance_matrix.coeff (5);
  }
  return num_vertices;
}




template <class T>
bool isMuchSmallerThan (T x, T y) {
  // copied from <eigen>/include/Eigen/src/Core/NumTraits.h
  const T prec_sqr = FLT_EPSILON * FLT_EPSILON;
  return x * x <= prec_sqr * y * y;
}

template <class T>
inline Eigen::Matrix<T, 3, 1> computeRoots2(const T b, const T c) {
  Eigen::Matrix<T, 3, 1> roots;
  roots(0) = 0.f;
  T d = (b) * (b) - 4.f * (c);
  if (d < 0.f) // no real roots!!!! THIS SHOULD NOT HAPPEN!
    d = 0.f;

  T sd = std::sqrt(d);

  roots(2) = 0.5f * ((b) + sd);
  roots(1) = 0.5f * ((b) - sd);

  return roots;
}
template <class T>
inline Eigen::Matrix<T, 3, 1> computeRoots3(T c0, T c1, T c2) {
  Eigen::Matrix<T, 3, 1> roots;

  if ( fabs(c0) < FLT_EPSILON)// one root is 0 -> quadratic equation
  {
    roots = computeRoots2 (c2, c1);
  }
  else
  {
    const T s_inv3 = 1.f / 3.f;
    const T s_sqrt3 = std::sqrt(3.f);
    // Construct the parameters used in classifying the roots of the equation
    // and in solving the equation for the roots in closed form.
    T c2_over_3 = c2 * s_inv3;
    T a_over_3 = (c1 - c2 * c2_over_3) * s_inv3;
    if (a_over_3 > 0.f)
      a_over_3 = 0.f;

    T half_b = 0.5f * (c0 + c2_over_3 * (2.f * c2_over_3 * c2_over_3 - c1));

    T q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;
    if (q > 0.f)
      q = 0.f;

    // Compute the eigenvalues by solving for the roots of the polynomial.
    T rho = sqrt(-a_over_3);
    T theta = atan2(sqrt (-q), half_b) * s_inv3;
    T cos_theta = cos (theta);
    T sin_theta = sin (theta);
    roots(0) = c2_over_3 + 2.f * rho * cos_theta;
    roots(1) = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
    roots(2) = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

    // Sort in increasing order.
    if (roots(0) >= roots(1))
    {
      std::swap(roots(0), roots(1));
    }

    if (roots(1) >= roots(2))
    {
      std::swap(roots(1), roots(2));

      if (roots(0) >= roots(1))
      {
        std::swap(roots(0), roots(1));
      }
    }
    if (roots(0) <=
        0) // eigenval for symetric positive semi-definite matrix can not be negative! Set it to 0
      roots = computeRoots2 (c2, c1);
  }
  return roots;
}


template <class T>
inline Eigen::Matrix<T, 3, 1> unitOrthogonal (const Eigen::Matrix<T, 3, 1>
    src) {
  Eigen::Matrix<T, 3, 1> perp;
  /* Let us compute the crossed product of *this with a vector
   * that is not too close to being colinear to *this.
   */
  /* unless the x and y coords are both close to zero, we can
   * simply take ( -y, x, 0 ) and normalize it.
   */
  if (!isMuchSmallerThan(src(0), src(2)) || !isMuchSmallerThan(src(1), src(2)))
  {
    float invnm = 1.0 / std::sqrt((src(0)) * (src(0)) + (src(1)) * (src(1)));
    perp(0) = -(src(1)) * invnm;
    perp(1) =  (src(2)) * invnm;
    perp(2) = 0.0f;
  }
  /* if both x and y are close to zero, then the vector is close
  * to the z-axis, so it's far from colinear to the x-axis for instance.
  * So we take the crossed product with (1,0,0) and normalize it.
  */
  else
  {
    float invnm = 1.0 / sqrt((src(2)) * (src(2)) + (src(1)) * (src(1)));
    perp(0) = 0.0f;
    perp(1) = -(src(2)) * invnm;
    perp(2) =  (src(1)) * invnm;
  }
  return perp;
}

template <class T>
inline Eigen::Matrix<T, 3, 1> compute(T mat_pkg[6]) {
  T max01 = std::max( fabs(mat_pkg[0]), fabs(mat_pkg[1]) );
  T max23 = std::max( fabs(mat_pkg[2]), fabs(mat_pkg[3]) );
  T max45 = std::max( fabs(mat_pkg[4]), fabs(mat_pkg[5]) );
  T m0123 = std::max( max01, max23);
  T scale = std::max( max45, m0123);

  if (scale <= FLT_EPSILON)
    scale = 1.f;

  mat_pkg[0] /= scale;
  mat_pkg[1] /= scale;
  mat_pkg[2] /= scale;
  mat_pkg[3] /= scale;
  mat_pkg[4] /= scale;
  mat_pkg[5] /= scale;


  float c0 = mat_pkg[0] * mat_pkg[3] * mat_pkg[5]
             + 2.f * mat_pkg[1] * mat_pkg[2] * mat_pkg[4]
             - mat_pkg[0] * mat_pkg[4] * mat_pkg[4]
             - mat_pkg[3] * mat_pkg[2] * mat_pkg[2]
             - mat_pkg[5] * mat_pkg[1] * mat_pkg[1];

  float c1 = mat_pkg[0] * mat_pkg[3] -
             mat_pkg[1] * mat_pkg[1] +
             mat_pkg[0] * mat_pkg[5] -
             mat_pkg[2] * mat_pkg[2] +
             mat_pkg[3] * mat_pkg[5] -
             mat_pkg[4] * mat_pkg[4];
  float c2 = mat_pkg[0] + mat_pkg[3] + mat_pkg[5];

  Eigen::Matrix<T, 3, 1>  evecs[3];
  Eigen::Matrix<T, 3, 1>  tmp[3];
  Eigen::Matrix<T, 3, 1>  vec_tmp[3];

  Eigen::Matrix<T, 3, 1>  row0 =  Eigen::Matrix<T, 3, 1> ( mat_pkg[0], mat_pkg[1],
                                  mat_pkg[2]);
  Eigen::Matrix<T, 3, 1>  row1 =  Eigen::Matrix<T, 3, 1> ( mat_pkg[1], mat_pkg[3],
                                  mat_pkg[4]);
  Eigen::Matrix<T, 3, 1>  row2 =  Eigen::Matrix<T, 3, 1> ( mat_pkg[2], mat_pkg[4],
                                  mat_pkg[5]);

  Eigen::Matrix<T, 3, 1>  evals = computeRoots3(c0, c1, c2);

  if (evals(2) - evals(0) <= FLT_EPSILON)
  {
    evecs[0] =  Eigen::Matrix<T, 3, 1>(1.f, 0.f, 0.f);
    evecs[1] =  Eigen::Matrix<T, 3, 1>(0.f, 1.f, 0.f);
    evecs[2] =  Eigen::Matrix<T, 3, 1>(0.f, 0.f, 1.f);
  }
  else if (evals(1) - evals(0) <= FLT_EPSILON)
  {
    // first and second equal
    tmp[0] = row0;  tmp[1] = row1;  tmp[2] = row2;
    tmp[0](0) -= evals(2); tmp[1](1) -= evals(2); tmp[2](2) -= evals(2);

    vec_tmp[0] = tmp[0].cross(tmp[1]);
    vec_tmp[1] = tmp[0].cross(tmp[2]);
    vec_tmp[2] = tmp[1].cross(tmp[2]);

    float len1 = vec_tmp[0].dot(vec_tmp[0]);
    float len2 = vec_tmp[1].dot(vec_tmp[1]);
    float len3 = vec_tmp[2].dot(vec_tmp[2]);

    if (len1 >= len2 && len1 >= len3)
    {
      evecs[2] = vec_tmp[0] * 1.0 / sqrt (len1);
    }
    else if (len2 >= len1 && len2 >= len3)
    {
      evecs[2] = vec_tmp[1] * 1.0 / sqrt (len2);
    }
    else
    {
      evecs[2] = vec_tmp[2] * 1.0 / sqrt (len3);
    }

    evecs[1] = unitOrthogonal(evecs[2]);
    evecs[0] = evecs[1].cross(evecs[2]);
  }
  else
  {

    tmp[0] = row0;  tmp[1] = row1;  tmp[2] = row2;
    tmp[0](0) -= evals(2); tmp[1](1) -= evals(2); tmp[2](2) -= evals(2);

    vec_tmp[0] = tmp[0].cross(tmp[1]);
    vec_tmp[1] = tmp[0].cross(tmp[2]);
    vec_tmp[2] = tmp[1].cross(tmp[2]);

    float len1 = vec_tmp[0].dot(vec_tmp[0]);
    float len2 = vec_tmp[1].dot(vec_tmp[1]);
    float len3 = vec_tmp[2].dot(vec_tmp[2]);

    float mmax[3];

    unsigned int min_el = 2;
    unsigned int max_el = 2;
    if (len1 >= len2 && len1 >= len3)
    {
      mmax[2] = len1;
      evecs[2] = vec_tmp[0] * 1.0 / sqrt (len1);
    }
    else if (len2 >= len1 && len2 >= len3)
    {
      mmax[2] = len2;
      evecs[2] = vec_tmp[1] * 1.0 / sqrt (len2);
    }
    else
    {
      mmax[2] = len3;
      evecs[2] = vec_tmp[2] * 1.0 / sqrt (len3);
    }

    tmp[0] = row0;  tmp[1] = row1;  tmp[2] = row2;
    tmp[0](0) -= evals(1); tmp[1](1) -= evals(1); tmp[2](2) -= evals(1);

    vec_tmp[0] = tmp[0].cross(tmp[1]);
    vec_tmp[1] = tmp[0].cross(tmp[2]);
    vec_tmp[2] = tmp[1].cross(tmp[2]);

    len1 = vec_tmp[0].dot(vec_tmp[0]);
    len2 = vec_tmp[1].dot(vec_tmp[1]);
    len3 = vec_tmp[2].dot(vec_tmp[2]);

    if (len1 >= len2 && len1 >= len3)
    {
      mmax[1] = len1;
      evecs[1] = vec_tmp[0] * 1.0 / sqrt(len1);
      min_el = len1 <= mmax[min_el] ? 1 : min_el;
      max_el = len1  > mmax[max_el] ? 1 : max_el;
    }
    else if (len2 >= len1 && len2 >= len3)
    {
      mmax[1] = len2;
      evecs[1] = vec_tmp[1] *  1.0 / sqrt(len2);
      min_el = len2 <= mmax[min_el] ? 1 : min_el;
      max_el = len2  > mmax[max_el] ? 1 : max_el;
    }
    else
    {
      mmax[1] = len3;
      evecs[1] = vec_tmp[2] *  1.0 / sqrt (len3);
      min_el = len3 <= mmax[min_el] ? 1 : min_el;
      max_el = len3 >  mmax[max_el] ? 1 : max_el;
    }

    tmp[0] = row0;  tmp[1] = row1;  tmp[2] = row2;
    tmp[0](0) -= evals(0); tmp[1](1) -= evals(0); tmp[2](2) -= evals(0);

    vec_tmp[0] = tmp[0].cross(tmp[1]);
    vec_tmp[1] = tmp[0].cross(tmp[2]);
    vec_tmp[2] = tmp[1].cross(tmp[2]);

    len1 = vec_tmp[0].dot(vec_tmp[0]);
    len2 = vec_tmp[1].dot(vec_tmp[1]);
    len3 = vec_tmp[2].dot(vec_tmp[2]);


    if (len1 >= len2 && len1 >= len3)
    {
      mmax[0] = len1;
      evecs[0] = vec_tmp[0] *  1.0 / sqrt (len1);
      min_el = len3 <= mmax[min_el] ? 0 : min_el;
      max_el = len3  > mmax[max_el] ? 0 : max_el;
    }
    else if (len2 >= len1 && len2 >= len3)
    {
      mmax[0] = len2;
      evecs[0] = vec_tmp[1] *  1.0 / sqrt (len2);
      min_el = len3 <= mmax[min_el] ? 0 : min_el;
      max_el = len3  > mmax[max_el] ? 0 : max_el;
    }
    else
    {
      mmax[0] = len3;
      evecs[0] = vec_tmp[2] *  1.0 / sqrt (len3);
      min_el = len3 <= mmax[min_el] ? 0 : min_el;
      max_el = len3  > mmax[max_el] ? 0 : max_el;
    }

    uint mid_el = 3 - min_el - max_el;

    evecs[min_el] = evecs[(min_el + 1) % 3].cross(evecs[(min_el + 2) % 3] );
    evecs[mid_el] = evecs[(mid_el + 1) % 3].cross(evecs[(mid_el + 2) % 3] );
    evecs[min_el].normalize();
    evecs[mid_el].normalize();

    //evecs[min_el] = normalize( cross( evecs[(min_el+1) % 3], evecs[(min_el+2) % 3] ) );
    //evecs[mid_el] = normalize( cross( evecs[(mid_el+1) % 3], evecs[(mid_el+2) % 3] ) );
  }

  // Rescale back to the original size.
  evals *= scale;
  evecs[0].normalize();

  return  evecs[0];
}


template<class T> Eigen::Matrix<T, 3, 1> normal(
  std::vector<Eigen::Matrix<T, 3, 1> > &list, Eigen::Matrix<T, 3, 1> &centroid) {
  centroid = Eigen::Matrix<T, 3, 1>::Zero();
  int counter = 0;

  for (int i = 0 ; i < list.size() ; ++i)
  {
    if (list[i](2) > 0.0)
    {
      centroid += list[i];
      counter++;
    }
  }

  Eigen::Matrix<T, 3, 3> covariance_matrix = Eigen::Matrix<T, 3, 3>::Zero(3, 3);
  Eigen::Matrix<T, 3, 1> N;

  if (counter > 0)
    centroid *= 1.f / counter;

  // Compute Covariance //
  if (counter >= 3)
  {
    for (int i = 0 ; i < list.size() ; ++i)
    {
      if (list[i](2) > 0)
      {
        Eigen::Matrix<T, 3, 1> d = list[i] - centroid;
        covariance_matrix += d * d.transpose();
      }
    }
  }

  EIGEN_ALIGN16 Eigen::Matrix<T, 3, 1> eigen_value;
  EIGEN_ALIGN16 Eigen::Matrix<T, 3, 1> eigen_vector;
  eigen33(covariance_matrix, eigen_value, eigen_vector);
  N(0) = eigen_vector [0];
  N(1) = eigen_vector [1];
  N(2) = eigen_vector [2];

  if (N.dot(centroid) > 0.0 )
    N *= -1.0;

  N.normalize();

  return N;
}

}  // namespace la

#endif // LINEAR_ALGEBRA_HPP
