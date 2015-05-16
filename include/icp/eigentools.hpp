//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#ifndef EIGEN_TOOLS_H
#define EIGEN_TOOLS_H

#include <algorithm>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace eigentools
{

/**
 * \brief Creates a transformation matrix:
 * [ R3x3, T3x1]
 * [ 0   , 1   ]
 *
 * \params
 *   tx, ty, tz : translation
 *   rx, ry, rz : rotation around axis x, y and z (in radian)
 *
 * \returns
 * The transformation matrix
 */
template<typename T>
Eigen::Matrix<T, 4, 4> createTransformationMatrix(T tx, T ty, T tz,
    T rx, T ry, T rz) {
  typedef Eigen::Transform<T, 3, Eigen::Affine> Affine3;
  typedef Eigen::Matrix<T, 3, 1> Vector3;

  Affine3 r =
    Affine3(Eigen::AngleAxis<T>(rx, Vector3::UnitX()))
    * Affine3(Eigen::AngleAxis<T>(ry,  Vector3::UnitY()))
    * Affine3(Eigen::AngleAxis<T>(rz, Vector3::UnitZ()));
  Affine3 t(Eigen::Translation<T, 3>(Vector3(tx, ty, tz)));
  Eigen::Matrix<T, 4, 4> m = (t * r).matrix();
  return m;
}


/*
 * \brief Computes the  (Moore-Penrose) pseudo inverse
 */
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a,
                            double epsilon = std::numeric_limits<double>::epsilon())
{
  Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,
                                        Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = epsilon * std::max(a.cols(),
                                        a.rows()) * svd.singularValues().array().abs()(0);
  return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(
           svd.singularValues().array().inverse(),
           0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

/**
 * @brief Sort vector in place
 *
 * @param M
 * The vector to be sorted
 */
template<typename Scalar>
void sort(Eigen::MatrixBase<Scalar> &M) {
  std::sort(M.derived().data(), M.derived().data() + M.derived().size());
}

/**
 * @brief Computes median of eigen vector
 *
 * @param M
 * The vector
 *
 * @return
 * Median of vector (element len/2 of the sorted array). Uses std::nth_element
 * to sort only half of the array.
 */
template<typename Scalar>
Scalar median(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &M) {
  // Work on a copy
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> copy = M;
  const int len = copy.derived().size();
  if (len % 2 == 0) {
    // Even number of elements,
    // the median is the average of the two central values
    // Sort half the elements
    std::nth_element( copy.data(),
                      copy.data() + len / 2 - 1,
                      copy.data() + len);
    const Scalar n1 = copy(len / 2 - 1);
    std::nth_element( copy.data(),
                      copy.data() + len / 2,
                      copy.data() + len);
    const Scalar n2 = copy(len / 2);
    return (n1 + n2) / Scalar(2);
  } else {
    std::nth_element( copy.data(),
                      copy.data() + len / 2,
                      copy.data() + len);
    // midpoint is the median
    return copy(len / 2 );
  }
}

template<typename Scalar>
void removeRow(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &matrix, unsigned int rowToRemove)
{
  unsigned int numRows = matrix.rows() - 1;
  unsigned int numCols = matrix.cols();

  if ( rowToRemove < numRows )
    matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) = matrix.block(rowToRemove + 1, 0, numRows - rowToRemove,
        numCols);

  matrix.conservativeResize(numRows, numCols);
}

template<typename Scalar>
void removeColumn(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &matrix, unsigned int colToRemove)
{
  unsigned int numRows = matrix.rows();
  unsigned int numCols = matrix.cols() - 1;

  if ( colToRemove < numCols )
    matrix.block(0, colToRemove, numRows, numCols - colToRemove) = matrix.block(0, colToRemove + 1, numRows,
        numCols - colToRemove);

  matrix.conservativeResize(numRows, numCols);
}

} /* eigentools */

//template<typename Scalar>
//Eigen::Matrix<Scalar, Eigen::Dynamic, 1> operator-(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &mat, Scalar s)
//{
//  return mat - s * Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Ones(mat.rows(), mat.cols());
//}

template<typename DerivedA, typename Scalar>
Eigen::Matrix<typename Eigen::MatrixBase<DerivedA>::Scalar, Eigen::Dynamic, Eigen::Dynamic>
operator-(const Eigen::MatrixBase<DerivedA> &M1,
          Scalar s) {

  return M1 - s * Eigen::Matrix<typename Eigen::MatrixBase<DerivedA>::Scalar, Eigen::Dynamic, Eigen::Dynamic>::Ones(
           M1.rows(), M1.cols());
}

#endif /* EIGEN_TOOLS_H */
