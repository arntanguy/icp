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
#include <glog/logging.h>

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
template<typename Derived>
void sort(Eigen::MatrixBase<Derived> &M) {
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
template<typename Derived>
Derived median(const Eigen::Matrix<Derived, Eigen::Dynamic, 1> &M) {
  // Work on a copy
  Eigen::Matrix<Derived, Eigen::Dynamic, 1> copy = M;
  const int len = copy.derived().size();
  // Sort half the elements 
  std::nth_element( copy.derived().data(),
                    copy.derived().data() + len / 2,
                    copy.derived().data() + len);
  // midpoint is the median
  return copy.derived()(len/2);
}

} /* eigentools */


#endif /* EIGEN_TOOLS_H */
