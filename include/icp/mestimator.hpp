//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <icp/types.hpp>
#include <icp/eigentools.hpp>

namespace icp {

template <typename Scalar>
Scalar median_absolute_deviation(const VectorX<Scalar>& v)
{
  typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;
  Scalar median_ = eigentools::median(v);
  // Median centered residual error
  VectorX r = (v - median_).cwiseAbs();

  // median absolute deviation deviation (MAD)
  Scalar mad = eigentools::median(r);
  return mad;
}

/**
 * @brief Computes huber weight for the scaled value z
 *
 * @param z Scaled residual
 * @param c
 *
 * @return hubert weight
 */
template <typename Scalar>
Scalar hubert_weight(const Scalar z, const Scalar c = 1.345)
{
  Scalar abs_z = std::abs(z);
  if(abs_z < c)
  {
    return 1;
  }
  else
  {
    return c/abs_z;
  }
}

/**
 * @brief Compute the huber weights for the residual vector
 *
 * @param r Residual vector
 * @param result Computed hubert weight results; should have the same size as r
 * @param scale Scale factor. Should be tau = med|r_i-med(r_i)|/0.6745
 * @param c Hubert parameter
 */
template <typename Scalar>
void hubert_weight(const VectorX<Scalar>& r, VectorX<Scalar>& result, Scalar scale, Scalar c = 1.345)
{
  assert(r.size() == result.size());
  for (int i = 0; i < r.size(); ++i) {
    result[i] = hubert_weight(r[i] / scale, c);
  }
}

}  // namespace icp
