//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#ifndef ICP_RESULT_HPP
#define ICP_RESULT_HPP

#include <vector>
#include <Eigen/Core>
#include <boost/optional.hpp>

#define DEFINE_RESULT_TYPES(Scalar, Suffix) \
  typedef IcpResults_<Scalar> IcpResults##Suffix;

namespace icp
{

/**
 * @brief Results for the ICP
 */
template<typename Dtype>
struct IcpResults_ {
  //! History of previous registration errors
  /*!
    - First value is the initial error before ICP,
    - Last value is the final error after ICP. */
  std::vector<Dtype> registrationError;

  //! Transformation (SE3) of the final registration transformation
  Eigen::Matrix<Dtype, 4, 4> transformation;
  Eigen::Matrix<Dtype, 4, 4> relativeTransformation;

  // Scale for Sim3 icp
  Dtype scale;

  // True if ICP has converged
  bool has_converged;

  IcpResults_() : transformation(Eigen::Matrix<Dtype, 4, 4>::Identity()),
    relativeTransformation(Eigen::Matrix<Dtype, 4, 4>::Identity()),
    scale(1.),
    has_converged(false) {
  }

  boost::optional<Dtype> getLastErrorVariation() const {
    if (registrationError.size() >= 2) {
      return registrationError[registrationError.size() - 1] - registrationError[registrationError.size() - 2];
    } else {
      return boost::none;
    }
  }

  boost::optional<Dtype> getLastError() const {
    if (registrationError.size() > 0) {
      return registrationError[registrationError.size() - 1];
    } else {
      return boost::none;
    }
  }

  void clear() {
    registrationError.clear();
    transformation = Eigen::Matrix<Dtype, 4, 4>::Identity();
  }
};

template<typename Dtype>
std::ostream &operator<<(std::ostream &s, const IcpResults_<Dtype> &r) {
  if (!r.registrationError.empty()) {
    s << "Initial error: " << r.registrationError[0]
      << "\nFinal error: " << r.registrationError[r.registrationError.size() - 1]
      << "\nFinal transformation: \n"
      << r.transformation
      << "\nRelative transformation: \n"
      << r.relativeTransformation
      << "\nScale factor: " << r.scale
      << "\nError history: ";
    for (int i = 0; i < r.registrationError.size(); ++i) {
      s << r.registrationError[i]  << ", ";
    }
  } else {
    s << "Icp: No Results!";
  }
  return s;
}


DEFINE_RESULT_TYPES(float, )
DEFINE_RESULT_TYPES(float, f)

}  // namespace icp

#endif
