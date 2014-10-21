#ifndef EIGEN_TOOLS_H
#define EIGEN_TOOLS_H

#include <Eigen/Geometry>

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


#endif /* EIGEN_TOOLS_H */
