#ifndef HEIGENLIB_H
#define HEIGENLIB_H

#define EIGEN_MAX_STATIC_ALIGN_BYTES 0
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3i)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3i)

#endif // HEIGENLIB_H