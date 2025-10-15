
#pragma once

#include <Eigen/Dense>
#include <cmath>

// TODO (BK): remove these once we have implemented eigen in motion
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/lu.hpp>

/* Creates a matrix that Rotate vector about the z axis
 * Currently needed for our robot relative coordinate system
 */
template <typename T>
Eigen::Matrix<T, 4, 4> rotateZMatrix_(T theta);

/* Creates a translation matrix
 */
template <typename T>
Eigen::Matrix<T, 4, 4> translateMatrix_(T x, T y, T z);

/* Creates a projection matrix
 */
template <typename T>
Eigen::Matrix<T, 4, 4> projectionMatrix_(T ex, T ey, T ez);


/* Creates a DH matrix used in Kinematics
 */
template <typename T>
Eigen::Matrix<T, 4, 4> createDHMatrix_(T a, T alpha, T d, T theta);

/* Function to invert matrix
 */
template <typename T, int Rows, int Cols>
bool invertMatrix_(const Eigen::Matrix<T, Rows, Cols>& input,
                  Eigen::Matrix<T, Rows, Cols>& inverse);

/* Creates a 4,1 vector */
template <typename T>
inline Eigen::Matrix<T, 4, 1> vec4(T a, T b, T c, T d) {
   Eigen::Matrix<T, 4, 1> m;
   m << a, b, c, d;
   return m;
}

template <typename T>
inline Eigen::Matrix<T, 4, 1> vec4(const T a[]) {
   Eigen::Matrix<T, 4, 1> m;
   m << a[0], a[1], a[2], a[3];
   return m;
}

#include "utils/math/matrix_helpers_eigen.tcc"
