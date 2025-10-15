
template <typename T>
Eigen::Matrix<T, 4, 4> translateMatrix(T x, T y, T z) {
    Eigen::Matrix<T, 4, 4> r;
    r << 1, 0, 0, x,
         0, 1, 0, y,
         0, 0, 1, z,
         0, 0, 0, 1;
    return r;
}

template <typename T>
Eigen::Matrix<T, 4, 4> rotateZMatrix(T theta) {
    Eigen::Matrix<T, 4, 4> r;
    r << cos(theta), -sin(theta), 0, 0,
         sin(theta),  cos(theta), 0, 0,
         0,           0,          1, 0,
         0,           0,          0, 1;
    return r;
}

template <typename T>
Eigen::Matrix<T, 4, 4> rotateXMatrix(T theta) {
    Eigen::Matrix<T, 4, 4> r;
    r << 1, 0,           0,            0,
         0, cos(theta), -sin(theta), 0,
         0, sin(theta),  cos(theta), 0,
         0, 0,           0,            1;
    return r;
}

template <typename T>
Eigen::Matrix<T, 4, 4> rotateYMatrix(T theta) {
    Eigen::Matrix<T, 4, 4> r;
    r << cos(theta), 0, sin(theta), 0,
         0,          1, 0,          0,
         -sin(theta), 0, cos(theta), 0,
         0,          0, 0,          1;
    return r;
}

template <typename T>
Eigen::Matrix<T, 4, 4> createDHMatrix(T a, T alpha, T d, T theta) {
    Eigen::Matrix<T, 4, 4> m;
    m << cos(theta),             -sin(theta),            0,              a,
         sin(theta) * cos(alpha),  cos(theta) * cos(alpha), -sin(alpha),    -sin(alpha) * d,
         sin(theta) * sin(alpha),  cos(theta) * sin(alpha),  cos(alpha),     cos(alpha) * d,
         0,                        0,                      0,              1;
    return m;
}