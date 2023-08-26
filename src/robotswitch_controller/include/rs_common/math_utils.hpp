#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

namespace RobotSwitch{
  const static double kEpsilon = 1e-7;
  typedef Eigen::Matrix<double, 2, 1> Vector2;
  typedef Eigen::Matrix<double, 3, 1> Vector3;
  typedef Eigen::Matrix<double, 4, 1> Vector4;
  typedef Eigen::Matrix<double, 6, 1> Vector6;
  typedef Eigen::Matrix<double, 7, 1> Vector7;

  typedef Eigen::Matrix<double, 3, 3> Matrix3;
  typedef Eigen::Matrix<double, 4, 4> Matrix4;
  typedef Eigen::Matrix<double, 6, 6> Matrix6;
  typedef Eigen::Matrix<double, 4, 6> Matrix46;
  typedef Eigen::Matrix<double, 6, 4> Matrix64;
  typedef Eigen::MatrixXd MatrixXd;


  typedef Eigen::Quaterniond Quaterniond;

  Quaterniond vec42quat(const Vector4 &v);
  Vector4     quat2vec4(const Quaterniond &q);
  Matrix3     quat2SO3(const Quaterniond &q);
  Matrix3    quat2SO3(double qw, double qx, double qy, double qz);

  Matrix3     wedge(const Vector3 &v);

  Quaterniond QuatBTVec(const Vector3 &v1, const Vector3 &v2);
  Quaterniond QuatMTimes(const Quaterniond &q1, const Quaterniond &q2);
  Quaterniond axisangle2quat(double theta, const Vector3 &v);

  template <int N>
  using Vec = Eigen::Matrix<double, N, 1>;

  template <int M, int N>
  using Mat = Eigen::Matrix<double, M, N>;

  Matrix4 posem2SE3(const double *pose);
  Matrix4 posemm2SE3(const double *pose);
  Matrix4 SE3Inv(const Matrix4 &SE3);
  Vector6 SE32spt(const Matrix4 &SE3);
  Matrix6 SE32Adj(const Matrix4 &SE3);
  Matrix4 spt2SE3(const Vector6 &spt);
  Matrix3 so32SO3(const Vector3 &v);
  Vector3 SO32so3(const Matrix3 &R);
  void SO32quat(const Matrix3 &SO3, double *q);

  Matrix6 JacobianSpt2BodyV(const Matrix3 &R);
  MatrixXd pseudoInverse(const MatrixXd &a, double epsilon = std::numeric_limits<double>::epsilon());

  void copyArray(const float *src, float *dest, int dim);
  void copyArray(const double *src, double *dest, int dim);

  void stream_array_in(std::ostream &st, double *array, int length);
  void stream_array_in(std::ostream &st, float *array, int length);
  void stream_array_in(std::ostream &st, int *array, int length);
  void stream_array_in(std::ostream &st, const std::vector<double> &vec, int length);
  void stream_array_in(std::ostream &st, const Eigen::VectorXd &vec, int length);
  void stream_array_in6d(std::ostream &st, const Vector6 &array);
  void stream_array_in3d(std::ostream &st, const Vector3 &array);

  double Gaussian(double x, double var);
  void truncate(double *ele, const double _min, const double _max);
  void truncate6d(Vector6 *v, double min, double max);
  void truncate6d(Vector6 *v, const Vector6 &min, const Vector6 &max);

  Matrix4 wedge6(const Vector6 &t);
  void SE32Posem(const Matrix4 &SE3, double *pose);
  void SE32Posemm(const Matrix4 &SE3, double *pose);

    /**
     * Linear interpolation between @p pose0 and @p pose_set.
     *
     * @param[in]  pose0      The initial pose. [x y z qw qx qy qz]
     * @param[in]  pose_set   The goal pose
     * @param[in]  Nsteps     The number of time steps
     * @param      pose_traj  The interpolated pose traj. 7 x Nsteps matrix
     */
    void MotionPlanningLinear(const double *pose0, const double *pose_set, const int Nsteps, MatrixXd *pose_traj);
}

#endif