#include <rs_common/math_utils.hpp>
namespace RobotSwitch{
const static double _PI = 3.14159265;


Quaterniond QuatMTimes(const Quaterniond &q1,
    const Quaterniond &q2)  {
  double s1 = q1.w();
  Vector3 v1(q1.x(), q1.y(), q1.z());

  double s2 = q2.w();
  Vector3 v2(q2.x(), q2.y(), q2.z());

  double cr_v1 = v1(1)*v2(2) - v1(2)*v2(1);
  double cr_v2 = v1(2)*v2(0) - v1(0)*v2(2);
  double cr_v3 = v1(0)*v2(1) - v1(1)*v2(0);

  Quaterniond qp;
  qp.w() = s1*s2 - v2.dot(v1);
  qp.x() = v2(0)*s1 + s2*v1(0) + cr_v1;
  qp.y() = v2(1)*s1 + s2*v1(1) + cr_v2;
  qp.z() = v2(2)*s1 + s2*v1(2) + cr_v3;

  return qp;
}

Quaterniond vec42quat(const Vector4 &v)
{
  return Quaterniond(v(0), v(1), v(2), v(3));
}
Vector4     quat2vec4(const Quaterniond &q)
{
  return Vector4(q.w(), q.x(), q.y(), q.z());
}
Matrix3 quat2SO3(const Quaterniond &q)
{
  return q.normalized().toRotationMatrix();
}

Matrix3 quat2SO3(double qw, double qx, double qy, double qz) {
  Quaterniond q(qw, qx, qy, qz);
  return q.normalized().toRotationMatrix();
}

Matrix3     wedge(const Vector3 &v)
{
  Matrix3 w;
          w<<  0, -v(2),   v(1),
            v(2),     0,  -v(0),
           -v(1),  v(0),      0;
  return w;
}

Quaterniond QuatBTVec(const Vector3 &v1, const Vector3 &v2)
{
  double mid;
  Quaterniond qtn;
  Vector3 v1_norm = v1.normalized();
  Vector3 v2_norm = v2.normalized();
  Vector3 x_unitVec3(1,0,0);
  Vector3 y_unitVec3(0,1,0);
  Vector3 tmp;

  mid = v1_norm.transpose() * v2_norm;
  if(mid < -0.999999)
  {
    tmp = x_unitVec3.cross(v1_norm);
    if(tmp.norm() < 0.000001)
      tmp = y_unitVec3.cross(v1_norm);
    tmp.normalize();
    qtn = axisangle2quat(_PI, tmp);
  }
  else if(mid > 0.999999)
  {
    qtn = Quaterniond(1,0,0,0);
  }
  else
  {
    tmp = v1_norm.cross(v2_norm);
    qtn.w() = 1+mid;
    qtn.x() = tmp(0);
    qtn.y() = tmp(1);
    qtn.z() = tmp(2);
    qtn.normalize();
  }
  return qtn;
}

Quaterniond axisangle2quat(double theta, const Vector3 &v)
{
  Vector3 n;
  Quaterniond qtn;
  n = v.normalized();
  qtn.w() = cos(theta/2); 
  qtn.x() = n(0)*sin(theta/2);
  qtn.y() = n(1)*sin(theta/2);
  qtn.z() = n(2)*sin(theta/2);
  return qtn;
}

/////////////////////////////////////////////////////////////////////////
  //                          iostream
  /////////////////////////////////////////////////////////////////////////
  void stream_array_in(std::ostream &st, double *array, int length) {
    for (int i = 0; i<length; i++) {
      st << array[i] << "\t";
   }
 }

 void stream_array_in(std::ostream &st, float *array, int length) {
  for (int i = 0; i<length; i++) {
   st << array[i] << "\t";
 }    
}

void stream_array_in(std::ostream &st, int *array, int length) {
  for (int i = 0; i<length; i++) {
   st << array[i] << "\t";
 }    
}

void stream_array_in(std::ostream &st, const std::vector<double> &vec, int length) {
  for (int i = 0; i < length; ++i) {
    st << vec[i] << "\t";
  }    
}

void stream_array_in(std::ostream &st, const Eigen::VectorXd &vec, int length) {
  for (int i = 0; i < length; ++i) {
    st << vec[i] << "\t";
  }
}
void stream_array_in6d(std::ostream &st, const Vector6 &array)
{
  for (int i = 0; i<6; i++)
  {
    st << array(i);
    st << "\t";
  }
}

void stream_array_in3d(std::ostream &st, const Vector3 &array)
{
  for (int i = 0; i<3; i++)
  {
    st << array(i);
    st << "\t";
  }
}

void copyArray(const float *src, float *dest, int dim)
{
  for(int i = 0; i<dim; i++)
  {
    dest[i] = src[i];
  }
}

void copyArray(const double *src, double *dest, int dim)
{
  for(int i = 0; i<dim; i++)
  {
    dest[i] = src[i];
  }
}



Matrix4 posem2SE3(const double *pose) {
  Matrix4 SE3 = Matrix4::Identity();
  SE3(0, 3) = pose[0];
  SE3(1, 3) = pose[1];
  SE3(2, 3) = pose[2];
  SE3.block<3,3>(0,0) = quat2SO3(pose[3], pose[4], pose[5], pose[6]);
  return SE3;
}

Matrix4 posemm2SE3(const double *pose) {
  Matrix4 SE3 = Matrix4::Identity();
  SE3(0, 3) = pose[0]/1000.0;
  SE3(1, 3) = pose[1]/1000.0;
  SE3(2, 3) = pose[2]/1000.0;
  SE3.block<3,3>(0,0) = quat2SO3(pose[3], pose[4], pose[5], pose[6]);
  return SE3;
}

Matrix4 SE3Inv(const Matrix4 &SE3) {
  Matrix4 SE3_inv = Matrix4::Identity();
  SE3_inv.block<3,1>(0, 3) =
      -SE3.block<3,3>(0,0).transpose()*SE3.block<3,1>(0,3);
  SE3_inv.block<3,3>(0,0) = SE3.block<3,3>(0,0).transpose();
  return SE3_inv;
}

Matrix6 SE32Adj(const Matrix4 &SE3) {
  Matrix6 Adj = Matrix6::Zero();
  Adj.topLeftCorner(3, 3)     = SE3.topLeftCorner(3, 3);
  Adj.bottomRightCorner(3, 3) = SE3.topLeftCorner(3, 3);
  Adj.topRightCorner(3, 3)    =
      wedge(SE3.block<3,1>(0, 3)) * SE3.topLeftCorner(3, 3);
  return Adj;
}

Matrix4 spt2SE3(const Vector6 &spt) {
  Matrix4 SE3 = Matrix4::Identity();
  SE3.block<3, 3>(0, 0) = so32SO3(spt.tail(3));
  SE3.block<3, 1>(0, 3) = spt.head(3);
  return SE3;
}

Matrix3 so32SO3(const Vector3 &v) {
  double theta = v.norm();
  if (theta > kEpsilon) {
    Vector3 vn = v/theta;
    Matrix3 v_wedge = wedge(vn);
    Matrix3 SO3;
    SO3 = Matrix3::Identity() + v_wedge*sin(theta) +
      v_wedge*v_wedge*(1.0 - cos(theta));
    return SO3;
  } else {
    return Matrix3::Identity();
  }
}

Vector3 SO32so3(const Matrix3 &R) {
  Vector3 so3;
  double temp_arg_to_cos = (R.trace() - 1.0)/2.0;
  truncate(&temp_arg_to_cos, -1.0, 1.0);
  double theta = acos(temp_arg_to_cos);
  if(fabs(theta) < kEpsilon) {
    so3(0) = 1.0;
    so3(1) = 0.0;
    so3(2) = 0.0;
  } else {
    so3(0) = R(2,1)-R(1,2);
    so3(1) = R(0,2)-R(2,0);
    so3(2) = R(1,0)-R(0,1);
    so3 /= 2.0*sin(theta);
  }
  so3 *= theta;
  return so3;
}

void SO32quat(const Matrix3 &SO3, double *q) {
  // Eigen implementation assumes input matrix is strictly orthogonal and is not
  // numerically stable sometimes.
   Quaterniond q_eigen(SO3);
   q_eigen.normalize();
   q[0] = q_eigen.w();
   q[1] = q_eigen.x();
   q[2] = q_eigen.y();
   q[3] = q_eigen.z();
}
// Return the 6x6 jacobian matrix mapping from spt time derivative
//  to body velocity.
// Jac * spt time derivative = body velocity
Matrix6 JacobianSpt2BodyV(const Matrix3 &R) {
  Matrix6 Jac;
  Jac = Matrix6::Identity();
  Jac(3, 3) = R(0,2)*R(0,2) + R(1,2)*R(1,2) + R(2,2)*R(2,2);
  Jac(3, 5) = -R(0,0)*R(0,2) - R(1,0)*R(1,2) - R(2,0)*R(2,2);
  Jac(4, 3) = -R(0,0)*R(0,1) - R(1,0)*R(1,1) - R(2,0)*R(2,1);
  Jac(4, 4) = R(0,0)*R(0,0) + R(1,0)*R(1,0) + R(2,0)*R(2,0);
  Jac(5, 4) = -R(0,1)*R(0,2) - R(1,1)*R(1,2) - R(2,1)*R(2,2);
  Jac(5, 5) = R(0,1)*R(0,1) + R(1,1)*R(1,1) + R(2,1)*R(2,1);
  return Jac;
}

double Gaussian(double x, double var) {
  double k = 1.0/var/std::sqrt(2.0*_PI);
  double exp = std::exp(-x*x/2.0/var/var);
  return k*exp;
}

Vector6 SE32spt(const Matrix4 &SE3) {
  Vector6 spt;
  spt.head(3) = SE3.block<3, 1>(0, 3);
  spt.tail(3) = SO32so3(SE3.block<3, 3>(0, 0));
  return spt;
}

void truncate(double *ele, const double _min, const double _max)
{
  if ( (*ele) > _max)
    (*ele) = _max;
  else if ( (*ele) < _min)
    (*ele) = _min;
}

void truncate6d(Vector6 *v, double min, double max)
{
  for(int i=0; i<6; i++)
  {
    (*v)[i] = ((*v)[i] > max)? max:(*v)[i];
    (*v)[i] = ((*v)[i] < min)? min:(*v)[i];
  }
}

void truncate6d(Vector6 *v, const Vector6 &min, const Vector6 &max)
{
  for(int i=0; i<6; i++)
  {
    (*v)[i] = ((*v)[i] > max[i])? max[i]:(*v)[i];
    (*v)[i] = ((*v)[i] < min[i])? min[i]:(*v)[i];
  }
}

Matrix4 wedge6(const Vector6 &t) {
  Matrix4 t_wedge;
  t_wedge <<    0,   -t(5),    t(4),   t(0),
             t(5),       0,   -t(3),   t(1),
            -t(4),    t(3),       0,   t(2),
                0,       0,       0,      0;
  return t_wedge;
}

void SE32Posem(const Matrix4 &SE3, double *pose) {
  pose[0] = SE3(0, 3);
  pose[1] = SE3(1, 3);
  pose[2] = SE3(2, 3);
  SO32quat(SE3.block<3,3>(0,0), pose + 3);
}

void SE32Posemm(const Matrix4 &SE3, double *pose) {
  pose[0] = SE3(0, 3)*1000.0;
  pose[1] = SE3(1, 3)*1000.0;
  pose[2] = SE3(2, 3)*1000.0;
  SO32quat(SE3.block<3,3>(0,0), pose + 3);
}

MatrixXd pseudoInverse(const MatrixXd &a,
    double epsilon) {
  if (a.norm() < epsilon) {
    return  MatrixXd::Zero(a.cols(), a.rows());
  } else {
    Eigen::JacobiSVD< MatrixXd > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
  }
}

/////////////////////////////////////////////////////////////////////////
//                      Motion Planning
/////////////////////////////////////////////////////////////////////////
void MotionPlanningLinear(const double *pose0, const double *pose_set, const int Nsteps,
    MatrixXd *pose_traj) {
  Quaterniond q0(pose0[3], pose0[4], pose0[5],pose0[6]);
  Quaterniond qset(pose_set[3], pose_set[4], pose_set[5],pose_set[6]);
  Quaterniond q;

  pose_traj->resize(7, Nsteps);
  for (int i = 0; i < Nsteps; ++i) {
    q = q0.slerp(double(i+1)/double(Nsteps), qset);
    (*pose_traj)(0, i) = (pose0[0]*double(Nsteps-i-1) + pose_set[0]*double(i+1))/double(Nsteps);
    (*pose_traj)(1, i) = (pose0[1]*double(Nsteps-i-1) + pose_set[1]*double(i+1))/double(Nsteps);
    (*pose_traj)(2, i) = (pose0[2]*double(Nsteps-i-1) + pose_set[2]*double(i+1))/double(Nsteps);
    (*pose_traj)(3, i) = q.w();
    (*pose_traj)(4, i) = q.x();
    (*pose_traj)(5, i) = q.y();
    (*pose_traj)(6, i) = q.z();
  }
}


Vector3 Euler2OmegaSVel(const Vector3& eul, const Vector3& eul_vel) {
  double sa = sin(eul[0]);
  double ca = cos(eul[0]);
  double sb = sin(eul[1]);
  double cb = cos(eul[1]);

  Matrix3 e2o_vel;
  e2o_vel << 0, -sa, ca * cb, 0, ca, sa * cb, 1, 0, -sb;

  Vector3 omega_vel = e2o_vel * eul_vel;

  return omega_vel;
}

Vector3 Euler2OmegaSAcc(const Vector3& eul, const Vector3& eul_vel,
                        const Vector3& eul_acc) {
  double sa = sin(eul[0]);
  double ca = cos(eul[0]);
  double sb = sin(eul[1]);
  double cb = cos(eul[1]);

  double va = eul_vel[0];
  double vb = eul_vel[1];

  Matrix3 e2o_vel;
  e2o_vel << 0, -sa, ca * cb, 0, ca, sa * cb, 1, 0, -sb;
  Matrix3 e2o_acc;
  e2o_acc << 0, -ca * va, -sa * cb * va - ca * sb * vb, 0, -sa * va,
      ca * cb * va - sa * cb * vb, 0, 0, -cb * vb;

  Vector3 omega_acc = e2o_acc * eul_vel + e2o_vel * eul_acc;

  return omega_acc;
}

Vector3 OmegaS2EulerVel(const Vector3& eul, const Vector3& omega_vel) {
  double sa = sin(eul[0]);
  double ca = cos(eul[0]);
  double sb = sin(eul[1]);
  double cb = cos(eul[1]);

  double det = sa * sa * cb + ca * ca * cb;

  Matrix3 o2e_vel;
  o2e_vel << sb * ca / det, sa * sb / det, 1, -sa * cb / det, ca * cb / det, 0,
      ca / det, sa / det, 0;

  Vector3 eul_vel = o2e_vel * omega_vel;

  return eul_vel;
}

  Vector3 OmegaS2EulerAcc(const Vector3& eul, const Vector3& eul_vel,
                          const Vector3& omega_acc) {
    double sa = sin(eul[0]);
    double ca = cos(eul[0]);
    double sb = sin(eul[1]);
    double cb = cos(eul[1]);

    double va = eul_vel[0];
    double vb = eul_vel[1];

    double det = sa * sa * cb + ca * ca * cb;

    Matrix3 o2e_vel;
    o2e_vel << sb * ca / det, sa * sb / det, 1, -sa * cb / det, ca * cb / det, 0,
        ca / det, sa / det, 0;
    Matrix3 e2o_acc;
    e2o_acc << 0, -ca * va, -sa * cb * va - ca * sb * vb, 0, -sa * va,
        ca * cb * va - sa * cb * vb, 0, 0, -cb * vb;

    Vector3 eul_acc = o2e_vel * (omega_acc - e2o_acc * eul_vel);

    return eul_acc;
  }

  Matrix3 quat2SO3_Matlab(const Eigen::Quaterniond& q) {
      Eigen::Matrix3d R;

      R(0, 0) = 2.0 * q.w() * q.w() - 1.0 + 2.0 * q.x() * q.x();
      R(0, 1) = 2.0 * (q.x() * q.y() + q.w() * q.z());
      R(0, 2) = 2.0 * (q.x() * q.z() - q.w() * q.y());
      R(1, 0) = 2.0 * (q.x() * q.y() - q.w() * q.z());
      R(1, 1) = 2.0 * q.w() * q.w() - 1.0 + 2.0 * q.y() * q.y();
      R(1, 2) = 2.0 * (q.y() * q.z() + q.w() * q.x());
      R(2, 0) = 2.0 * (q.x() * q.z() + q.w() * q.y());
      R(2, 1) = 2.0 * (q.y() * q.z() - q.w() * q.x());
      R(2, 2) = 2.0 * q.w() * q.w() - 1.0 + 2.0 * q.z() * q.z();
      return R;
  }

}