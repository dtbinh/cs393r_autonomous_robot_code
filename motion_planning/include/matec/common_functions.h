#ifndef COMMON_FUNCTIONS_H
#define COMMON_FUNCTIONS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

#define WTFASSERT(assertion) if(!(assertion)){std::cerr << "ASSERTION FAILED AT LINE " << __LINE__ << " OF FILE " << __FILE__ << std::endl; std::stringstream ss; ss << "kill -9 " << getpid(); int stupid = system(ss.str().c_str()); stupid=stupid;}

namespace dynamics_tree
{
  typedef Eigen::Matrix<double, 2, 1> Vector2;
  typedef Eigen::Matrix<double, 3, 1> Vector3;
  typedef Eigen::Matrix<double, 4, 1> Vector4;
  typedef Eigen::Matrix<double, 6, 1> Vector6;
  typedef Eigen::Matrix<double, 10, 1> Vector10;
  typedef Eigen::Matrix<double, 11, 1> Vector11;
  typedef Eigen::Matrix<double, 3, 3> Matrix3;
  typedef Eigen::Matrix<double, 4, 4> Matrix4;
  typedef Eigen::Matrix<double, 6, 6> Matrix6;
  typedef Eigen::Matrix<double, 6, 10> Matrix610;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrix;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;

  template<typename V> //V must be an eigen vector type
  inline V oneAugmentedZero() //todo: come up with a less silly name
  {
    V vector = V::Zero();
    vector(vector.rows() - 1) = 1.0;
    return vector;
  }

  inline double fixAngle(double angle)
  {
    return (angle > M_PI)? (angle - 2.0 * M_PI) : ((angle < -M_PI)? (angle + 2.0 * M_PI) : angle);
  }

  inline double clamp(double a, double b, double c)
  {
    return ((b) < (a)? (c) < (b)? (b) : (a) < (c)? (a) : (c) : (c) < (a)? (a) : (b) < (c)? (b) : (c));
  }

  template<typename V>
  inline V vectorClamp(V a, V b, V c)
  {
    V clamped;

    for(unsigned int i = 0; i < clamped.rows(); i++)
    {
      clamped(i) = clamp(a(i), b(i), c(i));
    }

    return clamped;
  }

  inline double sign(double val)
  {
    return (val == 0.0)? 0.0 : ((val < 0)? -1.0 : 1.0);
  }

  inline double sat(double val, double boundary_layer = 1e-3)
  {
    return (boundary_layer == 0)? sign(val) : clamp(val / boundary_layer, -1.0, 1.0);
  }

  inline double max3(double a, double b, double c)
  {
    return std::max(a, std::max(b, c));
  }

  inline double min3(double a, double b, double c)
  {
    return std::min(a, std::min(b, c));
  }

  inline double angleAverage(double a1, double a2)
  {
    return atan2((sin(a1) + sin(a2)) / 2.0, (cos(a1) + cos(a2)) / 2.0);
  }

  inline double angleAdd(double a1, double a2) //returns a1 + a2, wrapped
  {
    return atan2(sin(a1 + a2), cos(a1 + a2));
  }

  inline double angleSub(double a1, double a2) //returns a1 - a2, wrapped
  {
    return atan2(sin(a1 - a2), cos(a1 - a2));
  }

  //returns a real value between 0 and 1
  inline long double uniformRandom()
  {
    return ((long double) rand()) / ((long double) RAND_MAX);
  }

  //returns a real value between -1 and 1
  inline long double uniformRandomCentered()
  {
    return 2 * ((long double) (rand() - (((long double) RAND_MAX) / 2.0))) / ((long double) RAND_MAX);
  }

  inline std::pair<long double, long double> uniformRandomCircle()
  {
    std::pair<long double, long double> coordinates;
    long double t = 2 * M_PI * uniformRandom();
    long double u = uniformRandom() + uniformRandom();
    long double r = u > 1? 2 - u : u;
    coordinates.first = r * cos(t);
    coordinates.second = r * sin(t);
    return coordinates;
  }

  //box-muller transform method
#define NORMAL_RANDOM_USE_BASIC_FORM
  inline long double normalRandom(long double mean, long double stddev)
  {
    long double z0;
    static long double z1 = 0.0;
    static bool z1_cached = 0;
    if(!z1_cached)
    {
#ifdef NORMAL_RANDOM_USE_BASIC_FORM
      //normal form
      long double r = sqrt(-2.0 * log(uniformRandom()));
      long double t = 2 * M_PI * uniformRandom();
      z0 = r * cos(t);
      z1 = r * sin(t);

#else
      //polar form
      long double x = uniformRandomCentered();
      long double y = uniformRandomCentered();
      long double r = x * x + y * y;
      while(r == 0.0 || r > 1.0)
      {
        x = uniformRandomCentered();
        y = uniformRandomCentered();
        r = x * x + y * y;
      }

      long double d = sqrt(-2.0 * log(r) / r);
      z0 = x * d;
      z1 = y * d;
#endif
//      double result = z0 * stddev + mean;
      z1_cached = true;
      return z0 * stddev + mean;
    }
    else
    {
      z1_cached = false;
      return z1 * stddev + mean;
    }
  }

  inline Matrix3 cross(Vector3 v)
  {
    Matrix3 m;
    m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return m;
  }

  inline Matrix6 cross(Vector6 sv)
  {
    Matrix6 m;

    Vector3 w = sv.topRows(3);
    Vector3 v = sv.bottomRows(3);
    m.topLeftCorner(3, 3) = cross(w);
    m.bottomLeftCorner(3, 3) = cross(v);
    m.topRightCorner(3, 3) = Matrix3::Zero();
    m.bottomRightCorner(3, 3) = m.topLeftCorner(3, 3);

    return m;
  }

  inline Matrix6 constructInertia(double m, Vector3 r, Matrix3 Ic)
  {
    Matrix6 inertia;

    inertia.topLeftCorner(3, 3) = Ic - m * cross(r) * cross(r);
    inertia.topRightCorner(3, 3) = m * cross(r);
    inertia.bottomLeftCorner(3, 3) = -m * cross(r);
    inertia.bottomRightCorner(3, 3) = m * Matrix3::Identity(3, 3);

    return inertia;
  }

  inline Matrix4 pureTranslation(double dx, double dy, double dz)
  {
    Matrix4 mat = Matrix4::Identity();
    mat.topRightCorner(3, 1) << dx, dy, dz;
    return mat;
  }

  inline Matrix4 pureRotation(Eigen::Quaterniond quat)
  {
    Matrix4 mat = Matrix4::Identity();
    mat.topLeftCorner(3, 3) = quat.toRotationMatrix();
    return mat;
  }

  inline Matrix3 rotationMatrix(double roll, double pitch, double yaw)
  {
    double ci = cos(roll);
    double cj = cos(pitch);
    double ch = cos(yaw);
    double si = sin(roll);
    double sj = sin(pitch);
    double sh = sin(yaw);
    double cc = ci * ch;
    double cs = ci * sh;
    double sc = si * ch;
    double ss = si * sh;
    Matrix3 mat;
    mat << cj * ch, sj * sc - cs, sj * cc + ss, cj * sh, sj * ss + cc, sj * cs - sc, -sj, cj * si, cj * ci;
    return mat;
  }

  inline Matrix4 pureRotation(double roll, double pitch, double yaw)
  {
    Matrix4 mat = Matrix4::Identity();
    mat.topLeftCorner(3, 3) = rotationMatrix(roll, pitch, yaw);
    return mat;
  }

  inline Vector stdVectorToEigenVector(std::vector<double> std_vector)
  {
    Vector eigen_vector = Vector(std_vector.size());
    for(unsigned int i = 0; i < std_vector.size(); i++)
    {
      eigen_vector(i) = std_vector.at(i);
    }
    return eigen_vector;
  }

  inline std::vector<double> eigenVectorToStdVector(Vector eigen_vector)
  {
    std::vector<double> std_vector(eigen_vector.rows());
    for(unsigned int i = 0; i < std_vector.size(); i++)
    {
      std_vector.at(i) = eigen_vector(i);
    }
    return std_vector;
  }

  inline dynamics_tree::Matrix6 motionTransformFromAffine(dynamics_tree::Matrix4 affine)
  {
    dynamics_tree::Matrix3 E = affine.topLeftCorner(3, 3);
    dynamics_tree::Vector3 r = -E.inverse() * affine.topRightCorner(3, 1);

    dynamics_tree::Matrix6 motion = dynamics_tree::Matrix6::Identity();
    motion.topLeftCorner(3, 3) = E;
    // motion.topRightCorner(3, 3) = Matrix3::Zero();
    motion.bottomLeftCorner(3, 3) = -E * dynamics_tree::cross(r);
    motion.bottomRightCorner(3, 3) = E;

    return motion;
  }

  inline dynamics_tree::Matrix6 transformInertia(dynamics_tree::Matrix4 iTj, dynamics_tree::Matrix6 inertia_j)
  {
    dynamics_tree::Matrix6 jXi = motionTransformFromAffine(iTj).inverse(); //TODO: make sure we're going the right direction
    return jXi.transpose() * inertia_j * jXi;
  }

  inline void matrixToRPY(dynamics_tree::Matrix3 rot, double&roll, double&pitch, double&yaw)
  {
    double epsilon = 1E-12;
    pitch = atan2((double) -rot(2, 0), sqrt((double) (rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0))));
    if(fabs(pitch) > (M_PI / 2.0 - epsilon))
    {
      yaw = atan2((double) -rot(0, 1), (double) rot(1, 1));
      roll = 0.0;
    }
    else
    {
      roll = atan2((double) rot(2, 1), (double) rot(2, 2));
      yaw = atan2((double) rot(1, 0), (double) rot(0, 0));
    }
  }

  inline void poseToRPYxyz(dynamics_tree::Matrix4& pose, dynamics_tree::Vector6& rpyxyz)
  {
    dynamics_tree::Vector3 r = pose.topRightCorner(3, 1);
    dynamics_tree::Matrix3 E = pose.topLeftCorner(3, 3);

//    Eigen::AngleAxis<double> angle_axis(E);
//    double R = fixAngle(angle_axis.angle() * angle_axis.axis().x());
//    double P = fixAngle(angle_axis.angle() * angle_axis.axis().y());
//    double Y = fixAngle(angle_axis.angle() * angle_axis.axis().z());

    double R, P, Y;
    matrixToRPY(E, R, P, Y);

    rpyxyz << R, P, Y, r(0), r(1), r(2);
  }

  inline void extractRPY(Matrix3 rot, double& roll, double& pitch, double& yaw)
  {
    Vector3 backwards = rot.eulerAngles(2, 1, 0);
    roll = backwards(2);
    pitch = backwards(1);
    yaw = backwards(0);
  }

  //assumes that f1 and f2 are expressed w.r.t. the same frame (common)
  //returns a twist in the common frame that takes you from f1 to f2 in dt seconds
  inline dynamics_tree::Vector6 frameTwist(dynamics_tree::Matrix4 commonTf1, dynamics_tree::Matrix4 commonTf2, double dt, dynamics_tree::Vector6 weights = dynamics_tree::Vector6::Ones())
  {
    dynamics_tree::Matrix4 delta = commonTf1.inverse() * commonTf2;
    Eigen::AngleAxis<double> angle_axis((dynamics_tree::Matrix3) delta.topLeftCorner(3, 3));

    dynamics_tree::Vector6 twist; //expressed here in f1 coordinates
//    twist(0) = weights(0) * fixAngle(angle_axis.angle() * angle_axis.axis().x()) / dt;
//    twist(1) = weights(1) * fixAngle(angle_axis.angle() * angle_axis.axis().y()) / dt;
//    twist(2) = weights(2) * fixAngle(angle_axis.angle() * angle_axis.axis().z()) / dt;
    double R, P, Y;
    matrixToRPY((dynamics_tree::Matrix3) delta.topLeftCorner(3, 3), R, P, Y);
    twist(0) = weights(0) * R / dt;
    twist(1) = weights(1) * P / dt;
    twist(2) = weights(2) * Y / dt;
    twist(3) = weights(3) * delta(0, 3) / dt;
    twist(4) = weights(4) * delta(1, 3) / dt;
    twist(5) = weights(5) * delta(2, 3) / dt;

    //transform back to common
    twist.topRows(3) = commonTf1.topLeftCorner(3, 3) * twist.topRows(3);
    twist.bottomRows(3) = commonTf1.topLeftCorner(3, 3) * twist.bottomRows(3);

    return twist;
  }

  template<typename T>
  void printVector(std::vector<T> vector)
  {
    std::cerr << "[";
    for(unsigned int i = 0; i < vector.size(); i++)
    {
      if(i != 0)
      {
        std::cerr << ", ";
      }
      std::cerr << vector[i];
    }
    std::cerr << "]";
  }
}
;
#endif //COMMON_FUNCTIONS_H
