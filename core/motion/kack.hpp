#ifndef KACK_HPP
#define KACK_HPP

namespace KACK
{
  class Point
  {
    double x;
    double y;
    double z;

    void update(double a, double b, double c)
    {
      x = a;
      y = b;
      z = c;
    }
  };

  class Pose
  {
    //int Kick_foot_state; //0:double stand; 1:Right; 2: Left;
    double x;
    double y;
    double z;
    double R; //=PI to +PI
    double P; //=PI to +PI
    double Y; //=PI to +PI

    void update(double a, double b, double c, double roll, double pitch, double yaw)
    {
      //Kick_foot_state = kick_foot_s;
      x = a;
      y = b;
      z = c;
      R = roll;
      P = pitch;
      Y = yaw;
    }
  };

  class CartesianKeyframe
  {
    double duration; //how long the keyframe should last, in seconds
    Pose min_pose;   // min/max pose allow the specification of tolerances on each axis. Set them to the same pose for zero tolerance
    Pose max_pose;   //poses are specified in the coordinate frame of the supporting foot
    Point min_com;
    Point max_com;

    void update( double d, Pose min_p, Pose max_p, Point min_c, Point max_c)
    {
      duration = d;
      min_pose = min_p;
      max_pose = max_p;
      min_com  = min_c;
      max_com  = max_c;
    }
  };

  class FootSensor
  {
    double fl;
    double fr;
    double rl;
    double rr;

    void update(double a, double b, double c, double d)
    {
      fl = a;
      fr = b;
      rl = c;
      rr = d;
    }
  };

  class Kack
  {
  public:
    Kack(std::string model_filename)
    {

    }

    ~Kack()
    {

    }

    //first keyframe should be at the foot's pose in the nominal standing position
    bool plan(std::vector<CartesianKeyframe> keyframes, bool left_foot_supporting=true)
    {

    }

    //current_time in seconds, command is output joint positions
    void execute(double current_time, std::vector<double> current_joint_positions, FootSensor left_foot, FootSensor right_foot, std::vector<double>& command)
    {

    }
  };
}
#endif //KACK_HPP

