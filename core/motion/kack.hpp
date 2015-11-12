#ifndef KACK_HPP
#define KACK_HPP

class Pose
{
  double x;
  double y;
  double z;
  double R; //=PI to +PI
  double P; //=PI to +PI
  double Y; //=PI to +PI
};

class CartesianKeyframe
{
  double duration; //how long the keyframe should last, in seconds
  Pose min_pose;   // min/max pose allow the specification of tolerances on each axis. Set them to the same pose for zero tolerance
  Pose max_pose;   //poses are specified in the coordinate frame of the supporting foot
};

class FootSensor
{
  double fl;
  double fr;
  double rl;
  double rr;
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

  void execute(double time, std::vector<double> current_joint_positions, FootSensor left_foot, FootSensor right_foot, std::vector<double>& command)
  {

  }
};

#endif //KACK_HPP