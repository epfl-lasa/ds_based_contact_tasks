#include "TwoRobotsTransform.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "two_robots_transform");
  
  TwoRobotsTransform::Mode mode;

  if (argc == 3)
  {
    if(std::string(argv[1]) == "-m" && std::string(argv[2]) == "s")
    {
      mode = TwoRobotsTransform::Mode::SIM;
    }
    else if(std::string(argv[1]) == "-m" && std::string(argv[2]) == "r")
    {
      mode = TwoRobotsTransform::Mode::REAL;
    }
    else
    {
      ROS_ERROR("You are missing arguments: -m(mode) s(simulation) or r(real)");
      return 0;
    }
  }
  else
  {
    ROS_ERROR("You are missing arguments: -m(mode) s(simulation) or r(real)");
    return 0;
  }

  ros::NodeHandle n;
  float frequency = 200.0f;

  TwoRobotsTransform twoRobotsTransform(n,frequency,mode);

  if (!twoRobotsTransform.init()) 
  {
    return -1;
  }
  else
  {
    twoRobotsTransform.run();
  }
  return 0;
}
