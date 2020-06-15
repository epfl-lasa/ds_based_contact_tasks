#include "SurfacePuncturing.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "surface_puncturing");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
  std::string fileName;
  float targetVelocity;
  float targetForce;


  std::ostringstream ss;
  std::string temp;

  if(argc == 2) 
  {
    fileName = std::string(argv[1]);

  }
  else
  {
    ROS_ERROR("You are missing arguments: the command line arguments should be: fileName");
    return 0;
  }

  SurfacePuncturing surfacePuncturing(n,frequency,fileName);

  if(!surfacePuncturing.init()) 
  {
    return -1;
  }
  else
  { 
    surfacePuncturing.run();
  }

  return 0;
}

