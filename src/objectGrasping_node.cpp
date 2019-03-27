#include "ObjectGrasping.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "object_grasping");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
  float targetForce;

  std::string filename;
  ObjectGrasping::Mode mode;

  bool adaptNormalModulation;

  // rosrun motion_force_control modualtedDS fileName -u y/n -o c/a -m r/fr -f f1/f2/f3 -c v/a 
  if(argc == 8) 
  {
    filename = std::string(argv[1]);

    if(std::string(argv[2]) == "-m" && std::string(argv[3])=="rg")
    {
      mode = ObjectGrasping::Mode::REACHING_GRASPING;
    }
    else if(std::string(argv[2]) == "-m" && std::string(argv[3])=="rgm")
    {
      mode = ObjectGrasping::Mode::REACHING_GRASPING_MANIPULATING;
    }
    else
    {
      ROS_ERROR("Wrong contact dynamics arguments, the command line arguments should be: fileName -m(mode) rg(reaching and grasping)/rgm(reaching, grasping and manipulating) -f(target force) value -an(adapt normal modulation) y(yes)/n(no)");
      return 0;
    }  

    if(std::string(argv[4]) == "-f" && atof(argv[5])> 0.0f)
    {
      targetForce = atof(argv[5]);
    }
    else
    {
      ROS_ERROR("Wrong target force arguments, the command line arguments should be: fileName -m(mode) rg(reaching and grasping)/rgm(reaching, grasping and manipulating) -f(target force) value -an(adapt normal modulation) y(yes)/n(no)");
      return 0;
    }

    if(std::string(argv[6]) == "-an" && std::string(argv[7]) == "y")
    {
      adaptNormalModulation = true;
    }
    else if(std::string(argv[6]) == "-an" && std::string(argv[7]) == "n")
    {
      adaptNormalModulation = false;
    }
    else
    {
      ROS_ERROR("Wrong adapt normal modulation argument, the command line arguments should be: fileName -m(mode) rg(reaching and grasping)/rgm(reaching, grasping and manipulating) -f(target force) value -an(adapt normal modulation) y(yes)/n(no)");
      return 0;
    } 
  }
  else
  {
    ROS_ERROR("You are missing arguments: the command line arguments should be: fileName -m(mode) rg(reaching and grasping)/rgm(reaching, grasping and manipulating) -f(target force) value -an(adapt normal modulation) y(yes)/n(no)");
    return 0;
  }

  filename += "_"+std::string(argv[3])+"_"+std::string(argv[7]);


  ObjectGrasping objectGrasping(n,frequency,filename,mode,targetForce,adaptNormalModulation);

  if (!objectGrasping.init()) 
  {
    return -1;
  }
  else
  {
    objectGrasping.run();
  }

  return 0;
}

