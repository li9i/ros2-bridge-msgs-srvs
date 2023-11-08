#include "ros1_out.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros1_out_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  R1L r1l(nh, nh_private);
  ros::spin();
  return 0;
}
