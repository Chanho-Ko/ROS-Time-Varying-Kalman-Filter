#include <TimeVaryingKF_heading.h>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "pos_est_node");
   ros::NodeHandle nh;
   TimeVaryingKF TVKF(&nh);

   ros::Rate loop_rate(100);
   
   while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
   }

   return 0;
}

