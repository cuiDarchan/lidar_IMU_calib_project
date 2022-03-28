#include "align_to_map.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "align_to_map_node");
  ros::NodeHandle nh;
  AlignToMap app(nh);
  
  ros::Rate loop_rate(10);
  while(ros::ok()){
    app.Run();
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}