#include "ros/ros.h"
#include "open_data_msg/BoundingBox.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bounding_box_client");
  if ( (argc != 6 && argc !=7) || (argc==7 && strcmp(argv[1],"all")!=0) || (argc==6 && strcmp(argv[1],"all")==0) )
  {
    ROS_INFO("usage1: bounding_box_client all type long1 lat1 long2 lat2 - type=0 STATIC, type=1 DYNAMIC");
    ROS_INFO("usage2: bounding_box_client label long1 lat1 long2 lat2");
    return 1;
  }



  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<open_data_msg::BoundingBox>("bounding_box");
  open_data_msg::BoundingBox srv;

srv.request.label = argv[1];
srv.request.bounding_box.points.resize(2);

	if(argc == 7)
	{	
		srv.request.type = atoi(argv[2]);
	}

	srv.request.bounding_box.points[0].x = atof(argv[argc-4]); 
	srv.request.bounding_box.points[0].y = atof(argv[argc-3]);
	srv.request.bounding_box.points[1].x = atof(argv[argc-2]);
	srv.request.bounding_box.points[1].y = atof(argv[argc-1]);


  
 if (client.call(srv))
  {
	if(srv.response.resp)
	    ROS_INFO("Result: true");
	else			    
	   ROS_INFO("Result: false");
  }
  else
  {
    ROS_ERROR("Failed to call service bounding_box");
    return 1;
  }

  return 0;
}
