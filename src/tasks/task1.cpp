#include "tasks/task1.hpp"



bool task1(ros::ServiceClient &detect){

    VEC3 STATION;
    VEC3 pose;
    STATION << 0.30, -0.24, 0.72;
    
    
    pijoint_vision::ObjectDetection detection_srv;
    detection_srv.request.detect = true;
    pijoint_vision::Object obj;
    double block_rotation = 0;

    if (detect.call(detection_srv)){

        if (detection_srv.response.success && detection_srv.response.l > 0)
        {
            ROS_INFO_STREAM("Object detected");
            
            for(int i=0; i < detection_srv.response.l; i++ ){
               
                obj = detection_srv.response.objects[i];
                pose << obj.box.center.x, obj.box.center.y, obj.box.center.z;
                ROS_INFO_STREAM("Pose:\n" << pose);
                block_rotation = obj.box.rotation.yaw;

                ur5.move(pose);
                ur5.descent(Robot::descentHeight, 0, true);
                ur5.move(STATION);
                ur5.descent(Robot::descentHeight, M_PI/2, false);
            }

        }
        
    }
    else{
        ROS_INFO_STREAM("Failed to call service");
        exit(0);
    }

    
    return true;
}