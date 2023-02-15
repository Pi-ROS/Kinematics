#include "tasks/task.hpp"

bool task1(ros::ServiceClient &detect){
    VEC3 STATION;
    VEC3 pose;
    STATION << 0.30, -0.24, 0.71;
    
    pijoint_vision::ObjectDetection detection_srv;
    detection_srv.request.detect = true;
    pijoint_vision::Object obj;
    double block_rotation = 0;
    int class_id = 0;

    if (detect.call(detection_srv)){

        if (detection_srv.response.success && detection_srv.response.l > 0)
        {
            ROS_INFO_STREAM("Object detected");
            
            for(int i=0; i < detection_srv.response.l; i++ ){
               
                obj = detection_srv.response.objects[i];
                class_id = obj.o_class;
                pose << obj.box.center.x, obj.box.center.y, obj.box.center.z;
                block_rotation = obj.box.rotation.yaw;
                ROS_INFO_STREAM("\nClass: " << targetNames[class_id] << "\nPose:\n" << pose << "\nRotation: " << block_rotation);

                // Reach the brick
                SE3 T_des;
                T_des << cos(block_rotation), -sin(block_rotation), 0, pose(0),
                        sin(block_rotation), cos(block_rotation),  0, pose(1),
                        0, 0, 1, ur5.workingHeight,
                        0, 0, 0, 1;
                ur5.move(T_des);
                T_des(2, 3) = ur5.descentHeight;
                ur5.descent(T_des, true);

                // Move to the final position
                T_des << 0, -1, 0, STATION(0),
                         1,  0, 0, STATION(1),
                         0,  0, 1, ur5.workingHeight,
                         0,  0, 0, 1;
                ur5.move(T_des);
                T_des(2, 3) = STATION(2);
                ur5.descent(T_des, false);
            }

        }
        
    }
    else{
        ROS_INFO_STREAM("Failed to call service");
        exit(0);
    }
    
    return true;
}