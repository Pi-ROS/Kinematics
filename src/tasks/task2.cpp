#include "tasks/task.hpp"

bool task2(ros::ServiceClient &detect){
    VEC3 desiredPosition;
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
                desiredPosition << obj.box.center.x, obj.box.center.y, obj.box.center.z;
                block_rotation = obj.box.rotation.yaw;
                ROS_INFO_STREAM("\nClass: " << nameArray[class_id] << "\nPose:\n" << desiredPosition << "\nRotation: " << block_rotation);

                ur5.move(desiredPosition);
                ur5.descent(Robot::descentHeight, block_rotation, true);
                ur5.move(targetPositions[class_id]);
                ur5.descent(Robot::descentHeight, M_PI/2, false);
                Controller::velocityController(ur5, Controller::dt, 1.5, ur5.q_home, false);
            }

        }
        
    }
    else{
        ROS_INFO_STREAM("Failed to call service");
        exit(0);
    }
    return true;
}