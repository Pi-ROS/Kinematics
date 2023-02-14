#include "tasks/task.hpp"

static VEC3 castlePositions[3] = {
    VEC3(0.40, -0.1, 0.72),
    VEC3(0.355, -0.1, 0.72),
    VEC3(0.445, -0.1, 0.72)
};

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
            pijoint_vision::Object objectsArray[3];
            int tmp = 1;

            for(int i=0; i < detection_srv.response.l; i++ ){
                obj = detection_srv.response.objects[i];
                class_id = obj.o_class;
                if(class_id == 7){
                    // found x1-y4-z2
                    objectsArray[0] = obj;
                } else {
                    // found x1-y1-z2
                    objectsArray[tmp] = obj;
                    tmp++;
                }
            }
            
            for(int i=0; i < detection_srv.response.l; i++ ){
                obj = objectsArray[i];
                class_id = obj.o_class;
                desiredPosition << obj.box.center.x, obj.box.center.y, obj.box.center.z;
                block_rotation = obj.box.rotation.yaw;
                ROS_INFO_STREAM("\nClass: " << targetNames[class_id] << "\nPose:\n" << desiredPosition << "\nRotation: " << block_rotation);

                ur5.move(desiredPosition);
                ur5.descent(Robot::descentHeight, block_rotation, true);
                ur5.move(castlePositions[i]);
                ur5.descent(castlePositions[i](2), 0, false);
            }

        }
        
    }
    else{
        ROS_INFO_STREAM("Failed to call service");
        exit(0);
    }
    return true;
}