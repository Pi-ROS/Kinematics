#include "tasks/task.hpp"

bool task3(ros::ServiceClient &detect){
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
                desiredPosition << obj.box.center.x, obj.box.center.y, ur5.workingHeight;
                block_rotation = obj.box.rotation.yaw;
                ROS_INFO_STREAM("\nClass: " << targetNames[class_id] << "\nPose:\n" << desiredPosition << "\nRotation: " << block_rotation);

                // Reach the brick
                SE3 T_des = SE3Operations::getGripperPose(desiredPosition, block_rotation);
                ur5.move(T_des);
                T_des(2, 3) = ur5.descentHeight;
                ur5.descent(T_des, true);

                // Move to the final position
                VEC3 targetPosition = targetPositions[getClassTargetPosition(class_id)];
                desiredPosition << targetPosition(0), targetPosition(1), ur5.workingHeight;
                T_des = SE3Operations::getGripperPose(desiredPosition, M_PI/2);
                ur5.move(T_des);
                T_des(2, 3) = targetPosition(2);
                ur5.descent(T_des, false);

                // update the target height for the given brick class
                targetPositions[getClassTargetPosition(class_id)](2) -= 0.057; // brick's height
            }

        }
        
    }
    else{
        ROS_INFO_STREAM("Failed to call service");
        exit(0);
    }
    return true;
}