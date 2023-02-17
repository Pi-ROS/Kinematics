#include "tasks/task.hpp"

bool task1(ros::ServiceClient &detectClien, ros::ServiceClient &gripperClient){
    VEC3 station; // final position
    VEC3 pose;
    station << 0.30, -0.24, DESCENT_HEIGHT;
    
    pijoint_vision::ObjectDetection detection_srv;
    detection_srv.request.detect = true;
    pijoint_vision::Object obj;
    double block_rotation = 0;
    int class_id = 0;

    if (detectClient.call(detection_srv)){

        if (detection_srv.response.success && detection_srv.response.l > 0)
        {
            ROS_INFO_STREAM("Object detected");
            
            for(int i=0; i < detection_srv.response.l; i++ ){
               
                obj = detection_srv.response.objects[i];
                class_id = obj.o_class;
                pose << obj.box.center.x, obj.box.center.y, WORKING_HEIGHT;
                block_rotation = obj.box.rotation.yaw;
                ROS_INFO_STREAM("\nClass: " << targetNames[class_id] << "\nPose:\n" << pose << "\nRotation: " << block_rotation);

                // Reach the brick
                SE3 T_des;
                T_des = SE3Operations::getGripperPose(pose, block_rotation);
                ur5.move(T_des);
                T_des(2, 3) = DESCENT_HEIGHT;
                ur5.descent(T_des, true, gripperClient);

                // Move to the final position
                pose << station(0), station(1), WORKING_HEIGHT;
                T_des = SE3Operations::getGripperPose(pose, M_PI/2);
                ur5.move(T_des);
                T_des(2, 3) = station(2);
                ur5.descent(T_des, false, gripperClient);
            }

        }
        
    }
    else{
        ROS_INFO_STREAM("Failed to call service");
        exit(0);
    }
    
    return true;
}