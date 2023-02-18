#include "tasks/task.hpp"
#include "controllers.hpp"
#include "link_attacher.hpp"

const char* topBricks[11] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
void task3v1Descent(Robot &ur5, SE3 T_des, bool pick, int brick_class, char* detected_model);

bool task3(ros::ServiceClient &detectClient, ros::ServiceClient &gripperClient){
    VEC3 desiredPosition;
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
                desiredPosition << obj.box.center.x, obj.box.center.y, WORKING_HEIGHT;
                char* detected_model = (char*) brick_models_task3[getModelFromPosition(desiredPosition(0), desiredPosition(1))];
                block_rotation = obj.box.rotation.yaw;
                ROS_INFO_STREAM("\nClass: " << targetNames[class_id] << "\nPose:\n" << desiredPosition << "\nRotation: " << block_rotation);

                // Reach the brick
                SE3 T_des = SE3Operations::getGripperPose(desiredPosition, block_rotation);
                ur5.move(T_des);
                T_des(2, 3) = DESCENT_HEIGHT;
                task3v1Descent(ur5, T_des, true, class_id, detected_model);

                // Move to the final position
                VEC3 targetPosition = targetPositions[getClassTargetPosition(class_id)];
                desiredPosition << targetPosition(0), targetPosition(1), WORKING_HEIGHT;
                T_des = SE3Operations::getGripperPose(desiredPosition, M_PI/2);
                ur5.move(T_des);
                T_des(2, 3) = targetPosition(2);
                task3v1Descent(ur5, T_des, false, class_id, detected_model);

                // update the target height for the given brick class
                targetPositions[getClassTargetPosition(class_id)](2) -= classHeight[class_id]; // brick's height
            }

        }
        
    }
    else{
        ROS_INFO_STREAM("Failed to call objects detection service");
        exit(0);
    }
    return true;
}

void task3v1Descent(Robot &ur5, SE3 T_des, bool pick, int brick_class, char* detected_model){
    ros::Rate loop_rate(LOOP_FREQUENCY);
    VEC6 q0 = ur5.joints.q();
    VEC3 q_gripper = ur5.joints.q_gripper();
    VEC6 q_des;
    
    ROS_INFO_STREAM("START: descent");
    SE3 T_curr = ur5.forwardKinematics(q0);
    SE3 T_des1 = T_des;
    T_des1(0, 3) = T_curr(0, 3);
    T_des1(1, 3) = T_curr(1, 3);
    T_des1(2, 3) = T_curr(2, 3);
    q_des = ur5.inverseKinematics(T_des1);
    velocityController(ur5, DT, VELOCITY, q_des, false);

    q_des = ur5.inverseKinematics(T_des);
    velocityController(ur5, DT, VELOCITY, q_des, false);
    ROS_INFO_STREAM("FINISH: descent");
   
    if (pick) {
        attach(ur5_model, ur5_hand_link, detected_model, "link");
    }
    else {
        detach(ur5_model, ur5_hand_link, detected_model, "link");
        if (topBricks[brick_class] != NULL)
            attach(topBricks[brick_class], "link", detected_model, "link");
        else
            topBricks[brick_class] = detected_model;
    }
        
    ros::Duration(1).sleep();
    ROS_INFO_STREAM("START: ascent");
    velocityController(ur5, DT, VELOCITY, q0, true);
    ROS_INFO_STREAM("FINISH: ascent");
}

int getModelFromPosition(double x, double y){
    double distances[4];
    for (int i = 0; i < 4; i++){
        distances[i] = pow(x - models_positions[i][0], 2) + pow(y - models_positions[i][1], 2);
    }
    int min = 0;
    for (int i = 1; i < 4; i++){
        if (distances[i] < distances[min]){
            min = i;
        }
    }
    return min;
}