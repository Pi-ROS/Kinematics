#include "tasks/task.hpp"
#include "link_attacher.hpp"
#include "controllers.hpp"

#if TASK_SELECTION == 4

#endif

static SE3 castlePositions[4] = {
    SE3Operations::getGripperPose(VEC3(0.40, -0.1, DESCENT_HEIGHT), 0),
    SE3Operations::getGripperPose(VEC3(0.32, -0.1, DESCENT_HEIGHT), 0),
    SE3Operations::getGripperPose(VEC3(0.36, -0.145, DESCENT_HEIGHT - 0.1), M_PI/2),
    SE3Operations::getGripperPose(VEC3(0.36, -0.055, DESCENT_HEIGHT - 0.1), M_PI/2),
};

void task4Descent(Robot &ur5, SE3 T_des, bool pick, int brick_class, char* detected_model);

bool task4(ros::ServiceClient &detectClient, ros::ServiceClient &gripperClient){
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
            pijoint_vision::Object objectsArray[4];

            int tmp8 = 0;
            int tmp7 = 2;
            for(int i=0; i < detection_srv.response.l; i++ ){
                obj = detection_srv.response.objects[i];
                class_id = obj.o_class;

                switch (class_id) {
                    case 8:
                        objectsArray[tmp8++] = obj;
                        break;
                    case 7:
                        objectsArray[tmp7++] = obj;
                        break;
                }
            };
            
            for(int i=0; i < detection_srv.response.l; i++ ){
                obj = objectsArray[i];
                class_id = obj.o_class;
                desiredPosition << obj.box.center.x, obj.box.center.y, WORKING_HEIGHT;
                char* detected_model = (char*) brick_models_task4[getModelFromPosition(desiredPosition(0), desiredPosition(1))];
                block_rotation = obj.box.yaw;
                ROS_INFO_STREAM("\nClass: " << targetNames[class_id] << "\nPose:\n" << desiredPosition << "\nRotation: " << block_rotation);

                // Reach the brick
                SE3 T_des = SE3Operations::getGripperPose(desiredPosition, block_rotation);
                ur5.move(T_des);
                T_des(2, 3) = DESCENT_HEIGHT;
                task4Descent(ur5, T_des, true, class_id, detected_model);

                // Move to the final position
                T_des = castlePositions[i];
                T_des(2, 3) = WORKING_HEIGHT;
                ur5.move(T_des);
                T_des(2, 3) = castlePositions[i](2, 3);
                task4Descent(ur5, T_des, false, class_id, detected_model);
            }

        }
        
    }
    else{
        ROS_INFO_STREAM("Failed to call objects detection service");
        exit(0);
    }
    return true;
}

void task4Descent(Robot &ur5, SE3 T_des, bool pick, int brick_class, char* detected_model){
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
        if (brick_class == 7) {
            attach(brick_models_task4[0], "link", detected_model, "link");
            attach(brick_models_task4[1], "link", detected_model, "link");
        }
        detach(ur5_model, ur5_hand_link, detected_model, "link");
    }
        
    ros::Duration(1).sleep();
    ROS_INFO_STREAM("START: ascent");
    velocityController(ur5, DT, VELOCITY, q0, true);
    ROS_INFO_STREAM("FINISH: ascent");
}