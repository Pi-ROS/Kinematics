#include "tasks/task.hpp"
#include "controllers.hpp"
#include "link_attacher.hpp"

const char *brick_model = "x1-y4-z2";
void standUpBrick(Robot &ur5, double yaw, VEC2 piolini, char *model);
const SE3 getSE3Pose(VEC3 xyz, VEC3 rpy);
void computeMotionPlan(VEC2 pioliniDisplacement, pijoint_vision::Object obj);

typedef enum
{
    GRASP,
    RELEASE,
    NOTHING
} GripperMove;

typedef struct
{
    SE3 pose;
    GripperMove move;
} Position;

Position positions[13];

bool task5(ros::ServiceClient &detectClient, ros::ServiceClient &gripperClient)
{
    VEC3 desiredPosition;
    pijoint_vision::ObjectDetection detection_srv;
    detection_srv.request.detect = true;
    pijoint_vision::Object obj;
    double block_rotation = 0;
    int o_class = 0;

    if (detectClient.call(detection_srv) && detection_srv.response.success && detection_srv.response.l > 0)
    {
        // Configuration of the motion plan
        obj = detection_srv.response.objects[0];
        MAT2 rotation2D = rotMatrix2D(obj.box.yaw);
        VEC2 piolini;
        piolini << obj.box.piolini.x, obj.box.piolini.y;
        VEC2 pioliniDisplacement = rotation2D * piolini;
        computeMotionPlan(pioliniDisplacement, obj);

        // The motion plan is carried out
        for (int i = 0; i < 13; ++i)
        {
            VEC6 q_des = ur5.inverseKinematics(positions[i].pose);
            velocityController(ur5, DT, VELOCITY, q_des, false);

            if (positions[i].move == GRASP)
                attach(ur5_model, ur5_hand_link, (char *)brick_model, "link");
            else if (positions[i].move == RELEASE)
                detach(ur5_model, ur5_hand_link, (char *)brick_model, "link");
        }
        VEC3 targetPosition = targetPositions[getClassTargetPosition(obj.o_class)];
        desiredPosition << targetPosition(0), targetPosition(1), WORKING_HEIGHT;
        SE3 T_des = SE3Operations::getGripperPose(desiredPosition, M_PI / 2);
        ur5.move(T_des);
        T_des(2, 3) = targetPosition(2);
        task3v1Descent(ur5, T_des, false, obj.o_class, (char *)brick_model);
    }
    else
    {
        ROS_ERROR("Failed to call service detect");
        return false;
    }
    return true;
}

void computeMotionPlan(VEC2 pioliniDisplacement, pijoint_vision::Object obj)
{
    if (pioliniDisplacement(0) < 0)
    {
        positions[0] = {.pose = getSE3Pose(VEC3(0, 0, WORKING_HEIGHT), VEC3(0, 0, 0)), .move = NOTHING};
        positions[1] = {.pose = getSE3Pose(VEC3(0, 0, DESCENT_HEIGHT), VEC3(0, 0, obj.box.yaw)), .move = GRASP};
        positions[2] = {.pose = getSE3Pose(VEC3(0, 0, DESCENT_HEIGHT), VEC3(0, 0, 0)), .move = NOTHING};
        positions[3] = {.pose = getSE3Pose(VEC3(0, 0, WORKING_HEIGHT), VEC3(0, 0, 0)), .move = NOTHING};
        positions[4] = {.pose = getSE3Pose(VEC3(0, 0, WORKING_HEIGHT), VEC3(-M_PI / 2, 0, 0)), .move = NOTHING};
        positions[5] = {.pose = getSE3Pose(VEC3(0, 0, DESCENT_HEIGHT - 0.06), VEC3(-M_PI / 2, 0, 0)), .move = RELEASE};
        positions[6] = {.pose = getSE3Pose(VEC3(0, 0.04, WORKING_HEIGHT), VEC3(-M_PI / 2, 0, 0)), .move = NOTHING};
        positions[7] = {.pose = getSE3Pose(VEC3(0, 0.04, WORKING_HEIGHT), VEC3(0, 0, 0)), .move = NOTHING};
        positions[8] = {.pose = getSE3Pose(VEC3(0.02, 0.04, WORKING_HEIGHT), VEC3(0, 0, 0)), .move = NOTHING};
        positions[9] = {.pose = getSE3Pose(VEC3(0.02, 0.04, WORKING_HEIGHT), VEC3(-M_PI / 2, 0, M_PI / 2)), .move = NOTHING};
        positions[10] = {.pose = getSE3Pose(VEC3(0.02, 0.04, DESCENT_HEIGHT - 0.06), VEC3(-M_PI / 2, 0, M_PI / 2)), .move = GRASP};
        positions[11] = {.pose = getSE3Pose(VEC3(0.02, 0, WORKING_HEIGHT), VEC3(-M_PI / 2, 0, M_PI / 2)), .move = NOTHING};
        positions[12] = {.pose = getSE3Pose(VEC3(0.02, 0, WORKING_HEIGHT), VEC3(0, 0, 0)), .move = NOTHING};
    }
    else
    {
        positions[0] = {.pose = getSE3Pose(VEC3(0, 0, WORKING_HEIGHT), VEC3(0, 0, 0)), .move = NOTHING};
        positions[1] = {.pose = getSE3Pose(VEC3(0, 0, DESCENT_HEIGHT), VEC3(0, 0, obj.box.yaw)), .move = GRASP};
        positions[2] = {.pose = getSE3Pose(VEC3(0, 0, DESCENT_HEIGHT), VEC3(0, 0, 0)), .move = NOTHING};
        positions[3] = {.pose = getSE3Pose(VEC3(0, 0, WORKING_HEIGHT), VEC3(0, 0, 0)), .move = NOTHING};
        positions[4] = {.pose = getSE3Pose(VEC3(0, 0, WORKING_HEIGHT), VEC3(-M_PI / 2, 0, 0)), .move = NOTHING};
        positions[5] = {.pose = getSE3Pose(VEC3(0, 0, DESCENT_HEIGHT - 0.06), VEC3(-M_PI / 2, 0, 0)), .move = RELEASE};
        positions[6] = {.pose = getSE3Pose(VEC3(0, 0.04, WORKING_HEIGHT), VEC3(-M_PI / 2, 0, 0)), .move = NOTHING};
        positions[7] = {.pose = getSE3Pose(VEC3(0, 0.04, WORKING_HEIGHT), VEC3(0, 0, 0)), .move = NOTHING};
        positions[8] = {.pose = getSE3Pose(VEC3(-0.02, 0.04, WORKING_HEIGHT), VEC3(0, 0, 0)), .move = NOTHING};
        positions[9] = {.pose = getSE3Pose(VEC3(-0.02, 0.04, WORKING_HEIGHT), VEC3(-M_PI / 2, 0, -M_PI / 2)), .move = NOTHING};
        positions[10] = {.pose = getSE3Pose(VEC3(-0.02, 0.04, DESCENT_HEIGHT - 0.06), VEC3(-M_PI / 2, 0, -M_PI / 2)), .move = GRASP};
        positions[11] = {.pose = getSE3Pose(VEC3(-0.02, 0, WORKING_HEIGHT), VEC3(-M_PI / 2, 0, -M_PI / 2)), .move = NOTHING};
        positions[12] = {.pose = getSE3Pose(VEC3(-0.02, 0, WORKING_HEIGHT), VEC3(0, 0, 0)), .move = NOTHING};
    }

    for (int i = 0; i < 13; ++i)
    {
        positions[i].pose(0, 3) += obj.box.center.x;
        positions[i].pose(1, 3) += obj.box.center.y;
    }
}

const SE3 getSE3Pose(VEC3 xyz, VEC3 rpy)
{
    return SE3Operations::Tx(xyz(0)) * SE3Operations::Ty(xyz(1)) * SE3Operations::Tz(xyz(2)) *
           SE3Operations::Rz(rpy(2)) * SE3Operations::Ry(rpy(1)) * SE3Operations::Rx(rpy(0));
}