#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "seven_dof_arm_planner");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // moveit::planning_interface::PlanningSceneInterface 객체를 생성한 후
    moveit::planning_interface::MoveGroupInterface group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::PlanningSceneInterface current_scene;
    // 씬 초기화를 위해 기다림
    sleep(2.0);

    // moveit_msgs::AttachedCollisionObject인스턴스를 초기화 하고 로봇 몸체의 특정 링크에 연결될
    // 씬 객체에 대한 정보를 입력

    moveit_msgs::CollisionObject grasping_object;
    grasping_object.id = "grasping_object";

    // --- 씬에 잡을 물체를 넣는 작업
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.03;
    primitive.dimensions[1] = 0.03;
    primitive.dimensions[2] = 0.08;

    geometry_msgs::Pose pose;
    pose.position.x = 0.03;
    pose.position.y = 0.0;
    pose.position.z = 0.05;

    grasping_object.primitives.push_back(primitive);
    grasping_object.primitive_poses.push_back(pose);

    // 로봇 링크에 부착된 grasping_object 객체는 add_collision_object.cpp에서 사용됨
    // 객체가 성공적으로 부착되면 링크는 녹색 -> 보라색 그리고 로봇 모션과 함께 이동
    // 로봇 본체에서 객체를 분리하려면 applyAttachedCollisionObject 함수를 호출 후, 
    // ADD에서 REMOVE로 수정해야함
    //grasping_object.operation = grasping_object.REMOVE;
    grasping_object.operation = grasping_object.ADD;
    grasping_object.header.frame_id = "base_link";
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(grasping_object);

    current_scene.addCollisionObjects(collision_objects);

    sleep(4.0);

    ROS_INFO("Attaching object grasping_object to robot's body");
    moveit_msgs::AttachedCollisionObject attached_object;

    attached_object.link_name = "grasping_frame";
    attached_object.object = grasping_object;
    current_scene.applyAttachedCollisionObject(attached_object);

    sleep(1.0);

    ros::shutdown();

    return 0;
}