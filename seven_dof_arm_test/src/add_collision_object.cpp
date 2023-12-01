#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
    // ROS 초기화함, node Handle, async spinner를 만든다.
    ros::init(argc, argv, "add_collision_object");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(1);
    spin.start();

    // moveit::planning_interface::PlanningSceneInterface 객체를 생성
    // 이 객체는 MoveIt!의 플래닝 씬에 접근해 어떤 작업도 수행할 수 있다.
    // 이제 PlanningSceneInterface 객체 인스턴스 생성을 기다리고자 5초의 절전 모드를 추가
    moveit::planning_interface::PlanningSceneInterface current_scene;
    sleep(5.0);
    // moveit_msgs::CollisionObject 충돌 객체 메시지를 생성해야함. -> 이 메세지는 현재 플래닝 중인 씬으로 전송
    // 여기서는 실린더 모양에 대한 충돌 객체 메시지를 만들고 있음
    // 메시지는 seven_dof_arm_cylinder로 제공된다.
    // 이 객체를 플래닝 씬에 추가할 때 다음 코드처럼 객체의 이름은 객체의 id(identifier)이다.
    moveit_msgs::CollisionObject cylinder;
    cylinder.id = "seven_dof_arm_cylinder";

    // 충돌 객체 메시지를 만든 후에는 shape_msgs::SolidPrimitive 타입의 또 다른 메시지를 정의해야함
    // 이 메시지는 기본 모양의 종류와 속성을 정의하는데 사용
    // 모양의 타입, 크기 조정 요소(resize), 실린더 너비 및 높이를 정의해야함
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.6;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.2;

    // shape 메시지를 생성한 후 이 객체의 포즈 정의를 위해 geometry_msgs::Pose메시지를 생성해야함
    // 로봇에 더 가까울 수 있는 포즈를 정의함
    // 플래닝씬에서 객체를 만든 후 포즈를 변경할 수 있음
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.0;
    pose.position.y = -0.4;
    pose.position.z = 0.4;

    // 충돌 객체의 포즈를 정의 후 기본 객체와 포즈를 실린더 충돌 객체에 추가해야함
    // 수행해야하는 작업은 다음과 같이 플래닝 씬을 추가하는 것
    cylinder.primitives.push_back(primitive);
    cylinder.primitive_poses.push_back(pose);
    cylinder.operation = cylinder.ADD;
                cylinder.header.frame_id = "base_link";


    // 다음과 같이 이 벡터에 충돌 객체를 삽입해 moveit_msgs::CollisionObject타입의 collision_objects라는 벡터를 셍성
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(cylinder);

    // 충돌객체의 벡터를 현재 플래닝 씬에 추가
    // PlanningSceneInterface 클래스 내의 addCollisionObjects()는 플래닝 씬에 객체를 추가하는 데에 사용
    current_scene.addCollisionObjects(collision_objects);
                sleep(2);

    ros::shutdown();            

    return 0;
}