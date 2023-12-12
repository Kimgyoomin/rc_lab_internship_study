#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  // ROS 초기화, node handle, async spinner 추가
  ros::init(argc, argv, "add_collision_object");
  ros::NodeHandle nh;

  ros::AsyncSpinner spin(1);
  spin.start();

  // moveit::planning::PlanningSceneInterface 객체 생성
  // 객체 인스턴스 생성을 기다리고자 5초의 절전모드 추가
  moveit::planning_interface::PlanningSceneInterface current_scene;
  sleep(5.0);

  // moveit_msgs::CollisionObject 충돌 객체 메시지를 생성 -> 현재 플래닝 씬으로 전송
  // 여기서는 실린더 모양에 대한 충돌 객체 만듦
  // z1_arm_cylinder로 제공, 객체의 이름은 객체의 ID(identifier)임.
  moveit_msgs::CollisionObject cylinder;
  cylinder.id = "z1_arm_cylinder";

  // 충돌객체 메시지 만든 후, shape_msgs::SolidPrimitive 타입의 또 다른 메시지 정의
  // 기본 모양의 종류, 속성을 정의하는데 사용
  // 모양의 타입, 크기 조정 요소, 실린더 너비 및 높이를 정의
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.6;
  primitive.dimensions[1] = 0.2;
  primitive.dimensions[2] = 0.2;

  // shape 메시지를 생성 후, 객체 포즈 정의 -> geometry_msgs::Pose 메시지 생성
  // 로봇에 더 가까울 수 있는 포즈 정의
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = -0.4;
  pose.orientation.z = -0.4;

  // 충돌 객체의 포즈 정의 후 기본객체, 포즈를 실린더 충돌 객체에 추가!!
  // 다음과 같이 플래닝 씬에 추가!!
  cylinder.primitives.push_back(primitive);
  cylinder.primitive_poses.push_back(pose);
  cylinder.operation = cylinder.ADD;
  cylinder.header.frame_id = "base_link";

  // 이 벡터에 충돌 객체 삽입해 moveit_msgs::CollisionObject 타입의
  // collision_objects라는 벡터를 생성
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(cylinder);

  // 다음 코드로 충돌 객체의 벡터를 현재 플래닝 씬에 추가
  // PlanningSceneInterface 클래스 내의 addCollisionObjects()는
  // 플래닝 씬에 객체를 추가하는데 사용
  current_scene.addCollisionObjects(collision_objects);
  sleep(2.0);

  ros::shutdown();
  return 0;
}