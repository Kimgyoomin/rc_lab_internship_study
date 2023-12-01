#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "remove_collision_object");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(1);
    spin.start();

    // 물체를 생성해서 부착했던것처럼 moveit::planning_interface::PlanningSceneInterface 객체를 생성해야함
    moveit::planning_interface::PlanningSceneInterface current_scene;
    sleep(5.0);

    // 다음으로 충돌 객체 ID를 포함하는 문자열의 벡터를 만들어야함(여기선 seven_dof_arm_cylinder)
    // 이 벡터에 문자열을 푸시한 후 removeCollisionObjects(object_ids)를 호출하여 플래닝 씬에서 충돌 객체를 제거
    std::vector<std::string> object_ids;
    object_ids.push_back("seven_dof_arm_cylinder");
    current_scene.removeCollisionObjects(object_ids);

    ros::shutdown();

    return 0;
}