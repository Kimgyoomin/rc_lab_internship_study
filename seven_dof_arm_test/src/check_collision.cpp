#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_kinematics");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 다음 코드는 로봇의 기구 모델을 플래닝 씬에 로드한다.
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    // 충돌 테스트를 위한 변수들을 초기화함
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    // Collision Checking
    // ^^^^^^^^^^^^^^^^^^
    //
    //Self-collision Checking
    // ^^^^^^^^^^^^^^^^^^^^^^

    // 로봇의 현재 상태에서 자기 충돌 테스트를 위해 다음 두 가지 인스턴스를 만들 수 있다.
    // collision_request 및 collision_result로 명명된 collision_detection::CollisionRequest
    // 및 collision_detection::CollisionResult 클래스다.
    // 이런 객체 생성 후 MoveIt! 충돌 확인 함수인 planning_scene.checkSelfCollision()에 전달
    // 이 함수는 충돌 결과를 Collision_result 객체에 제공
    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    current_state.setToRandomPositions();
    collision_result.clear();

    // 다음 코드에서 세부 정보를 출력 할 수 있다.
    planning_scene.checkSelfCollision(collision_request, collision_result);

    ROS_INFO_STREAM("2. Self collision Test(Change the state): "<< (collision_result.collision ? "in" : "not in"));

    // 특정 그룹에서 충돌을 테스트하려면 다음 코드와 같이 group_name을 언급하면 된다.
    // 여기서 그룹 이름은 arm이다.
    
    collision_request.group_name = "arm";
    current_state.setToRandomPositions();

    // 이전 결과는 clear 매서드로 초기화되야한다.
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);

    ROS_INFO_STREAM("3. Self collision Test(In a group): "<< (collision_result.collision ? "in" : "not in"));

    // 전체 충돌 검사를 수행하려면 planning_scene.checkCollision()이라는 함수를 사용해야함
    // 이 함수에선 현재 로봇 상태와 ACM 매트릭스를 언급해야함

    std::vector<double> joint_values;
    const moveit::core::JointModelGroup* joint_model_group = 
        current_state.getJointModelGroup("arm");
    //const robot_model::JointModelGroup* joint_model_group =
    //    current_state.getJointModelGroup("arm");
    current_state.copyJointGroupPositions(joint_model_group, joint_values);
    joint_values[0] = 1.57; // 하드코딩된것 우리는 이미 이 지점에서 충돌이 발생할 것이라는 것을 알고 있다.
    current_state.setJointGroupPositions(joint_model_group, joint_values);
    ROS_INFO_STREAM("4. Collision points " << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));

    collision_request.contacts = true;
    collision_request.max_contacts = 1000;

    //

    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);

    ROS_INFO_STREAM("5. Self collision Test: "<< (collision_result.collision ? "in" : "not in")
                    << " self collision");
    
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for (it = collision_result.contacts.begin();
         it != collision_result.contacts.end();
         ++it)
    {
        ROS_INFO("6. Contact between: %s and %s",
                 it->first.first.c_str(),
                 it->first.second.c_str());
    }


    // collision_detection 클래스 : 'AllowedCollisionMatrix(ACM) <- 앞에서 언급함
    // 무시할 충돌 세계에 대한 매커니즘을 알려준다
    // 

    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = planning_scene.getCurrentState();

    collision_detection::CollisionResult::ContactMap::const_iterator it2;
    for(it2 = collision_result.contacts.begin();
        it2 != collision_result.contacts.end();
        ++it2)
    {
        acm.setEntry(it2->first.first, it2->first.second, true);
    }
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);

    ROS_INFO_STREAM("6. Self collision Test after modified ACM: "<< (collision_result.collision ? "in" : "not in")
                    << " self collision ");



    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);

    ROS_INFO_STREAM("6. Full collision Test: "<< (collision_result.collision ? "in" : "not in")
                    << " collision ");

    ros::shutdown();

    return 0;
}