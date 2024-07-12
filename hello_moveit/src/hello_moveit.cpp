#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Next step goes here
    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "classic_six");

    // Construct and initialize MoveItVisualTools
    auto moveit_visual_tools =
        moveit_visual_tools::MoveItVisualTools { node,
                                                 "base_link",
                                                 rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                                 move_group_interface.getRobotModel() };
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();
    // Create closures for visualization
    auto const draw_title = [&moveit_visual_tools](auto text) {
        auto const text_pose = [] {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.0; // Place text 1m above the base link
            return msg;
        }();
        moveit_visual_tools
            .publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    };
    auto const prompt = [&moveit_visual_tools](auto text) { moveit_visual_tools.prompt(text); };

    auto const draw_trajectory_tool_path =
        [&moveit_visual_tools,
         jmg = move_group_interface.getRobotModel()->getJointModelGroup("classic_six"
         )](auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

    {
        moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
        moveit::core::RobotState start_state(*current_state);
        std::vector<double> joint_group_positions;
        start_state.copyJointGroupPositions(
            move_group_interface.getCurrentState()->getJointModelGroup("classic_six"),
            joint_group_positions
        );
        joint_group_positions[0] = 1.0;
        start_state.setJointGroupPositions(
            move_group_interface.getCurrentState()->getJointModelGroup("classic_six"),
            joint_group_positions
        );
        move_group_interface.setStartState(start_state);
    }

    // Set a target Pose
    auto const target_pose = [] {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = 0.28;
        msg.position.y = -0.2;
        msg.position.z = 0.5;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);
    // Create collision object for the robot to avoid
    auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "box1";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 2.0;
        primitive.dimensions[primitive.BOX_Y] = 2.0;
        primitive.dimensions[primitive.BOX_Z] = 0.2;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w =
            1.0; // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = -0.1;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();
    // Add the collision object to the scene
    // 这玩意是操作一个全局唯一实例吗么
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);

    // Create a plan to that target pose
    prompt("Press 'Next' in the Gui to plan");
    draw_title("Planning");
    moveit_visual_tools.trigger();
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success) {
        draw_trajectory_tool_path(plan.trajectory_);
        const auto points = plan.trajectory_.joint_trajectory.points;
        for (size_t i = 0; i < points.size(); ++i) {
            const float duration =
                points[i].time_from_start.sec + points[i].time_from_start.nanosec / 1e9;
            RCLCPP_INFO(
                logger,
                "Point %zu: %f %f %f %f %f %f %f",
                i,
                points[i].positions[0],
                points[i].positions[1],
                points[i].positions[2],
                points[i].positions[3],
                points[i].positions[4],
                points[i].positions[5],
                duration
            );
        }
        const auto& last_point = points.back();
        std::vector<double> joint_group_positions = last_point.positions;
        moveit::core::RobotStatePtr target_state = move_group_interface.getCurrentState();
        const moveit::core::JointModelGroup* joint_model_group =
            move_group_interface.getCurrentState()->getJointModelGroup("classic_six");
        target_state->setJointGroupPositions(joint_model_group, joint_group_positions);
        const auto& end_effector_state = target_state->getGlobalLinkTransform("wrist3_Link");
        RCLCPP_INFO(
            logger,
            "End effector position: %f %f %f",
            end_effector_state.translation().x(),
            end_effector_state.translation().y(),
            end_effector_state.translation().z()
        );
        // RCLCPP_INFO(
        //     logger,
        //     "End effector orientation: %f %f %f %f",
        //     end_effector_state.rotation().x(),
        //     end_effector_state.rotation().y(),
        //     end_effector_state.rotation().z(),
        //     end_effector_state.rotation().w()
        // );

        moveit_visual_tools.trigger();
        prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
        draw_title("Executing");
        moveit_visual_tools.trigger();
        move_group_interface.execute(plan);
    } else {
        draw_title("Planning Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Planning failed!");
    }
    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
