#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <moveit_msgs/msg/robot_trajectory.hpp>


auto generatePoseMsg(float x,float y, float z,float qx,float qy,float qz,float qw) 
        {
          geometry_msgs::msg::PoseStamped msg;
          msg.pose.orientation.x = qx;
          msg.pose.orientation.y = qy;
          msg.pose.orientation.z = qz;
          msg.pose.orientation.w = qw;
          msg.pose.position.x = x;
          msg.pose.position.y = y;
          msg.pose.position.z = z;
          return msg;
        }
        //Function to generate a collision object
        auto generateCollisionObject(float sx,float sy, float sz, float x, float y, float z, std::string frame_id, std::string id) 
        {
          moveit_msgs::msg::CollisionObject collision_object;
          collision_object.header.frame_id = frame_id;
          collision_object.id = id;
          shape_msgs::msg::SolidPrimitive primitive;

          primitive.type = primitive.BOX;
          primitive.dimensions.resize(3);
          primitive.dimensions[primitive.BOX_X] = sx;
          primitive.dimensions[primitive.BOX_Y] = sy;
          primitive.dimensions[primitive.BOX_Z] = sz;

          geometry_msgs::msg::PoseStamped box_pose;
          box_pose.pose.orientation.w = 1.0; 
          box_pose.pose.position.x = x;
          box_pose.pose.position.y = y;
          box_pose.pose.position.z = z;

          collision_object.primitives.push_back(primitive);
          collision_object.primitive_poses.push_back(box_pose.pose);
          collision_object.operation = collision_object.ADD;

          return collision_object;
        }

using std::placeholders::_1;

class arm_control_test : public rclcpp::Node
{
    public:
        arm_control_test() : Node("arm_control_test")
        {
        planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        auto node = std::make_shared<rclcpp::Node>("arm_control_test");
        move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node,"ur_manipulator");
        frame_id = move_group_interface->getPlanningFrame();
        
        
        // Initialize collision objects here instead
        auto col_object_table = generateCollisionObject(2.4, 1.2, 0.04, 0.85, 0.25, -0.03, frame_id, "table");
        auto col_object_backWall = generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.25, 0.5, frame_id, "backWall");
        auto col_object_sideWall = generateCollisionObject(0.04, 1.2, 1.0, -0.45, 0.25, 0.5, frame_id, "sideWall");
        
        planning_scene_interface->applyCollisionObject(col_object_table);
        planning_scene_interface->applyCollisionObject(col_object_backWall);
        planning_scene_interface->applyCollisionObject(col_object_sideWall);

        // Initialize the pose subscriber
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("move_to", 10, std::bind(&arm_control_test::messageRead, this, _1));
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,std::bind(&arm_control_test::joint_state_callback, this, std::placeholders::_1));
        move_group_interface->setPlanningTime(10.0);
        move_group_interface->setNumPlanningAttempts(15);
        move_group_interface->setPlannerId("RRTstarkConfigDefault");

        generateJengaTower();
        }
        //Now we start to define the methods
        void move_to_block(const geometry_msgs::msg::PoseStamped &target_pose)
        {
          plan_and_execute(target_pose);
          //current_pose = move_group_interface->getCurrentPose();
        }
        void execute_push()
        {
          RCLCPP_INFO(this->get_logger(),"execute_push");
          //current_pose = move_group_interface->getCurrentPose();
          auto distance = 0.04; //4cm
          target_pose.pose.position.x = current_pose.pose.position.x + distance;
          //target_pose.pose.position.y = current_pose.pose.position.y;
          //target_pose.pose.position.z = current_pose.pose.position.z;
          RCLCPP_INFO(this->get_logger(), "Current Pose - Position: [x: %f, y: %f, z: %f], Orientation: [x: %f, y: %f, z: %f, w: %f]",
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z,
                current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.pose.orientation.w);
          RCLCPP_INFO(this->get_logger(), "Target Pose - Position: [x: %f, y: %f, z: %f], Orientation: [x: %f, y: %f, z: %f, w: %f]",
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w);
          applyLinearOrientationConstraint(current_pose,target_pose);
          plan_and_execute(target_pose);
          move_group_interface->clearPathConstraints(); // Clear constraints after movement
        
        }
        void move_to_same_level_other_side(const std::string &side, char face) 
        {
          current_pose = move_group_interface->getCurrentPose();
          // Check the face and calculate the target position
          if (face == 'A') {  // Assume face A
              if (side == "left") {
                  target_pose.pose.position.x = current_pose.pose.position.x - 2 * jenga_length;  // Move to the left side
              } else if (side == "right") {
                  target_pose.pose.position.x = current_pose.pose.position.x + 2 * jenga_length;  // Move to the right side
              } else if (side == "middle") {
                  target_pose.pose.position.x = current_pose.pose.position.x + jenga_length;  // Move to the middle side (add 1 block_length_)
              }
              // Keep Y and Z coordinates unchanged
              target_pose.pose.position.y = current_pose.pose.position.y;
              target_pose.pose.position.z = current_pose.pose.position.z;

          } else if (face == 'B') {  // Assume face B
              if (side == "left") {
                  target_pose.pose.position.y = current_pose.pose.position.y + 2 * jenga_length;  // Move to the left side (increase Y)
              } else if (side == "right") {
                  target_pose.pose.position.y = current_pose.pose.position.y - 2 * jenga_length;  // Move to the right side (decrease Y)
              } else if (side == "middle") {
                  target_pose.pose.position.y = current_pose.pose.position.y + jenga_length;  // Move to the middle side (add 1 block_length_)
              }
              // Keep X and Z coordinates unchanged
              target_pose.pose.position.x = current_pose.pose.position.x;
              target_pose.pose.position.z = current_pose.pose.position.z;
          }
          plan_and_execute(target_pose);
        }
        void move_to_next_level()
        {
          current_pose = move_group_interface->getCurrentPose();
          target_pose.pose.position.x = current_pose.pose.position.x;
          target_pose.pose.position.y = current_pose.pose.position.y;
          target_pose.pose.position.z = current_pose.pose.position.z + jenga_height;  // Move up by one layer height
          plan_and_execute(target_pose);
        }
        void put_on_the_top() 
        {
          current_pose = move_group_interface->getCurrentPose();
          target_pose.pose.position.x = x_first_block_center_point;
          target_pose.pose.position.y = y_first_block_center_point;
          target_pose.pose.position.z = current_pose.pose.position.z + 18*jenga_height;  // Move up by one layer height
          plan_and_execute(target_pose);
          RCLCPP_INFO(this->get_logger(), "Putting block on top is not implemented yet");
        }

        void stop_now()
        {
          current_pose = move_group_interface->getCurrentPose();
          move_group_interface->stop();
        }
        void go_back() 
        {
          RCLCPP_INFO(this->get_logger(), "Going back");

          current_pose = move_group_interface->getCurrentPose();
          target_pose.pose.position.x = current_pose.pose.position.x - 0.04;  // Move back by 4cm
          target_pose.pose.position.y = current_pose.pose.position.y;
          target_pose.pose.position.z = current_pose.pose.position.z;
          /*
          // Call the function to compute the path
          moveit_msgs::msg::RobotTrajectory trajectory = computeStraightLinePath(current_pose, target_pose);
          // Execute the generated trajectory
          move_group_interface->execute(trajectory);
          */
        }

        
        // complex movement, do after MVP
        void rotate_to_another_face() 
        {
          //void rotate_to_another_face(char face)
          RCLCPP_INFO(this->get_logger(), "Rotating to another face");
          current_pose = move_group_interface->getCurrentPose();
          target_pose.pose.position.x = current_pose.pose.position.x;
          target_pose.pose.position.y = current_pose.pose.position.y;
          target_pose.pose.position.z = current_pose.pose.position.z;

          // Implement rotation logic (for example, modifying orientation)
          target_pose.pose.orientation.z = 0.707;  // Example for 90 degrees rotation
          target_pose.pose.orientation.w = 0.707;
          plan_and_execute(target_pose);
        }
        void grip()
        {
          //
        }
        void move_to_opposite_face() 
        {
          RCLCPP_INFO(this->get_logger(), "Moving to opposite face");

          // Example logic to calculate position of opposite face
          target_pose.pose.position.x = x_first_block_center_point + jenga_length / 2 + 0.2;
          target_pose.pose.position.y = y_first_block_center_point;  // Offset for opposite face
          target_pose.pose.position.z = current_pose.pose.position.z;
          target_pose.pose.orientation.x = 0.0; 
          target_pose.pose.orientation.y = 0.707;
          target_pose.pose.orientation.z = 0.0;
          target_pose.pose.orientation.w = 0.707;
          plan_and_execute(target_pose);
        }

    private:        
      std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
      std::string frame_id;
      moveit::planning_interface::MoveGroupInterface::Plan planMessage;
      geometry_msgs::msg::PoseStamped target_pose;
      geometry_msgs::msg::PoseStamped current_pose;
      sensor_msgs::msg::JointState current_joint_states_;
    
      double jenga_length = 0.075;
      double jenga_width = 0.025;
      double jenga_height = 0.015;
      double x_first_block_center_point = 0.15;
      double y_first_block_center_point = 0.35;
        void simple_test(const geometry_msgs::msg::PoseStamped &msg)
        {
          target_pose = msg;
          move_to_block(target_pose);
          execute_push();
          //go_back();
          //move_to_opposite_face();
          //grip();
          //put_on_the_top();

        }
                
        void setJointLimits(std::vector<double>& joint_values)
        {
            // Define joint limits for each joint
            /*
            const double base_lower_limit = 0.0;         // Base joint: 0 degrees (0 radians)
            const double base_upper_limit = 1.5708;      // Base joint: 90 degrees (π/2 radians)
            const double shoulder_lower_limit = -1.5708; // Shoulder joint: -90 degrees (-π/2 radians)
            const double shoulder_upper_limit = 0.0;     // Shoulder joint: 0 degrees (0 radians)
            */
            joint_values[5] = 0.0;
            

            // Loop through joint values to enforce limits
            /*
            for (size_t i = 0; i < joint_values.size(); i++) 
            {
                if (i == 0) // Assuming joint 0 is the base joint
                {
                    if (joint_values[i] < base_lower_limit) 
                    {
                        joint_values[i] = base_lower_limit; // Clamp to lower limit
                    } 
                    else if (joint_values[i] > base_upper_limit) 
                    {
                        joint_values[i] = base_upper_limit; // Clamp to upper limit
                    }
                } 
                else if (i == 1) // Assuming joint 1 is the shoulder lift joint
                {
                    if (joint_values[i] < shoulder_lower_limit) 
                    {
                        joint_values[i] = shoulder_lower_limit; // Clamp to lower limit
                    } 
                    else if (joint_values[i] > shoulder_upper_limit) 
                    {
                        joint_values[i] = shoulder_upper_limit; // Clamp to upper limit
                    }
                }*/
                // Add more joints here as necessary
          } 
          
        
        
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
          current_joint_states_ = *msg;
          //move_group_interface->setJointValueTarget(current_joint_states_.position);
          RCLCPP_INFO(this->get_logger(), "Received joint states.");
        }

        void plan_and_execute(const geometry_msgs::msg::PoseStamped& target_pose)
        {
            std::this_thread::sleep_for(std::chrono::seconds(2));

            // Set the target pose
            move_group_interface->setPoseTarget(target_pose);
            RCLCPP_INFO(this->get_logger(), "Target Pose - Position: [x: %f, y: %f, z: %f], Orientation: [x: %f, y: %f, z: %f, w: %f]",
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w);
            // Get the current joint states and set joint limits
            //std::vector<double> joint_values = current_joint_states_.position;
            //dsetJointLimits(joint_values);
            //move_group_interface->setJointValueTarget(joint_values);

            // Loop until planning is successful
            while (rclcpp::ok())
            {
                // Plan the movement
                if (move_group_interface->plan(planMessage)) {
                    // Execute the planned movement
                    move_group_interface->execute(planMessage);
                    break;  // Exit the loop upon successful execution
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Planning failed! Retrying...");
                    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait before retrying
                }
            }
        }
        
        void messageRead(const geometry_msgs::msg::PoseStamped &msg)
        {
          //TODO:call back function once receive camera data, then send to "move_to_block"
          current_pose = msg;
          RCLCPP_INFO(this->get_logger(), "Updated current pose: [%f, %f, %f]", 
                     current_pose.pose.position.x,
                     current_pose.pose.position.y,
                     current_pose.pose.position.z);
          simple_test(msg);
        }
        double radiansToDegrees(double radians) 
        {
          return radians * 180.0 / M_PI_4;
        }
        void applyLinearOrientationConstraint(const geometry_msgs::msg::PoseStamped& current_pose,
                                      const geometry_msgs::msg::PoseStamped& target_pose) 
        {
          // Create a position constraint for a line
          moveit_msgs::msg::PositionConstraint line_constraint;
          line_constraint.header.frame_id = move_group_interface->getPoseReferenceFrame();
          line_constraint.link_name = move_group_interface->getEndEffectorLink();
          // Define the line as a box
          shape_msgs::msg::SolidPrimitive line;
          line.type = shape_msgs::msg::SolidPrimitive::BOX;
          line.dimensions = {0.005, 0.005, 1.0}; // Width, height, length of the box
          line_constraint.constraint_region.primitives.emplace_back(line);
          // Calculate orientation
          tf2::Quaternion myQuaternion;
          auto vectorX = target_pose.pose.position.x - current_pose.pose.position.x;
          auto vectorY = target_pose.pose.position.y - current_pose.pose.position.y;
          auto vectorZ = target_pose.pose.position.z - current_pose.pose.position.z;
          // Calculate angles for quaternion
          double angleX = atan2(vectorY, vectorX);
          double angleY = atan2(vectorZ, sqrt(vectorX * vectorX + vectorY * vectorY));
          double angleZ = atan2(vectorY, vectorZ);
          angleX = radiansToDegrees(angleX);
          angleY = radiansToDegrees(angleY);
          angleZ = radiansToDegrees(angleZ);          
          myQuaternion.setRPY(angleX, angleZ, angleY);
          // Set the line pose
          geometry_msgs::msg::Pose line_pose;
          line_pose.position = current_pose.pose.position;
          line_pose.orientation.x = myQuaternion.getX();
          line_pose.orientation.y = myQuaternion.getY();
          line_pose.orientation.z = myQuaternion.getZ();
          line_pose.orientation.w = myQuaternion.getW();         
          line_constraint.constraint_region.primitive_poses.emplace_back(line_pose);
          line_constraint.weight = 1.0;
          // Create the constraints message
          moveit_msgs::msg::Constraints line_constraints;
          line_constraints.position_constraints.emplace_back(line_constraint);
          line_constraints.name = "use_equality_constraints";
          // Set the path constraints in move_group_interface
          move_group_interface->setPathConstraints(line_constraints);
        }

        /*
        moveit_msgs::msg::RobotTrajectory computeStraightLinePath(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::PoseStamped& target_pose,
        double eef_step = 0.01,
        double jump_threshold = 0.0)
        {
            std::vector<geometry_msgs::msg::Pose> waypoints; // Define a vector for poses
            waypoints.push_back(current_pose.pose);  // Add the current pose
            waypoints.push_back(target_pose.pose);   // Add the target pose

            // Compute the Cartesian path
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = 0.0;
            while (fraction < 1.0) 
            {
              
              fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

              
              if (fraction == 1.0) {
                  RCLCPP_INFO(this->get_logger(), "Cartesian path computed successfully.");
                  return trajectory; 
              } else {
                  RCLCPP_WARN(this->get_logger(), "Could not compute Cartesian path for entire distance, fraction achieved: %f", fraction);
                  
                  std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
              }
            }

          return trajectory; 
      }
        void executeLinearMove(const geometry_msgs::msg::PoseStamped &msg) const 
        {
          //TODO NEW CHECK FOR LEGAL POSE
          if (true) {

            RCLCPP_INFO(this->get_logger(), "Starting linear move");
            
            // Cartesian Paths
            std::vector<geometry_msgs::msg::Pose> waypoints;
            geometry_msgs::msg::PoseStamped targetPose1 = msg;
            waypoints.push_back(targetPose1.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            //double fraction = move_group_interface->computeCartesianPath(waypoints, 0.01, 0.0, planMessage.trajectory_);
            double fraction = move_group_interface->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

            RCLCPP_INFO(this->get_logger(),"Visualising cartesian path, (%.2f%% achieved)", fraction*100.0);

            //Execute movement to point 1
            if (fraction == 1) {
              move_group_interface->execute(trajectory);
              RCLCPP_INFO(this->get_logger(), "Done moving");
            } else {
              std::cout <<  "Planning failed! \nExecuting free move" << std::endl;
              //plan_and_execute(msg);
            }
          } 
        }*/
        void generateJengaTower()
        {
          for (int layer = 1; layer <= 18; layer++) 
          { // A total of 18 layers
              for (int block = 0; block < 3; block++) { // 3 blocks per layer
                  float x, y;
                  float z = jenga_height * layer; // Height based on the layer number
                  if (layer % 2 == 0) 
                  { // Even layers with blocks arranged along the X-axis
                      x = x_first_block_center_point + jenga_width - block * jenga_width; // Each block length is 0.075
                      y = y_first_block_center_point + jenga_width; // Y position
                      auto jenga_block = generateCollisionObject(
                          jenga_width, jenga_length, jenga_height, // Swapped length and width for even layers
                          x, y, z,
                          frame_id, "jenga_block_" + std::to_string(layer * 3 + block) // Unique ID for each block
                      );
                      planning_scene_interface->applyCollisionObject(jenga_block);
                  } 
                  else { // Odd layers with blocks arranged along the Y-axis, rotated 90 degrees
                      x = x_first_block_center_point; // Fixed X position for odd layers
                      y = y_first_block_center_point + block * jenga_width; // Y position based on block index
                      auto jenga_block = generateCollisionObject(
                          jenga_length, jenga_width, jenga_height, // Normal length and width for odd layers
                          x, y, z,
                          frame_id, "jenga_block_" + std::to_string(layer * 3 + block) // Unique ID for each block
                      );
                      planning_scene_interface->applyCollisionObject(jenga_block);
                  }
              }
          }
        } 
             
};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //generate the tower
  rclcpp::spin(std::make_shared<arm_control_test>());
  rclcpp::shutdown();
  return 0;
}
