/**
 * @file   nmpc_bluerov2_2-D_main.cpp
 * @author Mohit Mehindratta / Hakim Amer
 * @date   Jan 2023
 *
 */

#include <nmpc_bluerov2_2D_main.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
using namespace std;
using namespace Eigen;
using namespace ros;
double sampleTime = 0.02;

mavros_msgs::State current_state_msg;


std::vector<std::vector<double>> generateIntermediatePoints(const std::vector<double>& current_pos, const std::vector<double>& goal_pos, double step_distance) {
    std::vector<std::vector<double>> intermediate_points;

    // Calculate the distance between the current position and the goal
    double distance = std::sqrt(std::pow(goal_pos[0] - current_pos[0], 2) +
                                std::pow(goal_pos[1] - current_pos[1], 2) +
                                std::pow(goal_pos[2] - current_pos[2], 2));

    // If the distance is greater than the step distance, generate intermediate points
    if (distance > step_distance) {
        int num_steps = static_cast<int>(distance / step_distance);
        for (int i = 1; i <= num_steps; ++i) {
            double t = i * step_distance / distance;
            std::vector<double> intermediate_point = {
                current_pos[0] + t * (goal_pos[0] - current_pos[0]),
                current_pos[1] + t * (goal_pos[1] - current_pos[1]),
                current_pos[2] + t * (goal_pos[2] - current_pos[2])
            };
            intermediate_points.push_back(intermediate_point);
        }
    }

    return intermediate_points;
}



void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_msg = *msg;
}
void ref_position_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_position << msg->x, msg->y, msg->z;
}
void ref_velocity_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_velocity << msg->x, msg->y, msg->z;
}
void ref_yaw_cb(const std_msgs::Float64::ConstPtr& msg)
{
    ref_yaw_rad = msg->data;
}

void pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    double roll = 0, pitch = 0, yaw = 0;
    current_att_quat = {
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
    current_vel_rate = {msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z,
                        msg->twist.twist.angular.x,
                        msg->twist.twist.angular.y,
                        msg->twist.twist.angular.z};


    current_att_mat.setRotation(current_att_quat);
    //current_att_mat.getRPY(roll, pitch, yaw);
    current_att_mat.getRPY(pitch, roll, yaw);
    current_pos_att = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, roll, pitch, yaw};
}



void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel_body = {msg->twist.linear.x,
                        msg->twist.linear.y,
                        msg->twist.linear.z,
                        msg->twist.angular.x,
                        msg->twist.angular.y,
                        msg->twist.angular.z};
}


void rope_traj_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    rope_goal[0] = msg->pose.position.x;
    rope_goal[1] = msg->pose.position.y;
    rope_goal[2] = msg->pose.position.z;

    // Get the quaternion from the message
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );

    // Convert quaternion to roll, pitch, yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Assign the yaw angle to rope_goal[3]
    rope_goal[3] = yaw;
}


void orientation_cb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{   
    
    angles = {msg->vector.x*(M_PI/180),
                   msg->vector.y*(M_PI/180),
                   msg->vector.z*(M_PI/180)};   
    angles_d ={msg->vector.x,
                   msg->vector.y,
                   msg->vector.z}; 
}


// Disturbance estimator Call back functions X, Y,Z

void dist_Fx_predInit_cb(const std_msgs::Bool::ConstPtr& msg)
{
    dist_Fx.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fx.predInit && dist_Fx.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fx estimates! \n";
        dist_Fx.print_predInit = 0;
    }
}
void dist_Fy_predInit_cb(const std_msgs::Bool::ConstPtr& msg)
{
    dist_Fy.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fy.predInit && dist_Fy.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fy estimates! \n";
        dist_Fy.print_predInit = 0;
    }
}
void dist_Fz_predInit_cb(const std_msgs::Bool::ConstPtr& msg)
{
    dist_Fz.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fz.predInit && dist_Fz.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fz estimates! \n";
        dist_Fz.print_predInit = 0;
    }
}

void dist_Fx_data_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (use_dist_estimates && dist_Fx.predInit)
    {
        dist_Fx.data.clear();
        dist_Fx.data.insert(dist_Fx.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        dist_Fx.data = dist_Fx.data_zeros;
}
void dist_Fy_data_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (use_dist_estimates && dist_Fy.predInit)
    {
        dist_Fy.data.clear();
        dist_Fy.data.insert(dist_Fy.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        dist_Fy.data = dist_Fy.data_zeros;
}
void dist_Fz_data_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (use_dist_estimates && dist_Fz.predInit)
    {
        dist_Fz.data.clear();
        dist_Fz.data.insert(dist_Fz.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        dist_Fz.data = dist_Fz.data_zeros;
}

void NMPC_PC::publish_wrench(struct command_struct& commandstruct)
{

    geometry_msgs::Wrench nmpc_wrench_msg;

    
    
    nmpc_wrench_msg.force.x =    commandstruct.control_wrench_vec[0];
    nmpc_wrench_msg.force.y =    commandstruct.control_wrench_vec[1];
    nmpc_wrench_msg.force.z =    commandstruct.control_wrench_vec[2];

    nmpc_wrench_msg.torque.x =    0.0;
    nmpc_wrench_msg.torque.y =    0.0;
    nmpc_wrench_msg.torque.z =   commandstruct.control_wrench_vec[3];

    
    // nmpc_wrench_msg.force.x =    0.0;
    // nmpc_wrench_msg.force.y =    0.0;
    // nmpc_wrench_msg.force.z =    commandstruct.control_wrench_vec[2];

    //  nmpc_wrench_msg.torque.x =    0.0;
    //  nmpc_wrench_msg.torque.y =    0.0;
    // nmpc_wrench_msg.torque.z =   0.0;


    nmpc_cmd_wrench_pub.publish(nmpc_wrench_msg);

    std_msgs::Float64 exe_time_msg;
    exe_time_msg.data = commandstruct.exe_time;
    nmpc_cmd_exeTime_pub.publish(exe_time_msg);

    std_msgs::Float64 kkt_tol_msg;
    kkt_tol_msg.data = commandstruct.kkt_tol;
    nmpc_cmd_kkt_pub.publish(kkt_tol_msg);

    std_msgs::Float64 obj_val_msg;
    obj_val_msg.data = commandstruct.obj_val;
    nmpc_cmd_obj_pub.publish(obj_val_msg);
    

}


void NMPC_PC::publish_pred_tarjectory(struct acado_struct& traj_struct)
{
      // Create an instance of the Float32MultiArray message type
    std_msgs::Float64MultiArray pred_traj_msg;

    // Resize the data array based on the size of nmpc_pc->nmpc_struct.x
   pred_traj_msg.data.resize(NMPC_NX * (NMPC_N + 1));


       for (int i = 0; i < NMPC_NX * (NMPC_N + 1); ++i)
    {
       // pred_traj_msg.data[i] = traj_struct.x[i];
        pred_traj_msg.data[i] =  nmpc_struct.x[0+9];
    }
   

  nmpc_pred_traj_pub.publish(pred_traj_msg);
  
    // a = nmpc_pc->nmpc_struct.x[0+9] <<endl;
 
}


int main(int argc, char** argv)
{


    ros::init(argc, argv, "bluerov2_nmpc_node");
    ros::NodeHandle nh;

    ros::param::get("mocap_topic_part", mocap_topic_part);
    ros::param::get("dist_Fx_predInit_topic", dist_Fx_predInit_topic);
    ros::param::get("dist_Fy_predInit_topic", dist_Fy_predInit_topic);
    ros::param::get("dist_Fz_predInit_topic", dist_Fz_predInit_topic);
    ros::param::get("dist_Fx_data_topic", dist_Fx_data_topic);
    ros::param::get("dist_Fy_data_topic", dist_Fy_data_topic);
    ros::param::get("dist_Fz_data_topic", dist_Fz_data_topic);



    // ----------
    // Subscribers
    // ----------

    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    ref_position_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/position", 1, ref_position_cb);
    ref_velocity_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/velocity", 1, ref_velocity_cb);
    ref_yaw_sub = nh.subscribe<std_msgs::Float64>("ref_trajectory/yaw", 1, ref_yaw_cb);
    pos_sub = nh.subscribe<nav_msgs::Odometry>("/qualisys/bluerov2/odom", 1, pos_cb);
    //vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/mocap/velocity_body", 1, vel_cb);
    dist_Fx_predInit_sub = nh.subscribe<std_msgs::Bool>(dist_Fx_predInit_topic, 1, dist_Fx_predInit_cb);
    dist_Fy_predInit_sub = nh.subscribe<std_msgs::Bool>(dist_Fy_predInit_topic, 1, dist_Fy_predInit_cb);
    dist_Fz_predInit_sub = nh.subscribe<std_msgs::Bool>(dist_Fz_predInit_topic, 1, dist_Fz_predInit_cb);
    dist_Fx_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(dist_Fx_data_topic, 1, dist_Fx_data_cb);
    dist_Fy_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(dist_Fy_data_topic, 1, dist_Fy_data_cb);
    dist_Fz_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(dist_Fz_data_topic, 1, dist_Fz_data_cb);
    orientation_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/mobula/rov/orientation", 1, orientation_cb);
    rope_traj_sub = nh.subscribe<geometry_msgs::PoseStamped>("/ropeplanner_goal", 1, rope_traj_cb);

    // ----------
    // Publishers
    // ----------
    nmpc_cmd_wrench_pub = nh.advertise<geometry_msgs::Wrench>("/mobula/rov/wrench", 1, true);
    nmpc_cmd_exeTime_pub = nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/exeTime", 1, true);
    nmpc_cmd_kkt_pub = nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/kkt", 1, true);
    nmpc_cmd_obj_pub = nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/obj", 1, true);

    nmpc_pred_traj_pub = nh.advertise<std_msgs::Float64MultiArray>("nmpc_predicted_trajectory", 1, true); 
    
    s_sdot_pub = nh.advertise<std_msgs::Float64MultiArray>("outer_nmpc_cmd/s_sdot", 1, true);

    nmpc_struct.U_ref.resize(NMPC_NU);
    nmpc_struct.W.resize(NMPC_NY);

    // Roslaunch parameters
    ros::param::get("verbose", nmpc_struct.verbose);
    ros::param::get("yaw_control", nmpc_struct.yaw_control);
    ros::param::get("online_ref_yaw", online_ref_yaw);
    ros::param::get("use_dist_estimates", use_dist_estimates);


    ros::param::get("W_Wn_factor", nmpc_struct.W_Wn_factor);
    int u_idx = 0;
    ros::param::get("F_x_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("F_y_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("F_z_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("Mz_ref", nmpc_struct.U_ref(u_idx++));

    assert(u_idx == NMPC_NU);

    int w_idx = 0;
    ros::param::get("W_x", nmpc_struct.W(w_idx++));
    ros::param::get("W_y", nmpc_struct.W(w_idx++));
    ros::param::get("W_z", nmpc_struct.W(w_idx++));
    ros::param::get("W_u", nmpc_struct.W(w_idx++));
    ros::param::get("W_v", nmpc_struct.W(w_idx++));
    ros::param::get("W_w", nmpc_struct.W(w_idx++));
    ros::param::get("W_psi", nmpc_struct.W(w_idx++));
    ros::param::get("W_r", nmpc_struct.W(w_idx++));
    ros::param::get("W_Fx", nmpc_struct.W(w_idx++));
    ros::param::get("W_Fy", nmpc_struct.W(w_idx++));
    ros::param::get("W_Fz", nmpc_struct.W(w_idx++));
    ros::param::get("W_Mz", nmpc_struct.W(w_idx++));
    assert(w_idx == NMPC_NY);

    nmpc_struct.sample_time = sampleTime;

    NMPC_PC* nmpc_pc = new NMPC_PC(nmpc_struct);
    ros::Rate rate(1 / sampleTime);

    current_pos_att.resize(6);
    current_vel_rate.resize(6);
    current_vel_body.resize(6);
    dist_Fx.data.resize(NMPC_N + 1);
    dist_Fy.data.resize(NMPC_N + 1);
    dist_Fz.data.resize(NMPC_N + 1);
    dist_Fx.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fy.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fz.data_zeros.resize(NMPC_N + 1, 0.0);

    ref_traj_type = 0;
    ref_position << 0.8, 3.0, -2.0;
    ref_velocity << 0, 0, 0;

     angles = { 0,0,0};
    rope_goal = std::vector<double>(4, 0.0);
    control_stop = false;

    for (int i = 0; i < (int)(1 / sampleTime); ++i)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // Delay loop
    ros::Time start_time = ros::Time::now();
    while (ros::Time::now() - start_time < ros::Duration(2.0)) {
        geometry_msgs::Wrench zero_wrench_msg;
        zero_wrench_msg.force.x = 0.0;
        zero_wrench_msg.force.y = 0.0;
        zero_wrench_msg.force.z = 0.0;
        zero_wrench_msg.torque.x = 0.0;
        zero_wrench_msg.torque.y = 0.0;
        zero_wrench_msg.torque.z = 0.0;
        nmpc_cmd_wrench_pub.publish(zero_wrench_msg);
        ros::Duration(0.01).sleep(); // Sleep for 10 milliseconds
    }

    

    while (ros::ok() && !control_stop)
    {
        t = ros::Time::now().toSec();

        if (current_state_msg.mode != "OFFBOARD" && print_flag_offboard == 1)
        {
            ROS_INFO("OFFBOARD mode is not enabled!");
            print_flag_offboard = 0;
        }
        if (!current_state_msg.armed && print_flag_arm == 1)
        {
            ROS_INFO("Vehicle is not armed!");
            print_flag_arm = 2;
        }
        else if (current_state_msg.armed && print_flag_arm == 2)
        {
            ROS_INFO("Vehicle is armed!");
            print_flag_arm = 0;
        }

        if (current_state_msg.mode == "ALTCTL")
        {
            pos_ref = current_pos_att;
            if (print_flag_altctl == 1)
            {
                ROS_INFO("ALTCTL mode is enabled!");
                print_flag_altctl = 0;
            }
        }

        if (!nmpc_pc->return_control_init_value())
        {
            nmpc_pc->nmpc_init(pos_ref, nmpc_pc->nmpc_struct);
            if (nmpc_struct.verbose && nmpc_pc->return_control_init_value())
            {
                std::cout << "***********************************\n";
                std::cout << "NMPC: initialized correctly\n";
                std::cout << "***********************************\n";
            }
        }

        while (ros::ok() && !control_stop)
        {

            t_cc_loop = ros::Time::now().toSec() - t;
            if (std::fmod(std::abs(t_cc_loop - (int)(t_cc_loop)), (double)(sampleTime)) == 0)
                std::cout << "loop time for outer NMPC: " << t_cc_loop << " (sec)"
                          << "\n";

            // Setting up state-feedback [x,y,z,u,v,w,psi,r]
            current_states = {current_pos_att.at(0),
                              -current_pos_att.at(1),
                              -current_pos_att.at(2),
                              current_vel_rate.at(0),
                              current_vel_rate.at(1),
                              current_vel_rate.at(2),
                              -current_pos_att.at(5),
                              current_vel_rate.at(5)
                              };


          std::vector<std::vector<double>> intermediate_points = generateIntermediatePoints(
            {current_pos_att[0], current_pos_att[1], current_pos_att[2]}, // Current position
            {rope_goal[0], rope_goal[1], rope_goal[2]},                  // Goal position
            0.5 // Step distance (0.5 meters)
        );
        
        if (!intermediate_points.empty()) {
            // Use the first intermediate point as the next reference position
            ref_position[0] = intermediate_points[0][0];
            ref_position[1] = intermediate_points[0][1];
            ref_position[2] = intermediate_points[0][2];
        } else {
            // If no intermediate points are needed, use the rope goal directly
            ref_position[0] = rope_goal[0];
            ref_position[1] = rope_goal[1];
            ref_position[2] = rope_goal[2];
        }  



                    ref_trajectory = {ref_position[0],  //x
                                      ref_position[1],  //y
                                      ref_position[2],   //z
                                      ref_velocity[0],   //u
                                      ref_velocity[1],   //v
                                      ref_velocity[2],   //w
                                      -ref_yaw_rad,
                                      0.0
                             };                   




            std::cout << "current_states = ";
            for (int idx = 0; idx < current_states.size(); idx++)
            {
                std::cout << current_states[idx] << ",";
            }
            std::cout << "\n";

            std::cout << "ref_trajectory = ";
            for (int idx = 0; idx < ref_trajectory.size(); idx++)
            {
                std::cout << ref_trajectory[idx] << ",";
            }
            std::cout << "\n";

            
            std::cout << "ref  yaw = "<< ref_yaw_rad << std::endl ;
            std::cout << "current yaw = "<<         current_pos_att.at(5) << std::endl ;

            std::cout << "hakim=122 " << std::endl ;




           double F_d_max = 50;

          //  online_data.distFx = std::min(dist_Fx.data, F_d_max);
          //  online_data.distFy = std::min(dist_Fy.data, F_d_max);
          //  online_data.distFz = std::min(dist_Fz.data, F_d_max);
          //cap disturbance to F_dmax
                for (size_t i = 0; i < dist_Fx.data.size(); ++i) {
                    dist_Fx.data[i] = std::min(std::max(dist_Fx.data[i], -F_d_max), F_d_max);
                    dist_Fy.data[i] = std::min(std::max(dist_Fy.data[i], -F_d_max), F_d_max);
                    dist_Fz.data[i] = std::min(std::max(dist_Fz.data[i], -F_d_max), F_d_max);
                }


                
           online_data.distFx = dist_Fx.data ;
           online_data.distFy = dist_Fy.data ;
           online_data.distFz = dist_Fx.data_zeros;


       //   ROS_ERROR_STREAM("online_data = " << online_data.distFx[0] << " (sec)");
       //   ROS_ERROR_STREAM("online_data = " << online_data.distFy[0] << " (sec)");
        //  ROS_ERROR_STREAM("online_data = " << online_data.distFz[0] << " (sec)");
          std::cout << "\033[1;31m" << "online_data = " << online_data.distFx[0] << " (sec)" << "\033[0m" << std::endl;
          std::cout << "\033[1;31m" << "online_data = " << online_data.distFy[0] << " (sec)" << "\033[0m" << std::endl;
          std::cout << "\033[1;31m" << "online_data = " << online_data.distFz[0] << " (sec)" << "\033[0m" << std::endl;


            nmpc_pc->nmpc_core(nmpc_struct,
                               nmpc_pc->nmpc_struct,
                               nmpc_pc->nmpc_cmd_struct,
                               ref_trajectory,
                               online_data,
                               current_states);

            if (nmpc_pc->acado_feedbackStep_fb != 0)
                control_stop = true;

            if (std::isnan(nmpc_pc->nmpc_struct.u[0]) == true || std::isnan(nmpc_pc->nmpc_struct.u[1]) == true ||
                std::isnan(nmpc_pc->nmpc_struct.u[2]) == true || std::isnan(nmpc_pc->nmpc_struct.u[3]) == true)
            {
                ROS_ERROR_STREAM("Controller ERROR at time = " << ros::Time::now().toSec() - t << " (sec)");
                control_stop = true;
                exit(0);
            }

            std::cout << "predicted dist x "<< online_data.distFx[0]<<endl;
           /// std::cout << "thrust input x1"<< nmpc_pc->nmpc_struct.x[1+9]<<endl;

            nmpc_pc->publish_pred_tarjectory(nmpc_pc->nmpc_struct);
            nmpc_pc->publish_wrench(nmpc_pc->nmpc_cmd_struct);

            print_flag_offboard = 1;
            print_flag_arm = 1;
            print_flag_altctl = 1;

            ros::spinOnce();
            rate.sleep();
        }
        



        
        nmpc_pc->publish_wrench(nmpc_pc->nmpc_cmd_struct);
        



        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
