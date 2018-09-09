
#include <vin_mission_control.h>

#define PI 3.14159265

using namespace std;
using namespace Eigen;

/*flags for detection of msgs and threshold check*/
float x = 0, y = 0, x_des = 0, y_des = 0, err_sum_x = 0.0, err_sum_y = 0.0, yaw_sp = 5.7, err_sum_pos_x = 0, err_sum_pos_y = 0;
float vel_x = 0, vel_y = 0, vel_thresh = 1.0, vel_sp_x = 0, z_dist, att_sp_thresh = 0.3, traj_sp_threshold = 0.08;
float vel_sp_y = 0, object_x, object_y, pick_goal_x_cb, pick_goal_y_cb, landing_threshold = 0.1, yaw_init = 5.7;
int index_x = 0, index_y = 0, yaw_reset = 0, off_flag = 1, grip_status = 0, trajectory_size = 0, take_off_flag = 0, distcb_count = 0;
double x_dist = 2.0, quad_yaw, yaw_set, yaw_traj, yaw_marker, yaw_sp_temp, odom_yaw_init, odom_yaw, yaw_diff_init;
double imu_yaw;
int vel_cross_flag = 0, cross_flag = 0, aruco_detected_flag = 0;

float landing_time = 7;
float takeoff_time = 8;
float landing_time_threshold = 4;
float yaw_alignment_time = 4;
float landing_height = 0.3;
float gripping_sleep_time = 3;
float land_mode_sleep_time = 3;
string mode_;

tf::Quaternion q;
geometry_msgs::PoseStamped mocap, setpoint, vel_sp, pos_sp, goal_sp,vel_;
geometry_msgs::PoseArray traj, frontiers_;
std_msgs::Int32 gripper_pos, mission_reset_flag;
nav_msgs::Odometry odom_;
geometry_msgs::Point32 goal_;
Quaternionf quat;

Matrix<float, 3, 3> R;
Matrix<float, 3, 3> pos;
Matrix<float, 3, 3> R_inv;
Matrix<float, 3, 1> vel_world;
Matrix<float, 3, 1> vel_quad;
Matrix<float, 3, 1> vel_sp_world;
Matrix<float, 3, 1> vel_sp_quad;
Matrix<float, 3, 1> rpy;
geometry_msgs::Quaternion quaternion;
//Quaternionf quat;

 
void statecb(const mavros_msgs::State::ConstPtr &msg);
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
void arucocb(const geometry_msgs::PoseStamped::ConstPtr &msg);
//void flowcb(const px_comm::OpticalFlow::ConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pidcontrol");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 100, statecb);
    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 100, imuCallback);
    ros::Subscriber aruco_sub = nh.subscribe("/aruco_single/pose", 10, arucocb);
    //ros::Subscriber flow_sub = nh.subscribe("/px4flow/opt_flow", 10, flowcb);
   
    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher mocap_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 10);
    ros::Publisher vel_sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/velocity_sp", 10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::PoseStamped>("/velocity", 10);
    ros::Publisher pos_sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/position_sp", 10);

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode land_set_mode, offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    land_set_mode.request.custom_mode = "AUTO.LAND";

    mission_reset_flag.data = 0;

    double timer_ = 0, timer_land = 0;
    float vel_x_prev, vel_y_prev, x_prev, y_prev, set_alt_temp = 0.8;
    int i = 0;
    float x_temp, y_temp;

    ros::Rate loop_rate(15);

    while (ros::ok())
    {
        float pos_k_p_x, pos_k_p_y, pos_k_d, pos_k_i, vel_x_k_p, vel_x_k_i, vel_x_k_d, vel_y_k_p, vel_y_k_i, vel_y_k_d, set_alt;
        nh.getParam("/vin_mission_control/pos_k_p_x", pos_k_p_x);
        nh.getParam("/vin_mission_control/pos_k_p_y", pos_k_p_y);
        nh.getParam("/vin_mission_control/vel_x_k_p", vel_x_k_p);
        nh.getParam("/vin_mission_control/vel_x_k_d", vel_x_k_d);
        nh.getParam("/vin_mission_control/vel_y_k_p", vel_y_k_p);
        nh.getParam("/vin_mission_control/vel_y_k_d", vel_y_k_d);
        nh.getParam("/vin_mission_control/set_alt", set_alt);
        nh.getParam("/vin_mission_control/yaw_reset", yaw_reset);

        mocap.header.stamp = ros::Time::now();
        setpoint.header.stamp = ros::Time::now();
        vel_sp.header.stamp = ros::Time::now();
        vel_.header.stamp = ros::Time::now();
        pos_sp.header.stamp = ros::Time::now();
    
        geometry_msgs::TransformStamped mav_in_world;
        

        if (aruco_detected_flag == 1 )
        {
            pos(0,0)=x;
            pos(1,0)=y;
            pos(2,0)=0;

            R_inv = R.inverse();
            pos = R_inv*pos ;

            cout<<x<<"  "<<y<<"    "<<pos(0,0)<<"    "<<pos(1,0)<<endl;

            if (i == 0)
            {
                x_prev = pos(0,0);
                y_prev = pos(1,0);
            }
            else
            {
                vel_x = (pos(0,0) - x_prev) * 15;
                vel_y = (pos(1,0) - y_prev) * 15;
                //cout<<vel_x<<"    "<<vel_y<<endl<<endl;

            }

            x_prev = pos(0,0);
            y_prev = pos(1,0);

            if (i == 1)
            {
                vel_x_prev = vel_x;
                vel_y_prev = vel_y;
            }
            else if (i>1)
            {
                err_sum_pos_x = err_sum_pos_x + (x_des - pos(0,0));
                err_sum_pos_y = err_sum_pos_y + (y_des - pos(1,0));

                if (mode_ == "OFFBOARD")
                {
                    nh.getParam("/vin_mission_control/pos_k_i", pos_k_i);
                }
                else
                {
                    pos_k_i = 0;
                    err_sum_pos_y = 0;
                    err_sum_pos_x = 0;
                }

                vel_sp_x = (x_des - pos(0,0)) * pos_k_p_x + (err_sum_pos_x)*0.015 * pos_k_i;
                vel_sp_y = (y_des - pos(1,0)) * pos_k_p_y + (err_sum_pos_y)*0.015 * pos_k_i;
                

                err_sum_x = err_sum_x + (vel_x - vel_sp_x);
                err_sum_y = err_sum_y + (vel_y - vel_sp_y);

                if (mode_ == "OFFBOARD")
                {
                    nh.getParam("/vin_mission_control/vel_x_k_i", vel_x_k_i);
                    nh.getParam("/vin_mission_control/vel_y_k_i", vel_y_k_i);
                }
                else
                {
                    vel_x_k_i = 0;
                    vel_y_k_i = 0;
                    err_sum_y = 0;
                    err_sum_x = 0;
                }

                // vel_sp_y = -0.0f;
                // vel_sp_x = 0.0f;

                if (vel_sp_x < -vel_thresh || vel_sp_x > vel_thresh || vel_sp_y < -vel_thresh || vel_sp_y > vel_thresh)
                    vel_cross_flag = 1;

                if (vel_sp_y > vel_thresh)
                    vel_sp_y = vel_thresh;
                else if (vel_sp_y < -vel_thresh)
                    vel_sp_y = -vel_thresh;

                if (vel_sp_x > vel_thresh)
                    vel_sp_x = vel_thresh;
                else if (vel_sp_x < -vel_thresh)
                    vel_sp_x = -vel_thresh;

                if (vel_cross_flag == 1)
                    cout << "vel Threshold reached" << endl;

                mocap.pose.position.y = ((vel_y - vel_sp_y) * vel_y_k_p + (err_sum_y)*0.015 * vel_y_k_i + (vel_y - vel_y_prev) * 15 * vel_y_k_d); //roll
                mocap.pose.position.x = ((vel_x - vel_sp_x) * vel_x_k_p + (err_sum_x)*0.015 * vel_x_k_i + (vel_x - vel_x_prev) * 15 * vel_x_k_d); //pitch

                vel_sp.pose.position.x = vel_sp_x;
                vel_sp.pose.position.y = vel_sp_y;
                vel_.pose.position.x = vel_x;
                vel_.pose.position.y = vel_y;
            }

            vel_x_prev = vel_x;
            vel_y_prev = vel_y;

            i = i + 1;
            //rpy = R.eulerAngles(0,1,2);
            //quad_yaw = (double)rpy(2,0);

            //q.setRPY(0, 0, quad_yaw);

            //setpoint.pose.orientation.z = q.z();
            //setpoint.pose.orientation.w = q.w();

            if (mocap.pose.position.x < -att_sp_thresh || mocap.pose.position.x > att_sp_thresh || mocap.pose.position.y < -att_sp_thresh || mocap.pose.position.y > att_sp_thresh)
                cross_flag = 1;

            if (mocap.pose.position.x > att_sp_thresh)
                mocap.pose.position.x = att_sp_thresh;
            else if (mocap.pose.position.x < -att_sp_thresh)
                mocap.pose.position.x = -att_sp_thresh;
            if (mocap.pose.position.y < -att_sp_thresh)
                mocap.pose.position.y = -att_sp_thresh;
            else if (mocap.pose.position.y > att_sp_thresh)
                mocap.pose.position.y = att_sp_thresh;

            if (cross_flag == 1)
                cout << "Attitude Threshold reached" << endl;
            
            mocap.pose.position.z = z_dist;
            mocap_pub.publish(mocap);
            
            aruco_detected_flag = 0;
        }

        setpoint_pub.publish(setpoint);
        vel_sp_pub.publish(vel_sp);
        vel_pub.publish(vel_);
        pos_sp_pub.publish(pos_sp);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


void statecb(const mavros_msgs::State::ConstPtr &msg)
{
    mode_ = msg->mode;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf::Quaternion q1(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q1);
    double r, p;
    m.getRPY(r, p, imu_yaw);
    //cout<<r*180/3.14<<"   "<<p*180/3.14<<"   "<<imu_yaw*180/3.14<<endl<<endl;
    tf::Quaternion q_temp;
    q_temp.setRPY(r, p, 0);
     
    //tf::Matrix3x3 R(q);
    //cout<<"q"<<q_temp.w()<<endl;
  
  quat=Eigen::Quaternionf(q_temp.w(),q_temp.x(),q_temp.y(),q_temp.z());
  R=quat.toRotationMatrix();

    
}

void arucocb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    x = msg->pose.position.x;
    y = -1*msg->pose.position.y;
    aruco_detected_flag = 1;
}

/*void flowcb(const px_comm::OpticalFlow::ConstPtr &msg)
{
    double x_dist_temp = msg->ground_distance;
    double x_dist_median[16];

    for (int i = 0; i < 16; i++)
        x_dist_median[i] = x_dist_temp;

    sort(x_dist_median, x_dist_median + 10);

    if ((x_dist_temp > (x_dist_median[8] - 0.3)) && (x_dist_temp < (x_dist_median[8] + 0.3)))
        x_dist = x_dist_temp;

    if (x_dist < 0.1)
        x_dist = 2.0;
    flow_flag = 1;
}*/

