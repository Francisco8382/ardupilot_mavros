#include <ros/ros.h>
#include <ros/console.h>
#include <mavros_msgs/StreamRate.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define STREAM_ID           6
#define MESSAGE_RATE        100
#define CONECCTION_RATE     100
#define ARMING_RATE         1
#define TAKEOFF_ALT         1.0

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void waitForConnection(int rate){
    ros::Rate r(rate);
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        r.sleep();
    }
}

bool setStreamRate(ros::ServiceClient& stream_rate_client, int stream_id, int message_rate, bool on_off){
    mavros_msgs::StreamRate srv;
    srv.request.stream_id = stream_id;
    srv.request.message_rate = message_rate;
    srv.request.on_off = on_off;
    return stream_rate_client.call(srv);
}

bool setModeGuided(ros::ServiceClient& set_mode_client){
    mavros_msgs::SetMode srv;
    srv.request.base_mode = 0;
    srv.request.custom_mode = "GUIDED";
    return set_mode_client.call(srv);
}

bool arm(ros::ServiceClient& arming_client, int rate){
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    ros::Rate r(rate);
    while(ros::ok() && !current_state.armed && arming_client.call(srv)){
        ros::spinOnce();
        r.sleep();
    }
    return current_state.armed;
}

bool takeoff(ros::ServiceClient &takeoff_client, float altitude){
    mavros_msgs::CommandTOL srv;
    srv.request.altitude = altitude;
    srv.request.latitude = 0;
    srv.request.longitude = 0;
    srv.request.min_pitch = 0;
    srv.request.yaw = 0;
    return takeoff_client.call(srv);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ardupilot_mavros_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");
    ros::ServiceClient stream_rate_client = nh.serviceClient<mavros_msgs::StreamRate>
        ("/mavros/set_stream_rate");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
        ("/mavros/cmd/takeoff"); 

    waitForConnection(CONECCTION_RATE);
    if (setStreamRate(stream_rate_client, STREAM_ID, MESSAGE_RATE, true)){
        ROS_INFO("Stream rate for position messages has been set to: %d", MESSAGE_RATE);
    }
    else {
        ROS_INFO("Failed to set stream rate");
        return -1;
    }

    if (setModeGuided(set_mode_client)){
        ROS_INFO("Guided mode has been set");
    }
    else {
        ROS_INFO("Failed to set guided mode");
        return -1;
    }
    
    if (arm(arming_client, ARMING_RATE)){
        ROS_INFO("Vehicle armed");
    }
    else {
        ROS_INFO("Failed arming");
        return -1;
    }

    if (takeoff(takeoff_client, TAKEOFF_ALT)){
        ROS_INFO("Takeoff at altitude: %f", TAKEOFF_ALT);
    }
    else {
        ROS_INFO("Failed takeoff");
        return -1;
    }

    return 0;
}
