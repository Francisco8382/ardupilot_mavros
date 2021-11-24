/*
 * Copyright 2021 Francisco Javier Torres Maldonado
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <fstream>
#include <iostream>
#include <string>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
//#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <tf/tf.h>

static const float DEG_2_RAD = M_PI / 180.0;

namespace mav_msgs {
  inline Eigen::Vector4d vector4FromQuaternionMsg(const geometry_msgs::Quaternion& msg) {
    return Eigen::Vector4d(msg.x, msg.y, msg.z, msg.w);
  }
  inline double secsFromHeaderMsg(const std_msgs::Header& msg) {
    return (double) (msg.stamp.sec) + (double) (msg.stamp.nsec)/1.0e9;
  }
}

namespace ardupilot_mavros {

    class CSV_Odometry{
        public:
            CSV_Odometry(std::string Topic, std::string Path, double TiempoMargen);
            ~CSV_Odometry();
            
        private:
            Eigen::Vector3d _Position;
            Eigen::Vector3d _LinearVelocity;
            Eigen::Vector3d _Orientation;
            Eigen::Vector3d _AngularVelocity;
            Eigen::Vector4d _Quaternion;
            double _Tiempo;
            double _TiempoMargen;
            double _TiempoInicio;

            ros::NodeHandle nh;
            ros::Subscriber odometry_sub_;
            ros::Subscriber Begin_;
            ros::Subscriber End_;
            
            bool Trigger = false;
            std::string _Path;
            std::string _Topic;
            std::ofstream _csvFile;

            Eigen::Vector3d Quat2RPY(Eigen::Vector4d Quaternion);
            void BeginCallback(const std_msgs::Empty::ConstPtr& empty_msg);
            void StopCallback(const std_msgs::Empty::ConstPtr& empty_msg);
            void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
    };

    CSV_Odometry::CSV_Odometry(std::string Topic, std::string Path, double TiempoMargen)
    : _Position(Eigen::Vector3d(0.0,0.0,0.0)),
      _LinearVelocity(Eigen::Vector3d(0.0,0.0,0.0)),
      _Orientation(Eigen::Vector3d(0.0,0.0,0.0)),
      _AngularVelocity(Eigen::Vector3d(0.0,0.0,0.0)),
      _Quaternion(Eigen::Vector4d(0.0,0.0,0.0,0.0)),
      _Path(Path),
      _Topic(Topic),
      _TiempoMargen(TiempoMargen),
      _TiempoInicio(0.0) {
        _csvFile.open(_Path);
        _csvFile << "Tiempo,X,Y,Z,Vx,Vy,Vz,Roll,Pitch,Yaw,Vroll,Vpitch,Vyaw\n";
        odometry_sub_ = nh.subscribe(_Topic, 
          1, &CSV_Odometry::OdometryCallback, this, ros::TransportHints().tcpNoDelay(true));
        Begin_ = nh.subscribe<std_msgs::Empty>("/csv/begin", 1, &CSV_Odometry::BeginCallback, this);
        End_ = nh.subscribe<std_msgs::Empty>("/csv/end", 1, &CSV_Odometry::StopCallback, this);
        
    }
    
    CSV_Odometry::~CSV_Odometry() {}

    void CSV_Odometry::BeginCallback(const std_msgs::Empty::ConstPtr& empty_msg) {
      if (!Trigger){
        ROS_INFO("Leyendo mensajes de topico: %s",_Topic.c_str());
      }
      Trigger = true;
    }

    void CSV_Odometry::StopCallback(const std_msgs::Empty::ConstPtr& empty_msg) {
      Trigger = false;
      _csvFile.close();
      ROS_INFO("Escritura de CSV finalizada para el topico: %s",_Topic.c_str());
      ros::shutdown();
    }

    void CSV_Odometry::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
      if (Trigger) {
        ros::param::get("/TiempoInicio", _TiempoInicio);
        _Tiempo = mav_msgs::secsFromHeaderMsg(odometry_msg->header) - _TiempoInicio;
        _Position = mav_msgs::vector3FromPointMsg(odometry_msg->pose.pose.position);
        _LinearVelocity = mav_msgs::vector3FromMsg(odometry_msg->twist.twist.linear);
        _Quaternion = mav_msgs::vector4FromQuaternionMsg(odometry_msg->pose.pose.orientation);
        _AngularVelocity = mav_msgs::vector3FromMsg(odometry_msg->twist.twist.angular);
        _Orientation = Quat2RPY(_Quaternion);
        _csvFile  << _Tiempo << "," 
                  << _Position.x() << ","
                  << _Position.y() << ","
                  << _Position.z() << ","
                  << _LinearVelocity.x() << ","
                  << _LinearVelocity.y() << ","
                  << _LinearVelocity.z() << ","
                  << _Orientation.x() << ","
                  << _Orientation.y() << ","
                  << _Orientation.z() << ","
                  << _AngularVelocity.x() << ","
                  << _AngularVelocity.y() << ","
                  << _AngularVelocity.z() << "\n";

      }
    }

    Eigen::Vector3d CSV_Odometry::Quat2RPY(Eigen::Vector4d Quaternion) {
      double x, y, z, w, roll, pitch, yaw;
      x = Quaternion.x();
      y = Quaternion.y();
      z = Quaternion.z();
      w = Quaternion.w();
      tf::Quaternion q(x, y, z, w);
      tf::Matrix3x3 m(q);
      m.getRPY(roll, pitch, yaw);
      return Eigen::Vector3d(roll, pitch, yaw);
    }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "csv_odometry", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  std::string Topic, Path, DateAndTime, Name, FullPath, File;
  double TiempoMargen;

  ros::param::get("~Topico", Topic);
  ros::param::get("~Ruta", Path);
  ros::param::get("/Subcarpeta", DateAndTime);
  ros::param::get("~TiempoMargen", TiempoMargen);
  ros::param::get("~Nombre", Name);

  DateAndTime.pop_back();
  FullPath = Path + DateAndTime;
  mkdir(FullPath.c_str(), 0777);

  File = FullPath + "/" + Name;

  ardupilot_mavros::CSV_Odometry csv_odometry_(Topic, File, TiempoMargen);

  ros::spin();
  
  return 0;

}

