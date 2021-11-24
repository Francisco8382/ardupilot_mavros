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

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#include <tf/tf.h>

#define FRAME_LOCAL_NED       1

#define IGNORE_PX             1
#define IGNORE_PY             2
#define IGNORE_PZ             4
#define IGNORE_VX             8
#define IGNORE_VY             16
#define IGNORE_VZ             32
#define IGNORE_AX             64
#define IGNORE_AY             128
#define IGNORE_AZ             256
#define IGNORE_YAW            1024
#define IGNORE_YAW_RATE       2048

static const float DEG_2_RAD = M_PI / 180.0;

struct Parametros {
  double TiempoPosicionInicial;
  double TiempoTrayectoria;
  double Dif_Tiempo;
  double TiempoMargen;
  Eigen::Vector3d PosicionInicial;
  Eigen::Vector3d DistanciaTrayectoria;
  double Yaw_Offset;
  bool Yaw_Enabled;
  double Exp;
  int ignore_bitmask;
};

struct Ignore {
  bool Px;
  bool Py;
  bool Pz;
  bool Vx;
  bool Vy;
  bool Vz;
  bool Ax;
  bool Ay;
  bool Az;
  bool Yaw;
  bool YawRate;
};

enum Estado {
  PosicionInicio,
  Trayectoria,
  Margen,
};

namespace mav_msgs {
  inline Eigen::Vector4d vector4FromQuaternionMsg(const geometry_msgs::Quaternion& msg) {
    return Eigen::Vector4d(msg.x, msg.y, msg.z, msg.w);
  }
  inline double secsFromHeaderMsg(const std_msgs::Header& msg) {
    return (double) (msg.stamp.sec) + (double) (msg.stamp.nsec)/1.0e9;
  }
}

void clamp(double& valor, double min, double max){
  if (valor < min){
    valor = min;
  }
  else if (valor > max){
    valor = max;
  }
}

namespace ardupilot_mavros {

    class Waypoint{
        public:
            Waypoint(Parametros param);
            ~Waypoint();
            
        private:
            Parametros _param;
            nav_msgs::Odometry setpoint_;
            nav_msgs::Odometry odometry_;
            mavros_msgs::PositionTarget position_target_;
            geometry_msgs::PoseStamped setpoint_position_;
            double _yaw_drone;
            double _yaw_os;
            
            ros::NodeHandle _n1;
            ros::Timer _timer1;
            ros::NodeHandle _n2;
            ros::Timer _timer2;
            ros::NodeHandle _n3;
            ros::Timer _timer3;
            ros::NodeHandle nh;

            ros::Subscriber odometry_sub_;
            ros::Subscriber yaw_sub_;
            ros::Publisher trajectory_ref_pub_;
            ros::Publisher Begin_;
            ros::Publisher End_;
            ros::Publisher odometry_pub_;
            ros::Publisher setpoint_pub_;
            ros::Publisher position_target_pub_;

            ros::Time TiempoInicio;

            void CallbackStart(const ros::TimerEvent& event);
            void CallbackPath(const ros::TimerEvent& event);
            void CallbackStop(const ros::TimerEvent& event);
            void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
            void YawCallback(const std_msgs::Float64& yaw_msg);
            Eigen::Vector3d Quat2RPY(Eigen::Vector4d Quaternion);
            Eigen::Vector4d RPY2Quat(Eigen::Vector3d RPY);

            enum Estado estado;
    };

    Waypoint::Waypoint(Parametros param)
    : _param(param),
      estado(PosicionInicio){
        odometry_sub_ = nh.subscribe("/mavros/local_position/odom", 1, &Waypoint::OdometryCallback, this);
        yaw_sub_ = nh.subscribe("/mavros/global_position/compass_hdg", 1, &Waypoint::YawCallback, this);
        trajectory_ref_pub_ = nh.advertise<nav_msgs::Odometry>("Setpoint", 1);
        odometry_pub_ = nh.advertise<nav_msgs::Odometry>("Odometry", 1);
        setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
        Begin_ = nh.advertise<std_msgs::Empty>("/csv/begin",1);
        End_ = nh.advertise<std_msgs::Empty>("/csv/end",1);
        position_target_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);

        _timer1 = _n1.createTimer(ros::Duration(_param.Dif_Tiempo), &Waypoint::CallbackPath, this, false, true);
        _timer2 = _n2.createTimer(ros::Duration(_param.TiempoPosicionInicial), &Waypoint::CallbackStart, this, true, true);
        double TotalTime = _param.TiempoPosicionInicial + _param.TiempoTrayectoria + _param.TiempoMargen;
        _timer3 = _n3.createTimer(ros::Duration(TotalTime), &Waypoint::CallbackStop, this, false, true);
    }
    
    Waypoint::~Waypoint() {}

    void Waypoint::CallbackStart(const ros::TimerEvent& event) {
      estado = Trayectoria;
      TiempoInicio = ros::Time::now();
      if (_param.Yaw_Enabled){
        _yaw_os = atan2(2*_param.DistanciaTrayectoria.y(),_param.DistanciaTrayectoria.x()) + _param.Yaw_Offset + _yaw_drone;
      }
      else {
        _yaw_os = _param.Yaw_Offset + _yaw_drone;
      }
      double TiempoSec = (double) (TiempoInicio.sec) + (double) (TiempoInicio.nsec)/1.0e9; 
      ros::param::set("/TiempoInicio", TiempoSec);
      std_msgs::Empty empty_;
      Begin_.publish(empty_);
    }

    void Waypoint::CallbackStop(const ros::TimerEvent& event) {
      std_msgs::Empty empty_;
      for (int i = 0; i < 50; i++){
        End_.publish(empty_);
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
      ros::shutdown();
    }

    void Waypoint::CallbackPath(const ros::TimerEvent& event) {
      ros::Time TiempoActual = ros::Time::now();
      double x, y, z, yaw;
      double dot_x, dot_y, dot_z, dot_yaw;
      double ddot_x, ddot_y, ddot_z;
      double t = TiempoActual.toSec() - TiempoInicio.toSec() + _param.Dif_Tiempo;
      double W1, W2, W3;
      W1 = W3 = 2*M_PI/_param.TiempoTrayectoria;
      W2 = 4*M_PI/_param.TiempoTrayectoria;

      double x1, y1, z1, x2, y2, z2;
      double dx1, dy1, dz1, dx2, dy2, dz2;
      double ddx1, ddy1, ddz1, ddx2, ddy2, ddz2;

      x1 = _param.DistanciaTrayectoria.x()*sin(W1*t);
      y1 = _param.DistanciaTrayectoria.y()*sin(W2*t);
      z1 = _param.DistanciaTrayectoria.z()*sin(W3*t);

      double expV = exp(-_param.Exp*(t-_param.TiempoTrayectoria));
      if (!std::isfinite(expV)){
        expV = 10000.0;
      }
      clamp(expV,0.00001,10000.0);

      x2 = y2 = z2 = 1 - 1.0/(1.0 + expV);

      dx1 = _param.DistanciaTrayectoria.x()*W1*cos(W1*t);
      dy1 = _param.DistanciaTrayectoria.y()*W2*cos(W2*t);
      dz1 = _param.DistanciaTrayectoria.z()*W3*cos(W3*t);
      
      ddx1 = -_param.DistanciaTrayectoria.x()*pow(W1,2)*sin(W1*t);
      ddy1 = -_param.DistanciaTrayectoria.y()*pow(W2,2)*sin(W2*t);
      ddz1 = -_param.DistanciaTrayectoria.z()*pow(W3,2)*sin(W3*t);

      dx2 = dy2 = dz2 = -(_param.Exp*expV)/pow(1+expV,2);
      ddx2 = ddy2 = ddz2 = (pow(_param.Exp,2)*expV)/(pow(1 + expV,2)) - (2*_param.Exp*pow(expV,2))/(pow(1 + expV,3));
      
      x = x1*x2 + _param.PosicionInicial.x();
      y = y1*y2 + _param.PosicionInicial.y();
      z = z1*z2 + _param.PosicionInicial.z();
      dot_x = x1*dx2 + x2*dx1;
      dot_y = y1*dy2 + y2*dy1;
      dot_z = z1*dz2 + z2*dz1;
      ddot_x = x1*ddx2 + x2*ddx1 + 2*dx1*dx2;
      ddot_y = y1*ddy2 + y2*ddy1 + 2*dy1*dy2;
      ddot_z = z1*ddz2 + z2*ddz1 + 2*dz1*dz2;
      
      switch (estado)
      {
      case PosicionInicio:
        x = _param.PosicionInicial.x();
        y = _param.PosicionInicial.y();
        z = _param.PosicionInicial.z();
        if (_param.Yaw_Enabled){
          yaw = atan2(2*_param.DistanciaTrayectoria.y(),_param.DistanciaTrayectoria.x()) + _param.Yaw_Offset;
        }
        else {
          yaw = _param.Yaw_Offset;
        }
        dot_x = dot_y = dot_z = ddot_x = ddot_y = ddot_z = dot_yaw = 0;
        break;

      case Margen:
        if (_param.Yaw_Enabled){
          yaw = atan2(2*_param.DistanciaTrayectoria.y(),_param.DistanciaTrayectoria.x()) + _param.Yaw_Offset;
        }
        else {
          yaw = _param.Yaw_Offset;
        }
        dot_yaw = 0;
        break;

      case Trayectoria:
        std_msgs::Empty empty_;
        Begin_.publish(empty_);
        if (t > _param.TiempoTrayectoria) {
          estado = Margen;
          return;
        }
        if (_param.Yaw_Enabled){
          yaw = atan2(dy1,dx1) + _param.Yaw_Offset;
        }
        else {
          yaw = _param.Yaw_Offset;
        }
        dot_yaw = ((dx1*ddy1 - dy1*ddx1)/pow(dx1,2))/(1+pow(dy1/dx1,2));
        break;
      }

      Eigen::Vector4d Quaternion_ref = RPY2Quat(Eigen::Vector3d(0.0,0.0,yaw));
      Eigen::Vector4d Quaternion_odom_ = RPY2Quat(Eigen::Vector3d(0.0,0.0,_yaw_os-_yaw_drone));

      position_target_.header.stamp = TiempoActual;
      position_target_.coordinate_frame = FRAME_LOCAL_NED;
      position_target_.type_mask = _param.ignore_bitmask;
      position_target_.position.x = x;
      position_target_.position.y = y;
      position_target_.position.z = z;
      position_target_.velocity.x = dot_x;
      position_target_.velocity.y = dot_y;
      position_target_.velocity.z = dot_z;
      position_target_.acceleration_or_force.x = ddot_x;
      position_target_.acceleration_or_force.y = ddot_y;
      position_target_.acceleration_or_force.z = ddot_z;
      position_target_.yaw = yaw;
      position_target_.yaw_rate = dot_yaw;
      position_target_pub_.publish(position_target_);

      setpoint_.header.stamp = TiempoActual;
      setpoint_.pose.pose.position.x = x;
      setpoint_.pose.pose.position.y = y;
      setpoint_.pose.pose.position.z = z;
      setpoint_.pose.pose.orientation.x = Quaternion_ref.x();
      setpoint_.pose.pose.orientation.y = Quaternion_ref.y();
      setpoint_.pose.pose.orientation.z = Quaternion_ref.z();
      setpoint_.pose.pose.orientation.w = Quaternion_ref.w();
      trajectory_ref_pub_.publish(setpoint_);

      odometry_.header.stamp = TiempoActual;
      odometry_.pose.pose.orientation.x = Quaternion_odom_.x();
      odometry_.pose.pose.orientation.y = Quaternion_odom_.y();
      odometry_.pose.pose.orientation.z = Quaternion_odom_.z();
      odometry_.pose.pose.orientation.w = Quaternion_odom_.w();
      odometry_pub_.publish(odometry_);    
    }

    void Waypoint::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
      odometry_.header = odometry_msg->header;
      odometry_.child_frame_id = odometry_msg->child_frame_id;
      odometry_.pose = odometry_msg->pose;
      odometry_.twist = odometry_msg->twist;
    }

    void Waypoint::YawCallback(const std_msgs::Float64& yaw_msg) {
      _yaw_drone = static_cast<double>(yaw_msg.data)*DEG_2_RAD;
    }

    Eigen::Vector3d Waypoint::Quat2RPY(Eigen::Vector4d Quaternion) {
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

    Eigen::Vector4d Waypoint::RPY2Quat(Eigen::Vector3d RPY) {
      tfScalar roll, pitch, yaw;
      tf::Quaternion q;
      roll = RPY.x();
      pitch = RPY.y();
      yaw = RPY.z();
      tf::Matrix3x3 m;
      m.setEulerYPR(yaw, pitch, roll);
      m.getRotation(q);
      return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
    }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_gen");
  ros::NodeHandle nh;

  double X_I, Y_I, Z_I, DIST_X, DIST_Y, DIST_Z;
  Parametros param;
  Ignore ignore;

  ros::param::get("~TiempoPosicionInicial", param.TiempoPosicionInicial);
  ros::param::get("~TiempoTrayectoria", param.TiempoTrayectoria);
  ros::param::get("~Dif_Tiempo", param.Dif_Tiempo);
  ros::param::get("~TiempoMargen", param.TiempoMargen);
  ros::param::get("~X_Inicial", X_I);
  ros::param::get("~Y_Inicial", Y_I);
  ros::param::get("~Z_Inicial", Z_I);
  ros::param::get("~X_Distancia", DIST_X);
  ros::param::get("~Y_Distancia", DIST_Y);
  ros::param::get("~Z_Distancia", DIST_Z);
  ros::param::get("~Yaw_Offset", param.Yaw_Offset);
  ros::param::get("~Yaw_Enabled", param.Yaw_Enabled);
  ros::param::get("~Exp", param.Exp);
  ros::param::get("~Ignore_Px", ignore.Px);
  ros::param::get("~Ignore_Py", ignore.Py);
  ros::param::get("~Ignore_Pz", ignore.Pz);
  ros::param::get("~Ignore_Vx", ignore.Vx);
  ros::param::get("~Ignore_Vy", ignore.Vy);
  ros::param::get("~Ignore_Vz", ignore.Vz);
  ros::param::get("~Ignore_Ax", ignore.Ax);
  ros::param::get("~Ignore_Ay", ignore.Ay);
  ros::param::get("~Ignore_Az", ignore.Az);
  ros::param::get("~Ignore_Yaw", ignore.Yaw);
  ros::param::get("~Ignore_Yaw_Rate", ignore.YawRate);

  param.PosicionInicial = Eigen::Vector3d(X_I,Y_I,Z_I);
  param.DistanciaTrayectoria = Eigen::Vector3d(DIST_X,DIST_Y,DIST_Z);
  param.ignore_bitmask = 0;

  if (ignore.Px){
    param.ignore_bitmask += IGNORE_PX;
  }
  if (ignore.Py){
    param.ignore_bitmask += IGNORE_PY;
  }
  if (ignore.Pz){
    param.ignore_bitmask += IGNORE_PZ;
  }
  if (ignore.Vx){
    param.ignore_bitmask += IGNORE_VX;
  }
  if (ignore.Vy){
    param.ignore_bitmask += IGNORE_VY;
  }
  if (ignore.Vz){
    param.ignore_bitmask += IGNORE_VZ;
  }
  if (ignore.Ax){
    param.ignore_bitmask += IGNORE_AX;
  }
  if (ignore.Ay){
    param.ignore_bitmask += IGNORE_AY;
  }
  if (ignore.Az){
    param.ignore_bitmask += IGNORE_AZ;
  }
  if (ignore.Yaw){
    param.ignore_bitmask += IGNORE_YAW;
  }
  if (ignore.YawRate){
    param.ignore_bitmask += IGNORE_YAW_RATE;
  }

  ardupilot_mavros::Waypoint waypoint(param);

  ros::spin();
  
  return 0;

}
