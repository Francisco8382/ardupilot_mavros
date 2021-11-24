/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Pasquale Oppido, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
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

#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <string>
#include <ros/time.h>
#include <tf/tf.h>

#include <gazebo_msgs/GetWorldProperties.h>

#include "bebop_simulator/ReferenceAngles.h"
#include "bebop_simulator/PosController.h"
#include "bebop_simulator/AttitudeController.h"

using namespace std;

namespace ardupilot_mavros {

struct EigenOdometry {
  EigenOdometry()
      : timeStampSec(-1),
        timeStampNsec(-1),
        position(0.0, 0.0, 0.0),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(0.0, 0.0, 0.0),
        angular_velocity(0.0, 0.0, 0.0) {};

  EigenOdometry(const double _timeStampSec,
                const double _timeStampNsec,
                const Eigen::Vector3d& _position,
                const Eigen::Quaterniond& _orientation,
                const Eigen::Vector3d& _velocity,
                const Eigen::Vector3d& _angular_velocity) {
    
    timeStampSec = _timeStampSec;
    timeStampNsec = _timeStampNsec;
    position = _position;
    orientation = _orientation;
    velocity = _velocity;
    angular_velocity = _angular_velocity;
  };

  double timeStampSec; //nano seconds timeStamp Odometry message
  double timeStampNsec; //nano seconds timeStamp Odometry message
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity; // Velocity is expressed in the Body frame!
  Eigen::Vector3d angular_velocity;
};

inline void eigenOdometryFromMsg(const nav_msgs::OdometryConstPtr& msg,
                                 EigenOdometry* odometry) {
  odometry->timeStampSec = msg->header.stamp.sec;
  odometry->timeStampNsec = msg->header.stamp.nsec;
  odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);
  odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
  odometry->velocity = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
  odometry->angular_velocity = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
}

// Default values for the Parrot Bebop controller. For more information about the control architecture, please take a look
// at the publications page into the Wiki section.
static const double KpDefaultXYController = 17.8;
static const double KpDefaultAltitudeController = 25.5;
static const double KpDefaultRollPitchController = 578.85;
static const double KpDefaultYawRateController = 23.15;
static const double TdDefaultXYController = 0.53;
static const double TdDefaultAltitudeController = 0.222;
static const double TdDefaultRollPitchController = 0.109;
static const double TdDefaultYawRateController = 0.254;
static const double TiDefaultXYController = 1.644;
static const double TiDefaultAltitudeController = 1.98;
static const double TiDefaultRollPitchController = 620.0;
static const double TiDefaultYawRateController = 1.44;
static const Eigen::Vector3d UqDefaultXYZ = Eigen::Vector3d(1.1810, 1.1810, 4.6697);

class PositionControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PositionControllerParameters()
      : Kp_xy_(KpDefaultXYController), 
        Kp_z_(KpDefaultAltitudeController), 
        Kp_phi_theta_(KpDefaultRollPitchController),  
        Kp_psi_(KpDefaultYawRateController),
        Td_xy_(TdDefaultXYController),
        Td_z_(TdDefaultAltitudeController),
        Td_phi_theta_(TdDefaultRollPitchController),
        Td_psi_(TdDefaultYawRateController),
        Ti_xy_(TiDefaultXYController),
        Ti_z_(TiDefaultAltitudeController),
        Ti_phi_theta_(TiDefaultRollPitchController),
        Ti_psi_(TiDefaultYawRateController),
	    U_q_(UqDefaultXYZ){
  }
  double Kp_xy_;
  double Kp_z_;
  double Kp_phi_theta_;
  double Kp_psi_;
  double Td_xy_;
  double Td_z_;
  double Td_phi_theta_;
  double Td_psi_;
  double Ti_xy_;
  double Ti_z_;
  double Ti_phi_theta_;
  double Ti_psi_;
  Eigen::Vector3d U_q_;
};
    
    class PositionController{
        public:
            PositionController();
            ~PositionController();
            void CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities);

            void SetOdometry(const EigenOdometry& odometry);
            void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory_positionControllerNode);
            void SetControllerGains();
            void SetVehicleParameters();
            void SetWaypointFilterParameters();
            void SetFilterParameters();
            void GetOdometry(nav_msgs::Odometry* odometry_filtered);
            void GetReferenceAngles(nav_msgs::Odometry* reference_angles);
            void GetTrajectory(nav_msgs::Odometry* smoothed_trajectory);
            void GetUTerrComponents(nav_msgs::Odometry* uTerrComponents);
            void GetVelocityAlongZComponents(nav_msgs::Odometry* zVelocity_components);
            void GetPositionAndVelocityErrors(nav_msgs::Odometry* positionAndVelocityErrors);
            void GetAngularAndAngularVelocityErrors(nav_msgs::Odometry* angularAndAngularVelocityErrors);
            
            PositionControllerParameters controller_parameters_;
            ExtendedKalmanFilter extended_kalman_filter_bebop_;
            VehicleParameters vehicle_parameters_;
            FilterParameters filter_parameters_;
            WaypointFilterParameters waypoint_filter_parameters_;
            WaypointFilter waypoint_filter_;

            //Launch file parameters
            std::string user_;
            double dataStoringTime_;
            bool dataStoring_active_;
            bool waypointFilter_active_;
            bool EKF_active_;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:
            //Boolean variables to active/unactive the controller and the data storage
            bool controller_active_;

            //Wall clock time offset variable
            double wallSecsOffset_;

            //Gazebo Message for attitude and position
            gazebo_msgs::GetWorldProperties my_messagePosition_;
            ros::NodeHandle clientHandlePosition_;
            ros::ServiceClient clientPosition_;

            ros::NodeHandle clientHandleAttitude_;
            ros::ServiceClient clientAttitude_;
            gazebo_msgs::GetWorldProperties my_messageAttitude_;
          
            //Controller gains
            double Kp_xy_, Kp_z_;
            double Kp_phi_theta_, Kp_psi_;
            double Td_xy_, Td_z_;
            double Td_phi_theta_, Td_psi_;
            double Ti_xy_, Ti_z_;
            double Ti_phi_theta_, Ti_psi_;
	        double lambda_x_, lambda_y_, lambda_z_;

            //Position and linear velocity errors
            double e_x_;
            double e_y_;
            double e_z_;
            double dot_e_x_;
            double dot_e_y_; 
            double dot_e_z_;
            double sum_e_x_;
            double sum_e_y_; 
            double sum_e_z_;
 
            //Attitude and angular velocity errors
            double e_phi_;
            double e_theta_;
            double e_psi_;
            double dot_e_phi_;
            double dot_e_theta_; 
            double dot_e_psi_;
            double sum_e_phi_;
            double sum_e_theta_; 
            double sum_e_psi_;

            //Vehicle parameters
            double bf_, m_, g_;
            double l_, bm_;
            double Ix_, Iy_, Iz_;
            
            ros::NodeHandle n1_;
            ros::NodeHandle n2_;
            ros::NodeHandle n3_;
            ros::Timer timer1_;
            ros::Timer timer2_;
            ros::Timer timer3_;

            //Callback functions to compute the errors among axis and angles
            void CallbackAttitude(const ros::TimerEvent& event);
            void CallbackPosition(const ros::TimerEvent& event);

            nav_msgs::Odometry odometry_filtered_private_;

	        state_t state_;
            control_t control_;
            mav_msgs::EigenTrajectoryPoint command_trajectory_;
            EigenOdometry odometry_;

            ros::NodeHandle nh;
            ros::Publisher ref_angles_pub;
            ros::Publisher position_controller_pub;
            ros::Publisher attitude_controller_pub;
            bebop_simulator::ReferenceAngles ref_angles;
            bebop_simulator::PosController position_controller;
            bebop_simulator::AttitudeController attitude_controller;

            void SetOdometryEstimated();
            void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;
            void AttitudeController(double* u_phi, double* u_theta, double* u_psi);
            void AngularVelocityErrors(double* dot_e_phi_, double* dot_e_theta_, double* dot_e_psi_);
            void AttitudeErrors(double* e_phi_, double* e_theta_, double* e_psi_);
            void PosController(double* u_T, double* phi_r, double* theta_r, double* u_x, double* u_y, double* u_z, double* u_Terr);
            void PositionErrors(double* e_x, double* e_y, double* e_z);
            void VelocityErrors(double* dot_e_x, double* dot_e_y, double* dot_e_z);

    };

}
#endif // POSITION_CONTROLLER_H
