#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <Eigen/Dense>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>

using namespace Eigen;

typedef Matrix<double, 8, 8> squareMat;
typedef Matrix<double, 8, 1> colVector;

class TimeVaryingKF
{
private:
    ros::NodeHandle _nh;
    ros::Subscriber sub_acc;
    ros::Subscriber sub_fix;
    ros::Publisher pub_state;
    double t_cur, t_past, T;
    double acc_x, acc_y, acc_z, imu_cov;
    double x_init, y_init, x, y, z_x, z_x_past, z_y, z_y_past, gps_cov;
    double z_yaw, z_yaw_past, th_yaw;
    double roll, pitch, yaw, yaw_rate;
    int count, num_init;
    bool set_init;
    double p_;
    squareMat F, Q, P, P_inv, K, H, H_imu, H_gps;
    squareMat R, R_inv, R_imu, R_imu_inv, R_gps, R_gps_inv;
    colVector X, z, z_hat;
    
public:
    TimeVaryingKF(ros::NodeHandle* nodehandle);
    void imuKF(const sensor_msgs::Imu::ConstPtr &msg_imu);
    void gpsKF(const sensor_msgs::NavSatFix::ConstPtr &msg_gps);
    void prediction();
    void correction(bool is_imu);
    void QuatToRPY(double q_x, double q_y, double q_z, double q_w);
    void FrameTransform(const double roll, const double pitch, const double yaw);
};

TimeVaryingKF::TimeVaryingKF(ros::NodeHandle* nodehandle) : _nh(*nodehandle)
{
    sub_acc = _nh.subscribe("/imu/data", 1, &TimeVaryingKF::imuKF, this);
    sub_fix = _nh.subscribe("/fix", 1, &TimeVaryingKF::gpsKF, this);
    pub_state = _nh.advertise<std_msgs::Float32MultiArray>("states",100);
    t_past = 0;
    count = 0, set_init = false;
    x_init = 0; y_init = 0;
    p_ = 0.125;
    imu_cov = 0.7;
    z_x_past = 0; z_y_past = 0; z_yaw_past = 0;
    th_yaw = 0.2; // 20cm

    X << 0, 0, 0, 0, 0, 0, M_PI, 0;

    P << p_*p_*p_/6, 0, 0, 0, 0, 0, 0, 0,
        0, p_*p_*p_/6, 0, 0, 0, 0, 0, 0,
        0, 0, p_*p_/2, 0, 0, 0, 0, 0,
        0, 0, 0, p_*p_/2, 0, 0, 0, 0,
        0, 0, 0, 0, p_, 0, 0, 0,
        0, 0, 0, 0, 0, p_, 0, 0,
        0, 0, 0, 0, 0, 0, p_*p_/2, 0,
        0, 0, 0, 0, 0, 0, 0, p_;

    R_imu << 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, imu_cov, 0, 0, 0,
             0, 0, 0, 0, 0, imu_cov, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0.02;

    R_imu_inv << 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 1/imu_cov, 0, 0, 0,
                0, 0, 0, 0, 0, 1/imu_cov, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 1/0.02;

    H_imu << 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1;

    H_gps << 1, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0;
}

void TimeVaryingKF::imuKF(const sensor_msgs::Imu::ConstPtr &msg_imu)
{   
    /* Roll-Pitch-Yaw conversion from quaternion */
    double q_x = msg_imu->orientation.x, q_y = msg_imu->orientation.y, 
            q_z = msg_imu->orientation.z, q_w = msg_imu->orientation.w;
    QuatToRPY(q_x, q_y, q_z, q_w);

    /* Sensor Frame to Global Frame */
    yaw = X(6);
    acc_x = msg_imu->linear_acceleration.x;
    acc_y = msg_imu->linear_acceleration.y;
    acc_z = msg_imu->linear_acceleration.z;
    yaw_rate = msg_imu->angular_velocity.z;
    FrameTransform(roll, pitch, yaw);

    /* Time step update */
    ros::Time now_ = ros::Time::now();
    t_cur = now_.sec + 1e-9*(now_.nsec);
    T = t_cur - t_past;

    /* Prediction & Correction */
    if (set_init==true){
        prediction();
        correction(true); // true indicates IMU
    }

    t_past = t_cur;
}

void TimeVaryingKF::gpsKF(const sensor_msgs::NavSatFix::ConstPtr &msg_gps)
{
    x = msg_gps->longitude;
    y = msg_gps->latitude;
    gps_cov = msg_gps->position_covariance[0];

    /* Set the initial point.
       Initial point is going to be set as (0,0).
       Set of points for initialization should be steady */
    num_init = 100;
    if (count < num_init)
    {
        x_init += x;
        y_init += y;
        count++;
        std::cout << "Count for setting init point: " << count << std::endl;
        if (count==num_init)
        {
            x_init = x_init/num_init;
            y_init = y_init/num_init; 
            set_init = true;
        }
    }

    /* Time step update */
    ros::Time now_ = ros::Time::now();
    t_cur = now_.sec + 1e-9*(now_.nsec);
    T = t_cur - t_past;

    /* Prediction & Correction */
    if (set_init == true)
    {
        prediction();
        correction(false); // false indicates GPS
    }

    t_past = t_cur;
}

void TimeVaryingKF::correction(bool is_imu)
{
    if (is_imu==true){
        std::cout << "###############################" << std::endl;
        std::cout << "############ IMU KF ###########" << std::endl;
        std::cout << "###############################" << std::endl;

        z << 0, 0, 0, 0, acc_x, acc_y, 0, yaw_rate; // z(k), Measurement of IMU

        P_inv = P.inverse();
        P = (P_inv + H_imu*R_imu_inv*H_imu).inverse(); // P(k|k)=P(k|k-1)+(H^T*R(k)^-1*H)^-1
        K = P*H_imu*R_imu_inv; // K(k)=P(k|k)H^T*R^-1
        X = X + K*(z-H_imu*X); // x(k|k)=x(k|k-1)+K(k)(z(k)-Hx(k|k-1))
    }
    else {
        std::cout << "###############################" << std::endl;
        std::cout << "############ GPS KF ###########" << std::endl;
        std::cout << "###############################" << std::endl;

        /* latitude & longitude to meter */
        z_x = (x - x_init)*91.290*1000;
        z_y = (y - y_init)*110.941*1000;

        /* IF current displacement is quite small,
           use previous value of yaw measurement */
        double dis = sqrt(pow(z_x - z_x_past,2)+pow(z_y - z_y_past,2));
        if (dis < th_yaw){
            z_yaw = z_yaw_past;
        }
        else{
            z_yaw = atan2(z_y-z_y_past, z_x - z_x_past);
            /* Range of atan2 is -pi to +pi
               tricky thing near +-pi !! */    
            if (X(6)*z_yaw < 0){
                if (z_yaw < 0){z_yaw += 2*M_PI;}
                else{z_yaw -= 2*M_PI;}
            }
        }

        z << z_x, z_y, 0, 0, 0, 0, z_yaw, 0; // z(k), Measurement of GPS
        z_x_past = z_x;
        z_y_past = z_y;
        z_yaw_past = z_yaw;
        
        R_gps << gps_cov, 0, 0, 0, 0, 0, 0, 0,
                0, gps_cov, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0.05, 0,
                0, 0, 0, 0, 0, 0, 0, 0;
            
        R_gps_inv << 1/gps_cov, 0, 0, 0, 0, 0, 0, 0,
                    0, 1/gps_cov, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 1/0.05, 0,
                    0, 0, 0, 0, 0, 0, 0, 0;

        P_inv = P.inverse();
        P = (P_inv + H_gps*R_gps_inv*H_gps).inverse(); // P(k|k)=P(k|k-1)+(H^T*R(k)^-1*H)^-1
        K = P*H_gps*R_gps_inv; // K(k)=P(k|k)H^T*R^-1
        X = X + K*(z-H_gps*X); // x(k|k)=x(k|k-1)+K(k)(z(k)-Hx(k|k-1))
    }
    // while (X(6)>M_PI){X(6)-=2*M_PI;}
    // while (X(6)<-M_PI){X(6)+=2*M_PI;}

    printf("est Heading: %3.10lf \n", X(6)*180/M_PI);

    /*######################### Write data in csv ###########################*/
    // std::ofstream writeFile1("campus1015_raw_position(rostime).txt", std::ios::app);
    // if (writeFile1.is_open()){
    //     writeFile1 << z_x << " " << z_y << std::endl;
    // }
    std::ofstream writeFile2("campus1015.txt", std::ios::app);
    if (writeFile2.is_open()){
        writeFile2 << X(0) << " " << X(1) << " " << X(2) << " "
                    << X(3) << " " << X(4) << " " << X(5) << " " << X(6) << " " << X(7) << std::endl;
    }
    /*#######################################################################*/

    /* Publish message for vehicle states*/
    std_msgs::Float32MultiArray array;
    array.data.clear();
    //for loop, pushing data in the size of the array
    for (int i = 0; i < 8; i++)
    {
        array.data.push_back(X(i));
    }
    pub_state.publish(array);
}

void TimeVaryingKF::prediction()
{
    F << 1, 0, T, 0, T*T/2, 0, 0, 0,
        0, 1, 0, T, 0, T*T/2, 0, 0,
        0, 0, 1, 0, T, 0, 0, 0,
        0, 0, 0, 1, 0, T, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 1, T,
        0, 0, 0, 0, 0, 0, 0, 1;

    Q << T*T*T/6, 0, 0, 0, 0, 0, 0, 0,
        0, T*T*T/6, 0, 0, 0, 0, 0, 0,
        0, 0, T*T/2, 0, 0, 0, 0, 0,
        0, 0, 0, T*T/2, 0, 0, 0, 0,
        0, 0, 0, 0, T, 0, 0, 0,
        0, 0, 0, 0, 0, T, 0, 0,
        0, 0, 0, 0, 0, 0, T*T/2, 0,
        0, 0, 0, 0, 0, 0, 0, T;

    X = F*X; // X(k+1|k)
    P = F*P*F.transpose() + Q; // P(k|k+1)=FP(k|k)F^T + Q
    while (X(6)>M_PI){X(6)-=2*M_PI;}
    while (X(6)<-M_PI){X(6)+=2*M_PI;}
}

void TimeVaryingKF::QuatToRPY(double _q_x, double _q_y, double _q_z, double _q_w)
{
    /* Reference : https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles */

    /* roll (x-axis rotation) */
    double sinr_cosp = 2 * (_q_w * _q_x + _q_y * _q_z);
    double cosr_cosp = 1 - 2 * (_q_x * _q_x + _q_y * _q_y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    /* pitch (y-axis rotation) */
    double sinp = 2 * (_q_w * _q_y - _q_z * _q_x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);
}

void TimeVaryingKF::FrameTransform(const double roll, const double pitch, const double yaw)
{
    double ca = cos(yaw), sa = sin(yaw);
    double cb = cos(pitch), sb = sin(pitch);
    double cr = cos(roll), sr = sin(roll);
    
    MatrixXd R =  MatrixXd(3,3);
    R << ca*cb, ca*sb*sr-sa*cr, ca*sb*cr+sa*sr,
        sa*cb, sa*sb*sr+ca*cr, sa*sb*cr-ca*sr,
        -sb, cb*sr, cb*cr;
    VectorXd acc_sen = VectorXd(3);
    acc_sen << acc_x, acc_y, acc_z;

    VectorXd acc_glo = VectorXd(3);
    acc_glo = R*acc_sen;
    acc_x = acc_glo(0);
    acc_y = acc_glo(1);

    std::cout << "Linear Acc. on Global frame: \n" << acc_glo << std::endl;
}