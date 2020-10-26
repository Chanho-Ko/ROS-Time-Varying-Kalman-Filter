#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <Eigen/Dense>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>

using namespace Eigen;

typedef Matrix<double, 6, 6> squareMat;
typedef Matrix<double, 6, 1> colVector;

class TimeVaryingKF
{
private:
    ros::NodeHandle _nh;
    ros::Subscriber sub_acc;
    ros::Subscriber sub_fix;
    double t_cur, t_past, T;
    double t_cur2, t_past2, T2;
    double acc_x, acc_y, imu_cov;
    double x_init, y_init, x, y, z_x, z_y, gps_cov;
    double roll, pitch, yaw;
    int count, num_init;
    bool set_init;
    double p_;
    squareMat F, Q, P, P_inv, K, H, H_imu, H_gps;
    squareMat R, R_inv, R_imu, R_imu_inv, R_gps, R_gps_inv;
    colVector X, z;
    
public:
    TimeVaryingKF(ros::NodeHandle* nodehandle);
    void imuKF(const sensor_msgs::Imu::ConstPtr &msg_imu);
    void gpsKF(const sensor_msgs::NavSatFix::ConstPtr &msg_gps);
    void prediction();
    void correction(bool is_imu);
    void QuatToRPY(double q_x, double q_y, double q_z, double q_w);
};

TimeVaryingKF::TimeVaryingKF(ros::NodeHandle* nodehandle) : _nh(*nodehandle)
{
    sub_acc = _nh.subscribe("/imu/data", 1, &TimeVaryingKF::imuKF, this);
    sub_fix = _nh.subscribe("/fix", 1, &TimeVaryingKF::gpsKF, this);

    t_past = 0;
    count = 0, set_init = false;
    x_init = 0; y_init = 0;
    p_ = 0.125;
    imu_cov = 0.7;

    X << 0, 0, 0, 0, 0, 0;
    P << p_*p_*p_/6, 0, 0, 0, 0, 0,
        0, p_*p_*p_/6, 0, 0, 0, 0,
        0, 0, p_*p_/2, 0, 0, 0,
        0, 0, 0, p_*p_/2, 0, 0,
        0, 0, 0, 0, p_, 0,
        0, 0, 0, 0, 0, p_;
    R_imu << 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, imu_cov, 0,
        0, 0, 0, 0, 0, imu_cov;
    R_imu_inv << 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1/imu_cov, 0,
            0, 0, 0, 0, 0, 1/imu_cov;
    H_imu << 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
    H_gps << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0;
    //P.diagonal() << p_*p_*p_/6, p_*p_*p_/6, p_*p_/2, p_*p_/2, p_, p_;
    //P_inv.diagonal() << 1/(p_*p_*p_/6), 1/(p_*p_*p_/6), 1/(p_*p_/2), 1/(p_*p_/2), 1/p_, 1/p_;
}

void TimeVaryingKF::imuKF(const sensor_msgs::Imu::ConstPtr &msg_imu)
{   
    ros::Time now_ = ros::Time::now();
    acc_x = msg_imu->linear_acceleration.x;
    acc_y = msg_imu->linear_acceleration.y;
    double q_x = msg_imu->orientation.x, q_y = msg_imu->orientation.y, 
            q_z = msg_imu->orientation.z, q_w = msg_imu->orientation.w;
    QuatToRPY(q_x, q_y, q_z, q_w);

    std::cout << "roll: " << roll << std::endl;
    std::cout << "pitch: " << pitch << std::endl;
    std::cout << "yaw: " << yaw << std::endl;
    
    
    t_cur = msg_imu->header.stamp.sec + 1e-9*(msg_imu->header.stamp.nsec);
    t_cur2 = now_.sec + 1e-9*(now_.nsec);

    // if ( t_past == 0.0 ) { T = 0.01; }
    // else { T = t_cur - t_past; }
    // if ( t_past2 == 0.0 ) { T2 = 0.01; }
    // else { T2 = t_cur2 - t_past2; }
    T = t_cur2 - t_past2;
    //std::cout << "Time Step:" << T << std::endl;   
    // std::cout << "###############################" << std::endl;
    // std::cout << "Current Time: \n" << now_ << std::endl;
    // std::cout << "Stamp Time: \n" << msg_imu->header.stamp << std::endl;
    // std::cout << "Value: \n" << msg_imu->vector << std::endl;
    
    // if (set_init==true){
    //     prediction();
    //     correction(true);
    // }
    t_past = t_cur;
    t_past2 = t_cur2;
}

void TimeVaryingKF::gpsKF(const sensor_msgs::NavSatFix::ConstPtr &msg_gps)
{
    x = msg_gps->longitude;
    y = msg_gps->latitude;
    gps_cov = msg_gps->position_covariance[0];

    num_init = 300;
    if (count<num_init)
    {
        x_init += x;
        y_init += y;
        count ++;
        if (count==num_init)
        {
            x_init = x_init/num_init; y_init = y_init/num_init; 
            set_init = true;
        }
    }

    ros::Time now_ = ros::Time::now();

    t_cur = msg_gps->header.stamp.sec + 1e-9*(msg_gps->header.stamp.nsec);
    t_cur2 = now_.sec + 1e-9*(now_.nsec);

    // if ( t_past == 0.0 ) { T = 0.01; }
    // else { T = t_cur - t_past; }
    // if ( t_past2 == 0.0 ) { T2 = 0.01; }
    // else { T2 = t_cur2 - t_past2; }
    T = t_cur2 - t_past2;

    //std::cout << "###############################" << std::endl;
    //std::cout << "Current Time: \n" << now_ << std::endl;
    //std::cout << "Stamp Time: \n" << msg_gps->header.stamp << std::endl;
    //std::cout << "Value: \n" << msg_gps->vector << std::endl;

    // Do estimation if initial position is set.

    if (set_init == true)
    {
        prediction();
        correction(false);
    }

    t_past = t_cur;
    t_past2 = t_cur2;
}

void TimeVaryingKF::correction(bool is_imu)
{
    if (is_imu==true){
        std::cout << "###############################" << std::endl;
        std::cout << "############ IMU KF ###########" << std::endl;
        std::cout << "###############################" << std::endl;

        z << 0, 0, 0, 0, acc_x, acc_y; // z(k), Measurement of IMU
        H = H_imu;
        R = R_imu;
        R_inv = R_imu_inv;
    }
    else {
        std::cout << "###############################" << std::endl;
        std::cout << "############ GPS KF ###########" << std::endl;
        std::cout << "###############################" << std::endl;

        // latitude & longitude to meter
        z_x = (x - x_init)*91.290*1000;
        z_y = (y - y_init)*110.941*1000;
        printf("x coord: %3.10lf \n", z_x);
        printf("y coord: %3.10lf \n", z_y);

        z << z_x, z_y, 0, 0, 0, 0; // z(k), Measurement of GPS
        H = H_gps;
        R_gps << gps_cov, 0, 0, 0, 0, 0,
            0, gps_cov, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0;
        R = R_gps;
        R_gps_inv << 1/gps_cov, 0, 0, 0, 0, 0,
                0, 1/gps_cov, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0;
        R_inv = R_gps_inv;
    }
    
    P_inv = P.inverse();
    P = (P_inv + H*R_inv*H).inverse(); // P(k|k)=P(k|k-1)+(H^T*R(k)^-1*H)^-1
    K = P*H*R_inv; // K(k)=P(k|k)H^T*R^-1
    X = X + K*(z-H*X); // x(k|k)=x(k|k-1)+K(k)(z(k)-Hx(k|k-1))

    printf("est x coord: %3.10lf \n", X(0));
    printf("est y coord: %3.10lf \n", X(1));
    std::cout << "State vector: \n" << X << std::endl;

    // Write data in txt
    // std::ofstream writeFile1("campus1015_raw_position(rostime).txt", std::ios::app);
    // if (writeFile1.is_open()){
    //     writeFile1 << z_x << " " << z_y << std::endl;
    // }
    std::ofstream writeFile2("campus1015_estimated_states(only_gps).txt", std::ios::app);
    if (writeFile2.is_open()){
        writeFile2 << X(0) << " " << X(1) << " " << X(2) << " "
                    << X(3) << " " << X(4) << " " << X(5) << std::endl;
    }
}

void TimeVaryingKF::prediction()
{
    //std::cout << "Time step by stamp: \n" << T << std::endl;
    //std::cout << "Time step by now: \n" << T2 << std::endl;
    //std::cout << "Current Time: \n" << t_cur_ns << std::endl;
    F << 1, 0, T, 0, T*T/2, 0,
        0, 1, 0, T, 0, T*T/2,
        0, 0, 1, 0, T, 0,
        0, 0, 0, 1, 0, T,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
    Q << T*T*T/6, 0, 0, 0, 0, 0,
        0, T*T*T/6, 0, 0, 0, 0,
        0, 0, T*T/2, 0, 0, 0,
        0, 0, 0, T*T/2, 0, 0,
        0, 0, 0, 0, T, 0,
        0, 0, 0, 0, 0, T;
    // Q.diagonal() << T*T*T/6, T*T*T/6, T*T/2, T*T/2, T, T;

    X = F*X; // X(k+1|k)
    P = F*P*F.transpose() + Q; // P(k|k+1)=FP(k|k)F^T + Q
}

void TimeVaryingKF::QuatToRPY(double _q_x, double _q_y, double _q_z, double _q_w)
{
    double sqx = _q_x*_q_x;
    double sqy = _q_y*_q_y;
    double sqz = _q_z*_q_z;
    double sqw = _q_w*_q_w;

    double pi = M_PI;
    roll = asin((_q_w*_q_x-_q_y*_q_z))*180/pi;
    pitch = atan2((_q_x*_q_z+_q_w*_q_y),(-sqx-sqy-sqz+sqw))*180/pi;
    yaw = atan2((_q_x*_q_y+_q_w*_q_z),(-sqx+sqy-sqz+sqw))*180/pi;
}