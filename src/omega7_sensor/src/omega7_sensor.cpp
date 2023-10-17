#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "dhdc.h"
#include "drdc.h"
#include <Eigen/Eigen>  // 确保这是Eigen库安装路径中的正确相对路径

using namespace Eigen;


#define KP    100.0
#define KVP    10.0
#define MAXF    4.0
#define KR      0.3
#define KWR     0.02
#define MAXT    0.1
#define KG    100.0
#define KVG     5.0
#define MAXG    1.0

class Omega7Sensor : public rclcpp::Node
{
public:
    double nullPose[DHD_MAX_DOF] = { 0.0, 0.0, 0.0,  // translation
                                        0.0, 0.0, 0.0,  // rotation (joint angles)
                                        0.0 };          // gripper
    bool   base  = false;
    //bool   wrist = false;
    bool   grip  = false;

    // 构造函数,有一个参数为节点名称
    Omega7Sensor(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());

          // open the first available device
        if (drdOpen () < 0) {
            printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr ());
            dhdSleep (2.0);
            dhdClose();
        }

        // print out device identifier
        if (!drdIsSupported ()) {
            printf ("unsupported device\n");
            printf ("exiting...\n");
            dhdSleep (2.0);
            drdClose ();
        }
        printf ("%s haptic device detected\n\n", dhdGetSystemName ());

        // perform auto-initialization
        if (!drdIsInitialized () && drdAutoInit () < 0) {
            printf ("error: auto-initialization failed (%s)\n", dhdErrorGetLastStr ());
            dhdSleep (2.0);
            drdClose ();
        }
        else if (drdStart () < 0) {
            printf ("error: regulation thread failed to start (%s)\n", dhdErrorGetLastStr ());
            dhdSleep (2.0);
            drdClose ();
        }

        // move to center
        drdMoveTo (nullPose);
        // request a null force (only gravity compensation will be applied)
        // this will only apply to unregulated axis
        drdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0,  // force
                                       0.0, 0.0, 0.0,  // torque
                                       0.0);           // gripper force

        // disable all axis regulation (but leave regulation thread running)
        drdRegulatePos  (base);
        //drdRegulateRot  (wrist);
        drdRegulateGrip (grip);

        // display instructions
        printf ("press 'q' to quit\n");
        printf ("      'b' to free/hold base (translations)\n");
        //printf ("      'w' to free/hold wrist (rotations)\n");
        printf ("      'g' to free/hold gripper\n\n");

        // display output header
        printf ("BASE  |  ");
        //if (dhdHasActiveWrist ()) printf ("WRIST |  ");
        if (dhdHasGripper ())     printf ("GRIP  |  ");
        printf ("---------");
        //if (dhdHasActiveWrist ()) printf ("---------");
        if (dhdHasGripper ())     printf ("---------");
        printf ("-----------------------\n");

        // 创建发布者
        command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("omega7_sensor_RH", 10);
        // 创建定时器，500ms为周期，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&Omega7Sensor::timecaller, this));
    }

private:

    void timecaller()
    {
        double p[DHD_MAX_DOF];
        double r[3][3];
        double v[DHD_MAX_DOF];
        double f[DHD_MAX_DOF];
        double normf, normt;
        int    res;

        // Eigen objects (mapped to the arrays above)
        Map<Vector3d> position(&p[0], 3);
        Map<Vector3d> force   (&f[0], 3);
        Map<Vector3d> torque  (&f[3], 3);
        Map<Vector3d> velpos  (&v[0], 3);
        Map<Vector3d> velrot  (&v[3], 3);
        Matrix3d center;
        Matrix3d rotation;

        double px, py, pz;  //末端执行器位置
        double angle_x,angle_y,angle_z;  //末端执行器姿态
        double fx, fy, fz, tx, ty, tz;  //force and torque

        // 创建消息
        auto message = std_msgs::msg::Float64MultiArray();
        message.data = {0,0,0,0,0,0,0,0,0,0,0,0};

        // center of workspace
        center.setIdentity ();                           // rotation (matrix)

        // synchronize with regulation thread
        drdWaitForTick ();

        // get position/orientation/gripper and update Eigen rotation matrix
        drdGetPositionAndOrientation (&(p[0]), &(p[1]), &(p[2]),
                                  &(p[3]), &(p[4]), &(p[5]),
                                  &(p[6]), r);
        for (int i=0; i<3; i++) rotation.row(i) = Vector3d::Map(&r[i][0], 3);

        // get position/orientation/gripper velocity
        drdGetVelocity (&(v[0]), &(v[1]), &(v[2]),
                    &(v[3]), &(v[4]), &(v[5]),
                    &(v[6]));

        // compute base centering force
        force = - KP * position;

        // compute wrist centering torque
        AngleAxisd deltaRotation (rotation.transpose() * center);
        torque = rotation * (KR * deltaRotation.angle() * deltaRotation.axis());

        // compute gripper centering force
        f[6]  = - KG * (p[6] - 0.015);

        // scale force to a pre-defined ceiling
        if ((normf = force.norm()) > MAXF) force *= MAXF/normf;

        // scale torque to a pre-defined ceiling
        if ((normt = torque.norm()) > MAXT) torque *= MAXT/normt;

        // scale gripper force to a pre-defined ceiling
        if (f[6] >  MAXG) f[6] =  MAXG;
        if (f[6] < -MAXG) f[6] = -MAXG;

        // add damping
        force  -= KVP * velpos;
        torque -= KWR * velrot;
        f[6]   -= KVG * v[6];

        // apply centering force with damping
        res = drdSetForceAndTorqueAndGripperForce (f[0], f[1], f[2],  // force
                                               f[3], f[4], f[5],  // torque
                                               f[6]);             // gripper force
        if (res < DHD_NO_ERROR) {
            printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr ());
            dhdClose ();
        }
        // print status
        if (base)    printf ("HOLD  |  ");
        else         printf ("free  |  ");
        
        if (dhdHasGripper()) {
            if( grip)  printf ("HOLD  |  ");
            else       printf ("free  |  ");
        }

        printf ("\r");

        // user input
        if (dhdKbHit ()) {
            switch (dhdKbGet ()) {
            case 'q': dhdClose (); break;
            case 'b':
                base = !base;
                drdRegulatePos  (base);
            break;
            case 'g':
                grip = !grip;
                drdRegulateGrip (grip);
            break;
            }
        }

        // retrieve position and orientationrad
        if (dhdGetPositionAndOrientationRad (&px, &py, &pz, &angle_x, &angle_y, &angle_z) < DHD_NO_ERROR) {
            RCLCPP_INFO(this->get_logger(), "error: cannot read position (%s)\n", dhdErrorGetLastStr());
            dhdClose ();
            RCLCPP_INFO(this->get_logger(), "\ndone.\n");
        }


        // retrieve force and torque
        if (dhdGetForceAndTorque (&fx, &fy, &fz, &tx, &ty, &tz) < DHD_NO_ERROR) {
            printf ("error: cannot read force (%s)\n", dhdErrorGetLastStr());
            dhdClose ();
            RCLCPP_INFO(this->get_logger(), "\ndone.\n");
        }
        
        // display status
        message.data[0] = px;
        message.data[1] = py;
        message.data[2] = pz;
        message.data[3] = angle_x;
        message.data[4] = angle_y;
        message.data[5] = angle_z;
        message.data[6] = fx;
        message.data[7] = fy;
        message.data[8] = fz;
        message.data[9] = tx;
        message.data[10] = ty;
        message.data[11] = tz;
        //RCLCPP_INFO(this->get_logger(), "位置： x = %f y = %f z = %f  姿态： angle_x = %f angle_y = %f angle_z = %f 力: fx = %f fy = %f fz = %f   力矩: tx = %f ty = %f tz = %f", message.data[0], message.data[1], message.data[2],message.data[3],message.data[4],message.data[5],message.data[6],message.data[7],message.data[8],message.data[9],message.data[10],message.data[11]);
        
        // 发布消息
        command_publisher_->publish(message);
    }
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<Omega7Sensor>("omega7_sensor");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    // close the connection
    dhdClose ();
    rclcpp::shutdown();
    return 0;
}
