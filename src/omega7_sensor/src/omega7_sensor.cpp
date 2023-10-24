#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
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
    double nullPose0[DHD_MAX_DOF] = { 0.0, 0.0, 0.0,  // translation
                                        0.0, 0.0, 0.0,  // rotation (joint angles)
                                        -0.26747439653555266 };          // gripper (distance)
    double nullPose1[DHD_MAX_DOF] = { 0.0, 0.0, 0.0,  // translation
                                        0.0, 0.0, 0.0,  // rotation (joint angles)
                                        0.0 };          // gripper (distance)
    double endPose[DHD_MAX_DOF] = { 0.0, 0.0, 0.05,  // translation
                                        0.0, 0.0, 0.0,  // rotation (joint angles)
                                        0.0 };          // gripper (distance)

    bool   base0  = false;
    bool   grip0  = false;
    bool   base1  = false;
    bool   grip1  = false;

    int deviceCount;
    int deviceID0;
    int deviceID1;
    int left = 0;
    int right = 1;
    

    // 构造函数,有一个参数为节点名称
    Omega7Sensor(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());

        //get device ID
        deviceCount = dhdGetDeviceCount ();
        if (deviceCount < 0) {
            printf ("error: %s\n", dhdErrorGetLastStr ());
        }
        else if (deviceCount < 1) {
            printf ("error: no device detected\n");
        }       
        else if (deviceCount < 2) {
            printf ("error: single device detected\n");
        }
        // open the first available device
        if ((deviceID0 = dhdOpenID (0)) < 0) {
            printf ("error: %s\n", dhdErrorGetLastStr ());
        }
        // open the second available device
        if ((deviceID1 = dhdOpenID (1)) < 0) {
            printf ("error: %s\n", dhdErrorGetLastStr ());
        }

        std::string systemName = dhdGetSystemName(1);

        //std::cout << "System Name: " << systemName << std::endl;

        if(systemName == "omega.7 (left-hand)"){
            left = 1;
            right = 0;

        }
        printf ("detected device number : %d\n", deviceCount);
        printf ("%s haptic device detected ID = 0\n\n", dhdGetSystemName (left));  //0 is left
        if(dhdIsLeftHanded(left)){printf(" the device is configured for left-handed use\n");}
        printf ("%s haptic device detected ID = 1\n\n", dhdGetSystemName (right));  //1 is right

        

        // open and initialize 2 devices
        for (int dev=0; dev<2; dev++) {

            // open device
            if (drdOpenID (dev) < 0) {
                printf ("error: not enough devices found\n");
                dhdSleep (2.0);
                for (int j=0; j<=dev; j++) drdClose (j);
            }


            // check that device is supported
            if (!drdIsSupported(dev)) {
                printf ("error: unsupported device (%s)\n", dhdGetSystemName(dev));
                dhdSleep (2.0);
                for (int j=0; j<=dev; j++) drdClose (j);
            }

            // initialize if necessary
            if (!drdIsInitialized (dev) && (drdAutoInit (dev) < 0)) {
                printf ("error: initialization failed (%s)\n", dhdErrorGetLastStr ());
                dhdSleep (2.0);
                for (int j=0; j<=dev; j++) drdClose (j);
            }

            // start robot control loop
            if (drdStart (dev) < 0) {
                printf ("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr ());
                dhdSleep (2.0);
                for (int j=0; j<=dev; j++) drdClose (j);
            }
        }

        // move to center
        drdMoveTo (nullPose0,false,left);
        drdMoveTo (nullPose1,false,right);
        // request a null force (only gravity compensation will be applied)
        // this will only apply to unregulated axis
        // drdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0,  // force
        //                                0.0, 0.0, 0.0,  // torque
        //                                0.0,left);           // gripper force
        // drdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0,  // force
        //                                0.0, 0.0, 0.0,  // torque
        //                                0.0,right);           // gripper force
                                       

        // disable all axis regulation (but leave regulation thread running)
        drdRegulatePos  (base0,left);
        drdRegulateGrip (grip0,left);

        drdRegulatePos  (base1,right);
        drdRegulateGrip (grip1,right);

        // display instructions
        //printf ("      'q' to quit\n");
        printf ("      'b' to free/hold base_left (translations)\n");
        //printf ("      'w' to free/hold wrist (rotations)\n");
        printf ("      'g' to free/hold gripper_left\n\n");
        printf ("      'a' to free/hold base_right (translations)\n");
        //printf ("      'w' to free/hold wrist (rotations)\n");
        printf ("      'j' to free/hold gripper_right\n\n");

        // display output header
        printf ("BASE_LEFT  |  ");
        if (dhdHasGripper (left))     printf ("GRIP_LEFT  |  ");
        printf ("BASE_RIGHT  |  ");
        if (dhdHasGripper (right))     printf ("GRIP_RIGHT  |  \n");
        printf ("---------------------------------------------------------\n");

        // 创建发布者
        command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("omega7_sensor", 100);
        // 创建定时器，500ms为周期，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&Omega7Sensor::timecaller, this));
    }

    ~Omega7Sensor()
    {
        RCLCPP_INFO(this->get_logger(), "Closing Omega7 devices.");
        

        dhdSleep(5.0); 

        // stop regulation and close the device
        for (int dev=0; dev<2; dev++) {
            if (drdIsRunning(dev)) {
                drdStop(dev); // stop the control loop of the device
            }
            drdClose(dev); // close the connection to the device
        }

        RCLCPP_INFO(this->get_logger(), "Omega7 devices successfully closed.");
    }


private:

    void timecaller()
    {
        double p0[DHD_MAX_DOF];
        double r0[3][3];
        double v0[DHD_MAX_DOF];
        double f0[DHD_MAX_DOF];
        double normf0, normt0;
        int    res0;

        // Eigen objects (mapped to the arrays above)
        Map<Vector3d> position0(&p0[0], 3);
        Map<Vector3d> force0   (&f0[0], 3);
        Map<Vector3d> torque0  (&f0[3], 3);
        Map<Vector3d> velpos0  (&v0[0], 3);
        Map<Vector3d> velrot0  (&v0[3], 3);
        Matrix3d center0;
        Matrix3d rotation0;

        double p1[DHD_MAX_DOF];
        double r1[3][3];
        double v1[DHD_MAX_DOF];
        double f1[DHD_MAX_DOF];
        double normf1, normt1;
        int    res1;

        // Eigen objects (mapped to the arrays above)
        Map<Vector3d> position1(&p1[0], 3);
        Map<Vector3d> force1   (&f1[0], 3);
        Map<Vector3d> torque1  (&f1[3], 3);
        Map<Vector3d> velpos1  (&v1[0], 3);
        Map<Vector3d> velrot1  (&v1[3], 3);
        Matrix3d center1;
        Matrix3d rotation1;

        double px_right, py_right, pz_right;  //末端执行器位置
        double angle_x_right,angle_y_right,angle_z_right;  //末端执行器姿态
        double gripper_angle_right;  //夹爪角度

        double px_left, py_left, pz_left;  //末端执行器位置
        double angle_x_left,angle_y_left,angle_z_left;  //末端执行器姿态
        double gripper_angle_left;  //夹爪角度

        // 创建消息
        auto message = std_msgs::msg::Float64MultiArray();
        message.data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

        /*------------------------------------------left hand---------------------------------------------------*/
        // center of workspace
        center0.setIdentity ();                           // rotation (matrix)

        // synchronize with regulation thread
        drdWaitForTick (left);

        // get position/orientation/gripper and update Eigen rotation matrix
        drdGetPositionAndOrientation (&(p0[0]), &(p0[1]), &(p0[2]),
                                  &(p0[3]), &(p0[4]), &(p0[5]),
                                  &(p0[6]), r0,left);
        for (int i=0; i<3; i++) rotation0.row(i) = Vector3d::Map(&r0[i][0], 3);

        // get position/orientation/gripper velocity
        drdGetVelocity (&(v0[0]), &(v0[1]), &(v0[2]),
                    &(v0[3]), &(v0[4]), &(v0[5]),
                    &(v0[6]),left);

        // compute base centering force
        force0 = - KP * position0;

        // compute wrist centering torque
        AngleAxisd deltaRotation0 (rotation0.transpose() * center0);
        torque0 = rotation0 * (KR * deltaRotation0.angle() * deltaRotation0.axis());

        // compute gripper centering force
        f0[6]  = - 6 * KG * (p0[6] + 0.015);

        // scale force to a pre-defined ceiling
        if ((normf0 = force0.norm()) > MAXF) force0 *= MAXF/normf0;

        // scale torque to a pre-defined ceiling
        if ((normt0 = torque0.norm()) > MAXT) torque0 *= MAXT/normt0;

        // scale gripper force to a pre-defined ceiling
        if (f0[6] >  MAXG) f0[6] =  MAXG;
        if (f0[6] < -MAXG) f0[6] = -MAXG;

        // add damping
        force0  -= KVP * velpos0;
        torque0 -= KWR * velrot0;
        f0[6]   -= KVG * v0[6];

        // apply centering force with damping
        res0 = drdSetForceAndTorqueAndGripperForce (f0[0], f0[1], f0[2],  // force
                                               f0[3], f0[4], f0[5],  // torque
                                               f0[6],left);             // gripper force
        if (res0 < DHD_NO_ERROR) {
            printf ("error: cannot set left hand force (%s)\n", dhdErrorGetLastStr ());
            drdClose (left);
        }
        /*------------------------------------------------------------------------------------------------------------*/

        /*------------------------------------right hand--------------------------------------------------------------*/
        // center of workspace
        center1.setIdentity ();                           // rotation (matrix)

        // synchronize with regulation thread
        drdWaitForTick (right);

        // get position/orientation/gripper and update Eigen rotation matrix
        drdGetPositionAndOrientation (&(p1[0]), &(p1[1]), &(p1[2]),
                                  &(p1[3]), &(p1[4]), &(p1[5]),
                                  &(p1[6]), r1,right);
        for (int i=0; i<3; i++) rotation1.row(i) = Vector3d::Map(&r1[i][0], 3);

        // get position/orientation/gripper velocity
        drdGetVelocity (&(v1[0]), &(v1[1]), &(v1[2]),
                    &(v1[3]), &(v1[4]), &(v1[5]),
                    &(v1[6]),right);

        // compute base centering force
        force1 = - KP * position1;

        // compute wrist centering torque
        AngleAxisd deltaRotation1 (rotation1.transpose() * center1);
        torque1 = rotation1 * (KR * deltaRotation1.angle() * deltaRotation1.axis());

        // compute gripper centering force
        f1[6]  = -  3 * KG * (p1[6] - 0.015);

        // scale force to a pre-defined ceiling
        if ((normf1 = force1.norm()) > MAXF) force1 *= MAXF/normf1;

        // scale torque to a pre-defined ceiling
        if ((normt1 = torque1.norm()) > MAXT) torque1 *= MAXT/normt1;

        // scale gripper force to a pre-defined ceiling
        if (f1[6] >  MAXG) f1[6] =  MAXG;
        if (f1[6] < -MAXG) f1[6] = -MAXG;

        // add damping
        force1  -= KVP * velpos1;
        torque1 -= KWR * velrot1;
        f1[6]   -= KVG * v1[6];

        // apply centering force with damping
        res1 = drdSetForceAndTorqueAndGripperForce (f1[0], f1[1], f1[2],  // force
                                               f1[3], f1[4], f1[5],  // torque
                                               f1[6],right);             // gripper force
        if (res1 < DHD_NO_ERROR) {
            printf ("error: cannot set right hand force (%s)\n", dhdErrorGetLastStr ());
            drdClose (right);
        }
        /*-------------------------------------------------------------------------------------------------------*/


        // print status
        if (base0)    printf ("HOLD  |  ");
        else         printf ("free  |  ");
        
        if (dhdHasGripper(0)) {
            if( grip0)  printf ("HOLD  |  ");
            else       printf ("free  |  ");
        }

        if (base1)    printf ("HOLD  |  ");
        else         printf ("free  |  ");
        
        if (dhdHasGripper(1)) {
            if( grip1)  printf ("HOLD  |  ");
            else       printf ("free  |  ");
        }

        printf ("\r");

        // user input
        if (dhdKbHit ()) {
            switch (dhdKbGet ()) {
            case 'b':
                base0 = !base0;
                drdRegulatePos  (base0,left);
            break;
            case 'g':
                grip0 = !grip0;
                drdRegulateGrip (grip0,left);
            break;
            case 'a':
                base1 = !base1;
                drdRegulatePos  (base1,right);
            break;
            case 'j':
                grip1 = !grip1;
                drdRegulateGrip (grip1,right);
            break;
            }
        }

        // retrieve position and orientationrad
        if (dhdGetPositionAndOrientationRad (&px_left, &py_left, &pz_left, &angle_x_left, &angle_y_left, &angle_z_left,left) < DHD_NO_ERROR) {
            RCLCPP_INFO(this->get_logger(), "error: cannot read left hand position (%s)\n", dhdErrorGetLastStr());
            dhdClose (left);
            RCLCPP_INFO(this->get_logger(), "\ndone.\n");
        }
        
        //retrieve gripper angle in rad
        if ( dhdGetGripperAngleRad(&gripper_angle_left,left)< DHD_NO_ERROR) {
            RCLCPP_INFO(this->get_logger(), "error: cannot read left hand gripper angle (%s)\n", dhdErrorGetLastStr());
            dhdClose (left);
            RCLCPP_INFO(this->get_logger(), "\ndone.\n");
        }

        // retrieve position and orientationrad
        if (dhdGetPositionAndOrientationRad (&px_right, &py_right, &pz_right, &angle_x_right, &angle_y_right, &angle_z_right,right) < DHD_NO_ERROR) {
            RCLCPP_INFO(this->get_logger(), "error: cannot read right hand position (%s)\n", dhdErrorGetLastStr());
            dhdClose (right);
            RCLCPP_INFO(this->get_logger(), "\ndone.\n");
        }
        
        //retrieve gripper angle in rad
        if ( dhdGetGripperAngleRad(&gripper_angle_right,right)< DHD_NO_ERROR) {
            RCLCPP_INFO(this->get_logger(), "error: cannot read right hand gripper angle (%s)\n", dhdErrorGetLastStr());
            dhdClose (right);
            RCLCPP_INFO(this->get_logger(), "\ndone.\n");
        }

        // display status
        message.data[0] = px_left;
        message.data[1] = py_left;
        message.data[2] = pz_left;
        message.data[3] = angle_x_left;
        message.data[4] = angle_y_left;
        message.data[5] = angle_z_left;
        message.data[6] = gripper_angle_left;
        message.data[7] = px_right;
        message.data[8] = py_right;
        message.data[9] = pz_right;
        message.data[10] = angle_x_right;
        message.data[11] = angle_y_right;
        message.data[12] = angle_z_right;
        message.data[13] = gripper_angle_right;
        //RCLCPP_INFO(this->get_logger(), "位置： x = %f y = %f z = %f  姿态： angle_x = %f angle_y = %f angle_z = %f 力: fx = %f fy = %f fz = %f   力矩: tx = %f ty = %f tz = %f", message.data[0], message.data[1], message.data[2],message.data[3],message.data[4],message.data[5],message.data[6],message.data[7],message.data[8],message.data[9],message.data[10],message.data[11]);
        
        // 发布消息
        command_publisher_->publish(message);
    }
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
};

    
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<Omega7Sensor>("omega7_sensor");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}