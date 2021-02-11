/** @file client.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  All the exampls for ROS are implemented here.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 *  ALL IN ONE for drone
 *  Modified by ChuaChongEN
 */



#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iostream>
#include <string>
#include <fstream>
#include<stdio.h> // Libraries for reading characters from the keyboard
#include<conio.h> // Libraries for reading characters from the keyboard


using namespace DJI::onboardSDK;
using namespace std;

//! Function Prototypes for Mobile command callbacks - Core Functions
void ObtainControlMobileCallback(DJIDrone *drone);
void ReleaseControlMobileCallback(DJIDrone *drone);
void TakeOffMobileCallback(DJIDrone *drone);
void LandingMobileCallback(DJIDrone *drone);
void GetSDKVersionMobileCallback(DJIDrone *drone);
void ArmMobileCallback(DJIDrone *drone);
void DisarmMobileCallback(DJIDrone *drone);
void GoHomeMobileCallback(DJIDrone *drone);
void TakePhotoMobileCallback(DJIDrone *drone);
void StartVideoMobileCallback(DJIDrone *drone);
void StopVideoMobileCallback(DJIDrone *drone);
//! Function Prototypes for Mobile command callbacks - Custom Missions
void DrawCircleDemoMobileCallback(DJIDrone *drone);
void DrawSquareDemoMobileCallback(DJIDrone *drone);
void GimbalControlDemoMobileCallback(DJIDrone *drone);
void AttitudeControlDemoMobileCallback(DJIDrone *drone);
void LocalNavigationTestMobileCallback(DJIDrone *drone);
void GlobalNavigationTestMobileCallback(DJIDrone *drone);
void WaypointNavigationTestMobileCallback(DJIDrone *drone);
void VirtuaRCTestMobileCallback(DJIDrone *drone);

//! For LAS logging
void StartMapLASLoggingMobileCallback(DJIDrone *drone);
void StopMapLASLoggingMobileCallback(DJIDrone *drone);
void StartCollisionAvoidanceCallback(DJIDrone *drone);
void StopCollisionAvoidanceCallback(DJIDrone *drone);


static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+--------------------------------------------------- < Main menu > -------------------------------------------------+\n");
	printf("| [1]  SDK Version Query        | [17] Arm the Drone                                                            |\n");
	printf("| [2]  Request Control          | [18] Disarm the Drone                                                         |\n");
	printf("| [3]  Release Control          | [19] TakeOff->Hover->land (Autonomous)                                        |\n");
	printf("| [4]  Takeoff                  | [20] TakeOff->Hover->PitchFwd->Hover->PitchBackwards->Hover->Land (Autonomous)|\n");
	printf("| [5]  Landing                  | [21] TakeOff->Hover->RollRight->Hover->RollLeft->Hover->Land(Autonomous)      |\n");
        printf("| [x]  Empty                    | [22] TakeOff->Hover->YawRight->Hover->YawLeft->Hover->Land (Autonomous)       |\n");
        printf("| [x]  Empty                    | [23] Draw A Square (Autonomous)                                               |\n");  
        printf("| [x]  Empty                    | [24] Keyboard Control                                                         |\n"); 
        printf("| [x]  Empty                    | [25] Wall Tracing (Autonomous)                                                |\n");
        printf("| [x]  Empty                    | [26] Wall Tracing Inward(Autonomous)                                          |\n"); 
        printf("| [x]  Empty                    | [27] Wall Tracing Outward(Autonomous)                                         |\n");  
        printf("| [x]  Empty                    | [28] Wall Tracing -Refined (Autonomous)                                       |\n");                         
    printf("+-------------------------------------------------------------------------------------------------------------------+\n");
    printf("input 1/2/3/4/5/17/18/19/20/21/22/23 then press enter key\r\n");
    printf("Press 'Q' to terminate VirtualRC Flight and return to manual flight\r\n");
    printf("-------------------------------------------------------------------------------------------------------------------\r\n");
}

int main(int argc, char *argv[])
{
    int main_operate_code = 0;
    int temp32;
    int circleRadius;
    int circleHeight;
    float Phi, circleRadiusIncrements;
    int x_center, y_center, yaw_local;
    bool valid_flag = false;
    bool err_flag = false;
    ros::init(argc, argv, "sdk_client");
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle nh;
    DJIDrone* drone = new DJIDrone(nh);

	//virtual RC test data
	uint32_t virtual_rc_data[16];

	//set frequency test data
	uint8_t msg_frequency_data[16] = {1,2,3,4,3,2,1,2,3,4,3,2,1,2,3,4};
	//waypoint action test data
    dji_sdk::WaypointList newWaypointList;
    dji_sdk::Waypoint waypoint0;
    dji_sdk::Waypoint waypoint1;
    dji_sdk::Waypoint waypoint2;
    dji_sdk::Waypoint waypoint3;
    dji_sdk::Waypoint waypoint4;

	//groundstation test data
	dji_sdk::MissionWaypointTask waypoint_task;
	dji_sdk::MissionWaypoint 	 waypoint;
	dji_sdk::MissionHotpointTask hotpoint_task;
	dji_sdk::MissionFollowmeTask followme_task;
	dji_sdk::MissionFollowmeTarget followme_target;
    uint8_t userData = 0;
    ros::spinOnce();

    //! Setting functions to be called for Mobile App Commands mode
    drone->setObtainControlMobileCallback(ObtainControlMobileCallback, &userData);
    drone->setReleaseControlMobileCallback(ReleaseControlMobileCallback, &userData);
    drone->setTakeOffMobileCallback(TakeOffMobileCallback, &userData);
    drone->setLandingMobileCallback(LandingMobileCallback, &userData);
    drone->setGetSDKVersionMobileCallback(GetSDKVersionMobileCallback, &userData);
    drone->setArmMobileCallback(ArmMobileCallback, &userData);
    drone->setDisarmMobileCallback(DisarmMobileCallback, &userData);
    drone->setGoHomeMobileCallback(GoHomeMobileCallback, &userData);
    drone->setTakePhotoMobileCallback(TakePhotoMobileCallback, &userData);
    drone->setStartVideoMobileCallback(StartVideoMobileCallback,&userData);
    drone->setStopVideoMobileCallback(StopVideoMobileCallback,&userData);
    drone->setDrawCircleDemoMobileCallback(DrawCircleDemoMobileCallback, &userData);
    drone->setDrawSquareDemoMobileCallback(DrawSquareDemoMobileCallback, &userData);
    drone->setGimbalControlDemoMobileCallback(GimbalControlDemoMobileCallback, &userData);
    drone->setAttitudeControlDemoMobileCallback(AttitudeControlDemoMobileCallback, &userData);
    drone->setLocalNavigationTestMobileCallback(LocalNavigationTestMobileCallback, &userData);
    drone->setGlobalNavigationTestMobileCallback(GlobalNavigationTestMobileCallback, &userData);
    drone->setWaypointNavigationTestMobileCallback(WaypointNavigationTestMobileCallback, &userData);
    drone->setVirtuaRCTestMobileCallback(VirtuaRCTestMobileCallback, &userData);

    drone->setStartMapLASLoggingMobileCallback(StartMapLASLoggingMobileCallback, &userData);
    drone->setStopMapLASLoggingMobileCallback(StopMapLASLoggingMobileCallback, &userData);
    drone->setStartCollisionAvoidanceCallback(StartCollisionAvoidanceCallback, &userData);
    drone->setStopCollisionAvoidanceCallback(StopCollisionAvoidanceCallback, &userData);


    Display_Main_Menu();
    while(1)
    {
        ros::spinOnce();
        std::cout << "Enter Input Val: ";
        while(!(std::cin >> temp32)){
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input.  Try again: ";
	}

        if(temp32>0 && temp32<38)
        {
            main_operate_code = temp32;
        }
        else
        {
            printf("ERROR - Out of range Input \n");
            Display_Main_Menu();
            continue;
        }
        switch(main_operate_code)
        {
			case 1:
				/* SDK version query*/
				drone->check_version();
				break;
            case 2:
                /* request control ability*/
                drone->request_sdk_permission_control();
                break;
            case 3:
                /* release control ability*/
                drone->release_sdk_permission_control();
                break;
            case 4:
                /* take off */
                drone->takeoff();
                break;
            case 5:
                /* landing*/
                drone->landing();
                break;
            case 6:
                /* go home*/
                drone->gohome();
                break;
            case 7:
                /*gimbal test*/

                drone->gimbal_angle_control(0, 0, 1800, 20);
                sleep(2);
                drone->gimbal_angle_control(0, 0, -1800, 20);
                sleep(2);
                drone->gimbal_angle_control(300, 0, 0, 20);
                sleep(2);
                drone->gimbal_angle_control(-300, 0, 0, 20);
                sleep(2);
                drone->gimbal_angle_control(0, 300, 0, 20);
                sleep(2);
                drone->gimbal_angle_control(0, -300, 0, 20);
                sleep(2);
                drone->gimbal_speed_control(100, 0, 0);
                sleep(2);
                drone->gimbal_speed_control(-100, 0, 0);
                sleep(2);
                drone->gimbal_speed_control(0, 0, 200);
                sleep(2);
                drone->gimbal_speed_control(0, 0, -200);
                sleep(2);
                drone->gimbal_speed_control(0, 200, 0);
                sleep(2);
                drone->gimbal_speed_control(0, -200, 0);
                sleep(2);
                drone->gimbal_angle_control(0, 0, 0, 20);
                break;

            case 8:
                /* attitude control sample*/
                drone->takeoff();
                sleep(8);


                for(int i = 0; i < 100; i ++)
                {
                    if(i < 90)
                        drone->attitude_control(0x40, 0, 2, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 2, 0, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, -2, 0, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, 2, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, -2, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, 0, 0.5, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, 0, -0.5, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0xA, 0, 0, 0, 90);
                    else
                        drone->attitude_control(0xA, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0xA, 0, 0, 0, -90);
                    else
                        drone->attitude_control(0xA, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                drone->landing();

                break;

            case 9:
                /*draw circle sample*/
                static float R = 2;
                static float V = 2;
                static float x;
                static float y;
                Phi = 0;
                std::cout<<"Enter the radius of the circle in meteres (10m > x > 4m)\n";
                std::cin>>circleRadius;

                std::cout<<"Enter height in meteres (Relative to take off point. 15m > x > 5m) \n";
                std::cin>>circleHeight;

                 if (circleHeight < 5)
                {
                    circleHeight = 5;
                }
                else if (circleHeight > 15)
                {
                    circleHeight = 15;
                }
                if (circleRadius < 4)
                {
                    circleRadius = 4;
                }
                else if (circleRadius > 10)
                {
                    circleRadius = 10;
                }

                x_center = drone->local_position.x;
                y_center = drone->local_position.y;
                circleRadiusIncrements = 0.01;

    		    for(int j = 0; j < 1000; j ++)
                {
                  if (circleRadiusIncrements < circleRadius)
    			   {
    		        x =  x_center + circleRadiusIncrements;
    		        y =  y_center;
    			    circleRadiusIncrements = circleRadiusIncrements + 0.01;
    		        drone->local_position_control(x ,y ,circleHeight, 0);
    		        usleep(20000);
    			  }
                   else
    			   {
                    break;
                   }
                }

                /* start to draw circle */
                for(int i = 0; i < 1890; i ++)
                {
                    x =  x_center + circleRadius*cos((Phi/300));
                    y =  y_center + circleRadius*sin((Phi/300));
                    Phi = Phi+1;
                    drone->local_position_control(x ,y ,circleHeight, 0);
                    usleep(20000);
                }
                break;

            case 10:
                /*draw square sample*/
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            3, 3, 0, 0 );
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            -3, 3, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            -3, -3, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            3, -3, 0, 0);
                    usleep(20000);
                }
                break;
            case 11:
                /*take a picture*/
                drone->take_picture();
                break;
            case 12:
                /*start video*/
                drone->start_video();
                break;
            case 13:
                /*stop video*/
                drone->stop_video();
                break;
            case 14:
                /* Local Navi Test */
                drone->local_position_navigation_send_request(-100, -100, 100);
                break;
            case 15:
                /* GPS Navi Test */
                drone->global_position_navigation_send_request(22.5420, 113.9580, 10);
                break;
            case 16:
                /* Waypoint List Navi Test */
                {
                    waypoint0.latitude = 22.535;
                    waypoint0.longitude = 113.95;
                    waypoint0.altitude = 100;
                    waypoint0.staytime = 5;
                    waypoint0.heading = 0;
                }
                newWaypointList.waypoint_list.push_back(waypoint0);

                {
                    waypoint1.latitude = 22.535;
                    waypoint1.longitude = 113.96;
                    waypoint1.altitude = 100;
                    waypoint1.staytime = 0;
                    waypoint1.heading = 90;
                }
                newWaypointList.waypoint_list.push_back(waypoint1);

                {
                    waypoint2.latitude = 22.545;
                    waypoint2.longitude = 113.96;
                    waypoint2.altitude = 100;
                    waypoint2.staytime = 4;
                    waypoint2.heading = -90;
                }
                newWaypointList.waypoint_list.push_back(waypoint2);

                {
                    waypoint3.latitude = 22.545;
                    waypoint3.longitude = 113.96;
                    waypoint3.altitude = 10;
                    waypoint3.staytime = 2;
                    waypoint3.heading = 180;
                }
                newWaypointList.waypoint_list.push_back(waypoint3);

                {
                    waypoint4.latitude = 22.525;
                    waypoint4.longitude = 113.93;
                    waypoint4.altitude = 50;
                    waypoint4.staytime = 0;
                    waypoint4.heading = -180;
                }
                newWaypointList.waypoint_list.push_back(waypoint4);

                drone->waypoint_navigation_send_request(newWaypointList);
                break;
			case 17:
				//drone arm
				drone->drone_arm();
                break;
			case 18:
				//drone disarm
				drone->drone_disarm();
                break;

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			case 19:
			 {
                                  double throttle1 = 25;
                                  double throttle2 = 50;
                                  double throttle3 = 75;
                                  double throttle4 = 100;
                                  double roll = 0;
                                  double yaw = 0; 
                                  double pitch = 0;
                                  char killkey; //Variable for killing the code and return to manual flight. 
                                  //Automatic Throttle Up
                                    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone taking off..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024+ throttle1; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 50; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey =getche();
                                                  if(killkey == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                         }

                                    //Automatic Throttle Up
                                    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone taking off..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024+ throttle2; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 50; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey =getche();
                                                  if(killkey == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                         }

                                    //Automatic Throttle Up
                                    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone taking off..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024+ throttle3; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 50; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey =getche();
                                                  if(killkey == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                         }

                                    //Automatic Throttle Up
                                    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone taking off..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024+ throttle4; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 50; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey =getche();
                                                  if(killkey == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                         }
                                    cout<< "End of drone takeoff..." <<endl;


			            //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey =getche();
                                                  if(killkey == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                        
				         }
                                    cout<< "End of drone hover..." <<endl;

                                    //Automatic Landing	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone descending..." <<endl;
                                    virtual_rc_data[0] = 1024;          //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;          //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024-throttle4; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;          //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;          //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;          //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(130000);
                                          while (kbhit())
                                                 {
                                                  killkey =getche();
                                                  if(killkey == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }

				         }

                                cout<< "End of decend & drone disarm." <<endl;
                                drone->virtual_rc_enable();
				drone->landing();
				break;
                         

			}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			case 20:
                                {
				//TAKEOFF->HOVER->PITCHFORWARD->HOVER->PITCHBACKWARDS-HOVER->LAND
                                char killkey1;
                                double throttle = 100;
                                double roll = 0;
                                double yaw = 0;
                                double pitch = 1; //50
			        //Automatic Throttle Up
                                    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone taking off..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024+ throttle; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 150; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey1 =getche();
                                                  if(killkey1 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                         }
                                    
                                    cout<< "End of drone takeoff..." <<endl;


			            //Automatic hover 5 seconds
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey1 =getche();
                                                  if(killkey1 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;


                                    //Pitch forward	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone Pitching Forward for 2meters..." <<endl;
                                    virtual_rc_data[0] = 1024;         //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024+ pitch;  //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;         //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;         //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;         //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;         //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 150; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey1 =getche();
                                                  if(killkey1 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }
                                    cout<< "Drone finish pitching forwards..." <<endl; 

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey1 =getche();
                                                  if(killkey1 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;
                                    
                                    //Pitch backwards	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone Pitching Backwards for 2 meters back to original position..." <<endl;
                                    virtual_rc_data[0] = 1024;         //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024- pitch;  //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;         //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;         //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;         //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;         //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 150; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey1 =getche();
                                                  if(killkey1 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }  
                                    
                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey1 =getche();
                                                  if(killkey1 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;
                                    
                                    //Automatic Landing	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone descending..." <<endl;
                                    virtual_rc_data[0] = 1024;     //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;     //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024-100; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;     //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;     //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;     //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(100000);
                                          while (kbhit())
                                                 {
                                                  killkey1 =getche();
                                                  if(killkey1 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }

                                cout<< "End of decend & drone disarm." <<endl;
                                drone->virtual_rc_enable();
				drone->landing();
				break;
                                }

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			case 21:
                                {
				//TAKEOFF->HOVER->ROLLRIGHT->HOVER->ROLLLEFT-HOVER->LAND
                                double throttle = 100;
                                double roll = 1; //50
                                double yaw = 0; 
                                double pitch = 0;
                                char killkey2;
                                //Automatic Throttle Up
                                    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone taking off..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024+ throttle; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 150; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey2 =getche();
                                                  if(killkey2 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                         }
                                    
                                    cout<< "End of drone takeoff..." <<endl;


			            //Automatic hover 5 seconds
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey2 =getche();
                                                  if(killkey2 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;


                                    //Roll right for 2 meters
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone rolling 2meters to the right..." <<endl;
                                    virtual_rc_data[0] = 1024+ roll;  //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;        //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;        //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;        //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;        //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;        //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 150; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey2 =getche();
                                                  if(killkey2 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }
                                    cout<< "Drone finish rolling right..." <<endl; 

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey2 =getche();
                                                  if(killkey2 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;
                                    
                                    //Roll left for 2 meters	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone rolling 2 meters to the left..." <<endl;
                                    virtual_rc_data[0] = 1024- roll;  //0-> roll      [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;        //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;        //2-> throttle  [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;        //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;        //4-> gear      {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;        //6-> mode      {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 150; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey2 =getche();
                                                  if(killkey2 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }  
                                    cout<< "Drone finish rolling left..." <<endl; 
                                    
                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey2 =getche();
                                                  if(killkey2 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;
                                    
                                    //Automatic Landing	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone decending..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024- throttle; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}
 
                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(100000);
                                          while (kbhit())
                                                 {
                                                  killkey2 =getche();
                                                  if(killkey2 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }

                                cout<< "End of decend & drone disarm." <<endl;
                                usleep(20000);
                                drone->virtual_rc_enable();
				drone->landing();
				break;
                                }

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			case 22:
                                {
                                //TAKEOFF->HOVER->YAWRIGHT->HOVER->YAWLEFT-HOVER->LAND
                                double throttle = 100;
                                double roll = 0;
                                double yaw = 100; 
                                double pitch = 0;
                                char killkey3;
                                //Automatic Throttle Up
                                    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone taking off..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024+ throttle; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 150; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey3 =getche();
                                                  if(killkey3 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                         }
                                    
                                    cout<< "End of drone takeoff..." <<endl;


			            //Automatic hover 5 seconds
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey3 =getche();
                                                  if(killkey3 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;


                                    //Yawing 90 degrees to the right
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone yawing 90 degrees to the right..." <<endl;
                                    virtual_rc_data[0] = 1024;      //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;      //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;      //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024+ yaw; //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;      //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;      //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 450; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey3 =getche();
                                                  if(killkey3 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }
                                    cout<< "Drone finish yawing right..." <<endl; 

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey3 =getche();
                                                  if(killkey3 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;
                                    
                                    //Yawing 90 degrees to the left	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone yawing 90 degrees to the left..." <<endl;
                                    virtual_rc_data[0] = 1024;      //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;      //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;      //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024- yaw; //3-> yaw     [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;      //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;      //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 450; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey3 =getche();
                                                  if(killkey3 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }  
                                    cout<< "Drone finish yawing left..." <<endl; 
                                    
                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey3 =getche();
                                                  if(killkey3 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;
                                    
                                    //Automatic Landing	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone descending..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024- throttle; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(130000);
                                          while (kbhit())
                                                 {
                                                  killkey3 =getche();
                                                  if(killkey3 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }

                                cout<< "End of decend & drone disarm." <<endl;
                                usleep(20000);
                                drone->virtual_rc_enable();
				drone->landing();
				break;

                               }
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------                
			case 23:
                                {
				//DRAW A SQUARE (2 Level wall tracing)
                                double throttle = 100;
                                double roll = 1; //50
                                double yaw = 100; 
                                double pitch = 1; //50
                                char killkey4;
                                //Automatic Throttle Up
                                    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone taking off..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024+ throttle; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 180; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                         }
                                    
                                    cout<< "End of drone takeoff..." <<endl;

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;
                                    
                                    //Pitch forward	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone Pitching Forward for 2meters..." <<endl;
                                    virtual_rc_data[0] = 1024;         //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024+ pitch;  //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;         //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;         //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;         //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;         //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 180; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }
                                    cout<< "Drone finish pitching forwards..." <<endl; 

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;

                                    //START DRAW A SQUARE (Level 1)
                                  
                                    //Roll right for 2 meters
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone will now start to draw a square..." <<endl;
                                    cout<< "Drone rolling 2meters to the right..." <<endl;
                                    virtual_rc_data[0] = 1024+ roll;  //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;        //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;        //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;        //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;        //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;        //6-> mode     {1552(P), 1024(A), 496(F)}
  
                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
	                                  while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
				         }
                                    cout<< "Drone finish rolling right..." <<endl; 

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;
           
                                    //Yawing 90 degrees to the left	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone yawing 90 degrees to the left..." <<endl;
                                    virtual_rc_data[0] = 1024;      //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;      //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024- yaw; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;      //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;      //6-> mode     {1552(P), 1024(A), 496(F)}
 
                                    for (int i = 0; i < 450; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }  
                                    cout<< "Drone finish yawing left..." <<endl;

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;  

                                    //Roll right for 4 meters
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone rolling 4meters to the right..." <<endl;
                                    virtual_rc_data[0] = 1024+ roll;  //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;        //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;        //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;        //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;        //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;        //6-> mode     {1552(P), 1024(A), 496(F)}
  
                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(40000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }
                                    cout<< "Drone finish rolling right..." <<endl; 

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;
                     
                                    //Yawing 90 degrees to the left	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone yawing 90 degrees to the left..." <<endl;
                                    virtual_rc_data[0] = 1024;      //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;      //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024- yaw; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;      //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;      //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 480; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }  
                                    cout<< "Drone finish yawing left..." <<endl;

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;

                                    //Roll right for 4 meters
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone rolling 4meters to the right..." <<endl;
                                    virtual_rc_data[0] = 1024+ roll;  //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;        //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;        //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;        //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;        //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;        //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(40000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }
                                    cout<< "Drone finish rolling right..." <<endl;

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;

                                    //Yawing 90 degrees to the left	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone yawing 90 degrees to the left..." <<endl;
                                    virtual_rc_data[0] = 1024;      //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;      //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024- yaw; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;      //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;      //6-> mode     {1552(P), 1024(A), 496(F)}
 
                                    for (int i = 0; i < 450; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }  
                                    cout<< "Drone finish yawing left..." <<endl; 

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;

                                    //Roll right for 4 meters
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone rolling 4meters to the right..." <<endl;
                                    virtual_rc_data[0] = 1024+ roll; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;       //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;       //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;       //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;       //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;       //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(40000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }
                                    cout<< "Drone finish rolling right..." <<endl;

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;

                                    //Yawing 90 degrees to the left	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone yawing 90 degrees to the left..." <<endl;
                                    virtual_rc_data[0] = 1024;      //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;      //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024- yaw; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;      //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;      //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 480; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }  
                                    cout<< "Drone finish yawing left..." <<endl;

                                    //Automatic hover
 				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;

                                    //Roll right for 2 meters
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone rolling 2meters to the right..." <<endl;
                                    virtual_rc_data[0] = 1024+ roll; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;       //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;       //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;       //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;       //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;       //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }
                                    cout<< "Drone finish rolling right..." <<endl; 

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;

                                    //---------------------Increase height----------------------------- 
                                    //Automatic Throttle Up
                                    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone taking off..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024+ throttle; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 180; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                         }
                                    
                                    cout<< "End of drone takeoff..." <<endl;

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;
                                    
              
                                    
                                    //START DRAW A SQUARE (level 2)
                                  
                                    //Roll right for 2 meters
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone will now start to draw a square..." <<endl;
                                    cout<< "Drone rolling 2meters to the right..." <<endl;
                                    virtual_rc_data[0] = 1024+ roll;  //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;        //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;        //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;        //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;        //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;        //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
	                                  while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
				         }
                                    cout<< "Drone finish rolling right..." <<endl; 

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;
           
                                    //Yawing 90 degrees to the left	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone yawing 90 degrees to the left..." <<endl;
                                    virtual_rc_data[0] = 1024;      //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;      //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024- yaw; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;      //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;      //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 450; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }  
                                    cout<< "Drone finish yawing left..." <<endl;

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;  

                                    //Roll right for 4 meters
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone rolling 4meters to the right..." <<endl;
                                    virtual_rc_data[0] = 1024+ roll;  //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;        //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;        //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;        //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;        //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;        //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(40000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }
                                    cout<< "Drone finish rolling right..." <<endl; 

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;
                     
                                    //Yawing 90 degrees to the left	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone yawing 90 degrees to the left..." <<endl;
                                    virtual_rc_data[0] = 1024;      //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;      //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024- yaw; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;      //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;      //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 480; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }  
                                    cout<< "Drone finish yawing left..." <<endl;

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;

                                    //Roll right for 4 meters
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone rolling 4meters to the right..." <<endl;
                                    virtual_rc_data[0] = 1024+ roll;  //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;        //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;        //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;        //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;        //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;        //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(40000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }
                                    cout<< "Drone finish rolling right..." <<endl;

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;

                                    //Yawing 90 degrees to the left	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone yawing 90 degrees to the left..." <<endl;
                                    virtual_rc_data[0] = 1024;      //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;      //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024- yaw; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;      //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;      //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 450; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }  
                                    cout<< "Drone finish yawing left..." <<endl; 

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;

                                    //Roll right for 4 meters
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone rolling 4meters to the right..." <<endl;
                                    virtual_rc_data[0] = 1024+ roll;  //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;        //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;        //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;        //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;        //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;        //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(40000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }
                                    cout<< "Drone finish rolling right..." <<endl;

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;

                                    //Yawing 90 degrees to the left	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone yawing 90 degrees to the left..." <<endl;
                                    virtual_rc_data[0] = 1024;      //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;      //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024- yaw; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;      //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;      //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 480; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }  
                                    cout<< "Drone finish yawing left..." <<endl;

                                    //Automatic hover
 				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;

                                    //Roll right for 2 meters
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone rolling 2meters to the right..." <<endl;
                                    virtual_rc_data[0] = 1024+ roll;  //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;        //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;        //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;        //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;        //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;        //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }
                                    cout<< "Drone finish rolling right..." <<endl; 

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;

                                    //Pitch backwards	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone Pitching Backwards for 2 meters back to original position..." <<endl;
                                    virtual_rc_data[0] = 1024;         //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024- pitch;  //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024;         //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;         //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;         //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;         //6-> mode     {1552(P), 1024(A), 496(F)}
   
                                    for (int i = 0; i < 180; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }  

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover..." <<endl;
                  
                                    //Automatic Landing	
				    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone decending..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024- throttle; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 380; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(130000);
                                          while (kbhit())
                                                 {
                                                  killkey4 =getche();
                                                  if(killkey4 == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
	                                 
				         }

                                cout<< "End of decend & drone disarm." <<endl;
                                usleep(20000);
                                drone->virtual_rc_enable();
				drone->landing();
				break;
                                }

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			case 24:
				{ 
                          double throttle = 100;
                          double roll = 1; //75
                          double yaw = 100; 
                          double pitch = 1; //75
                          char key1; //Declare for keyboard control
                          char killkey5; //Varaible to terminate the code
                                {
                                  //Automatic Throttle Up
                                    cout<< "Drone taking-off to a safe height." <<endl;
                                    drone->virtual_rc_enable();
                                    usleep(20000);
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024+ throttle; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                         }

                                   cout<< "This program will allow user to control the drone with the keyboard controls." <<endl;
                                   cout<< "These are the follow controls: " <<endl;
                                   cout<< "Key in (w) -Move Forward" <<endl;
                                   cout<< "Key in (A) -Roll Left" <<endl;
                                   cout<< "Key in (S) -Move Backwards" <<endl;
                                   cout<< "key in (D) -Roll Right" <<endl;
                                   cout<< "Key in (I) -Pitch Up" <<endl;
                                   cout<< "Key in (K) -Pitch Down" <<endl;
                                   cout<< "Key in (J) -Yaw Left" <<endl;
                                   cout<< "Key in (L) -Yaw Right" <<endl;
                                   cout<< "Please key in the command you want: " <<endl;
                                   cout<< "Please press 'q' to end the code." <<endl;

                                   for (;;)
                                       {
                                        key1 = getche();//It waits for keyboard input
                                        cout<< "Please key in the command you want: " <<endl;
                                        killkey5 = getche();
                                        if (killkey5 == 'q')
                                           {
                                            return 0;
                                           }

                                        else 
                                           {
                                        
                                        if (key1 == 'i')
                                            {
                                              //Throttle up
                                              cout<< "Drone pitching up... " <<endl;
                                              drone->virtual_rc_enable();
                                              usleep(20000);
                                              virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                              virtual_rc_data[1] = 1024;           //1-> pitch     [1024-660,1024+660]
                                              virtual_rc_data[2] = 1024+ throttle; //2-> throttle [1024-660,1024+660]
                                              virtual_rc_data[3] = 1024;           //3-> yaw       [1024-660,1024+660]
                                              virtual_rc_data[4] = 1684;           //4-> gear {1684(UP), 1324(DOWN)}
                                              virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                                 for (int i = 0; i < 10; i++)
                                                      {
                                                       drone->virtual_rc_control(virtual_rc_data);
                                                       usleep(20000);
                                                       
                                                      }
                                            }

                                        else if (key1 == 'k')
                                                 {
                                                  //Throttle down
                                                  cout<< "Drone throttling down... " <<endl;
                                                  drone->virtual_rc_enable();
                                                  usleep(20000);
                                                  virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                                  virtual_rc_data[1] = 1024;           //1-> pitch     [1024-660,1024+660]
                                                  virtual_rc_data[2] = 1024- throttle; //2-> throttle [1024-660,1024+660]
                                                  virtual_rc_data[3] = 1024;           //3-> yaw       [1024-660,1024+660]
                                                  virtual_rc_data[4] = 1684;           //4-> gear {1684(UP), 1324(DOWN)}
                                                  virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                                     for (int i = 0; i < 10; i++)
                                                          {
                                                           drone->virtual_rc_control(virtual_rc_data);
                                                           usleep(20000);
                                                           
                                                          }
                                                 }

                                        else if (key1 == 'w')
                                                 {
                                                  //Pitch forward
                                                  cout<< "Drone moving forward... " <<endl;
                                                  drone->virtual_rc_enable();
                                                  usleep(20000);
                                                  virtual_rc_data[0] = 1024;        //0-> roll     [1024-660,1024+660]
                                                  virtual_rc_data[1] = 1024+ pitch; //1-> pitch     [1024-660,1024+660]
                                                  virtual_rc_data[2] = 1024;        //2-> throttle [1024-660,1024+660]
                                                  virtual_rc_data[3] = 1024;        //3-> yaw       [1024-660,1024+660]
                                                  virtual_rc_data[4] = 1684;        //4-> gear {1684(UP), 1324(DOWN)}
                                                  virtual_rc_data[6] = 1552;        //6-> mode     {1552(P), 1024(A), 496(F)}

                                                     for (int i = 0; i < 10; i++)
                                                          {
                                                           drone->virtual_rc_control(virtual_rc_data);
                                                           usleep(20000);
                                                          
                                                          }

                                                 }

                                        else if (key1 == 's')
                                                 {
                                                  //Pitch backward
                                                  cout<< "Drone moving backwards... " <<endl;
                                                  drone->virtual_rc_enable();
                                                  usleep(20000);
                                                  virtual_rc_data[0] = 1024;        //0-> roll     [1024-660,1024+660]
                                                  virtual_rc_data[1] = 1024- pitch; //1-> pitch     [1024-660,1024+660]
                                                  virtual_rc_data[2] = 1024;        //2-> throttle [1024-660,1024+660]
                                                  virtual_rc_data[3] = 1024;        //3-> yaw       [1024-660,1024+660]
                                                  virtual_rc_data[4] = 1684;        //4-> gear {1684(UP), 1324(DOWN)}
                                                  virtual_rc_data[6] = 1552;        //6-> mode     {1552(P), 1024(A), 496(F)}

                                                     for (int i = 0; i < 10; i++)
                                                          {
                                                           drone->virtual_rc_control(virtual_rc_data);
                                                           usleep(20000);
                                                           
                                                          }

                                                 }

                                        else if (key1 == 'a')
                                                {
                                                 //Roll left
                                                 cout<< "Drone rolling left... " <<endl;
                                                 drone->virtual_rc_enable();
                                                 usleep(20000);
                                                 virtual_rc_data[0] = 1024- roll; //0-> roll     [1024-660,1024+660]
                                                 virtual_rc_data[1] = 1024;       //1-> pitch     [1024-660,1024+660]
                                                 virtual_rc_data[2] = 1024;       //2-> throttle [1024-660,1024+660]
                                                 virtual_rc_data[3] = 1024;       //3-> yaw       [1024-660,1024+660]
                                                 virtual_rc_data[4] = 1684;       //4-> gear {1684(UP), 1324(DOWN)}
                                                 virtual_rc_data[6] = 1552;       //6-> mode     {1552(P), 1024(A), 496(F)}

                                                    for (int i = 0; i < 10; i++)
                                                         {
                                                          drone->virtual_rc_control(virtual_rc_data);
                                                          usleep(20000);
                                                          
                                                         }

                                                }
                           
                                        else if (key1 == 'd')
                                                 {
                                                  //Roll right
                                                  cout<< "Drone rolling right... " <<endl;
                                                  drone->virtual_rc_enable();
                                                  usleep(20000);
                                                  virtual_rc_data[0] = 1024+ roll; //0-> roll     [1024-660,1024+660]
                                                  virtual_rc_data[1] = 1024;       //1-> pitch     [1024-660,1024+660]
                                                  virtual_rc_data[2] = 1024;       //2-> throttle [1024-660,1024+660]
                                                  virtual_rc_data[3] = 1024;       //3-> yaw       [1024-660,1024+660]
                                                  virtual_rc_data[4] = 1684;       //4-> gear {1684(UP), 1324(DOWN)}
                                                  virtual_rc_data[6] = 1552;       //6-> mode     {1552(P), 1024(A), 496(F)}

                                                     for (int i = 0; i < 10; i++)
                                                          {
                                                           drone->virtual_rc_control(virtual_rc_data);
                                                           usleep(20000);
                                                           
                                                          }
                                                 }

                                        else if (key1 == 'j')
                                                 {
                                                  //Yaw left
                                                  cout<< "Drone yawing left... " <<endl;
                                                  drone->virtual_rc_enable();
                                                  usleep(20000);
                                                  virtual_rc_data[0] = 1024;      //0-> roll     [1024-660,1024+660]
                                                  virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
                                                  virtual_rc_data[2] = 1024;      //2-> throttle [1024-660,1024+660]
                                                  virtual_rc_data[3] = 1024- yaw; //3-> yaw       [1024-660,1024+660]
                                                  virtual_rc_data[4] = 1684;      //4-> gear {1684(UP), 1324(DOWN)}
                                                  virtual_rc_data[6] = 1552;      //6-> mode     {1552(P), 1024(A), 496(F)}

                                                     for (int i = 0; i < 10; i++)
                                                          {
                                                           drone->virtual_rc_control(virtual_rc_data);
                                                           usleep(20000);
                                                          
                                                          }
                                                 }
                                          else if (key1 == 'l')
                                                   {
                                                    //Yaw right
                                                    cout<< "Drone yawing right... " <<endl;
                                                    drone->virtual_rc_enable();
                                                    usleep(20000);
                                                    virtual_rc_data[0] = 1024;      //0-> roll     [1024-660,1024+660]
                                                    virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
                                                    virtual_rc_data[2] = 1024;      //2-> throttle [1024-660,1024+660]
                                                    virtual_rc_data[3] = 1024+ yaw; //3-> yaw       [1024-660,1024+660]
                                                    virtual_rc_data[4] = 1684;      //4-> gear {1684(UP), 1324(DOWN)}
                                                    virtual_rc_data[6] = 1552;      //6-> mode     {1552(P), 1024(A), 496(F)}

                                                       for (int i = 0; i < 10; i++)
                                                            {
                                                             drone->virtual_rc_control(virtual_rc_data);
                                                             usleep(20000);
                                                             
                                                            }
                                                  }

                                             }

                                       }

                                   }
                          
				drone->virtual_rc_disable();
				drone->landing();
				break;
                                cout<< "End of Control." <<endl;
			}
                                


//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------                                

			case 25:
				{
                                   double throttle = 100;
                                   double roll = 75;
                                   double yaw = 100; 
                                   double pitch = 75;
                                   double frontHighest = 2.8; //STOP ROLLING WHEN THERE'S CLEARANCE AND YAW RIGHT
                                   double frontUpper = 2;   //IF FRONT MORE THAN 2 METERS PITCH FORWARD
                                   double frontLower = 0.8; //IF FRONT LESS THAN 0.8 METERS STOP PITCHING FORWARD
                                   double rightHighest = 1.4; 
                                   double rightUpper = 1.2;
                                   double rightLower = 0.8;
                                   char killkey; //Variable for killing the code and return to manual flight. 
                                   //Automatic Throttle Up
                                    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "------------------WALL TRACING SELECTED----------------" <<endl;
                                    cout<< "Drone taking off..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024+ throttle; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey =getche();
                                                  if(killkey == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                         }
                                    
                                    cout<< "End of drone takeoff." <<endl;

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey =getche();
                                                  if(killkey == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover." <<endl;
                                    cout<< "------------------Start WALL TRACING----------------" <<endl;

                                   
                                    for ( ;; )

                                        {
                                         ifstream FileF("Front.txt");
			                 ifstream FileR("Right.txt");
			                 ifstream FileL("Left.txt");
			                 sleep(1);
			                 double Front;
			                 double Right;
			                 FileF >> Front;
			                 FileR >> Right;
                                         cout<< Front;
                                         cout<< Right;
                                         
                                        //...............................................................................................................................
                                           if (Front > frontUpper)
		                                
                                              {
			                       //Pitch Forward
                                               cout<< "*Empty space at the Front detected*" <<endl;
                                               cout<< "Drone Pitching Forward..." <<endl; 
			                       drone->virtual_rc_enable();
			                       usleep(20000);
			                       virtual_rc_data[0] = 1024;		//0-> roll     	[1024-660,1024+660] 
			                       virtual_rc_data[1] = 1024+ pitch;	//1-> pitch    	[1024-660,1024+660]
			                       virtual_rc_data[2] = 1024;	        //2-> throttle 	[1024-660,1024+660]
			                       virtual_rc_data[3] = 1024;		//3-> yaw      	[1024-660,1024+660]
			                       virtual_rc_data[4] = 1324;	 	//4-> gear	{1684(UP), 1324(DOWN)}
			                       virtual_rc_data[6] = 1552;    	        //6-> mode     	{1552(P), 1024(A), 496(F)
			                       cout << "Pitch forward ";

				                for(;;)
				                  {
				                   double Front;
				                   drone->virtual_rc_control(virtual_rc_data);
		  		                   usleep(20000);
				                   ifstream FileF("Front.txt");
			                           ifstream FileR("Right.txt");
				                   FileF >> Front;
                                                   //cout<< Front;
                                                   //cout<< Right;
				                   if (Front < frontLower )
                                                      {
                                                       //Automatic hover
				                       drone->virtual_rc_enable();
                                                       usleep(2000);
                                                       cout<< "Drone stopped Pitching Forward." <<endl;
                                                       cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                       virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                       virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                       virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                       virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                       virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                       virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                       for (int i = 0; i < 200; i++)
                                                           {
                                                            drone->virtual_rc_control(virtual_rc_data);
                                                            usleep(23000);
                                                            while (kbhit())
                                                                   {
                                                                    killkey =getche();
                                                                    if(killkey == 'q')
                                                                      {
                                                                       return 0;
                                                                      }
                                                                   }	
				                           }
                                                           cout<< "End of drone hover." <<endl;
                                                           break; //exit for(;;)
                                                      }
				                    }
		                               
                                                 }
                                        //......................................................................................................................
                                           FileF >> Front;
			                   FileR >> Right;
                                           cout<< Front;
                                           cout<< Right;
                                           if ( Right > rightUpper )
		                                  
                                               {
				                     //Roll Right
                                                     cout<< "*Empty space on the Right detected*" <<endl;
                                                     cout<< "Drone Rolling right..." <<endl;
				                     drone->virtual_rc_enable();
				                     usleep(20000);
				                     virtual_rc_data[0] = 1024+ roll;   //0-> roll     	[1024-660,1024+660] 
				                     virtual_rc_data[1] = 1024;		//1-> pitch    	[1024-660,1024+660]
				                     virtual_rc_data[2] = 1024;	        //2-> throttle 	[1024-660,1024+660]
				                     virtual_rc_data[3] = 1024;		//3-> yaw      	[1024-660,1024+660]
				                     virtual_rc_data[4] = 1324;	 	//4-> gear	{1684(UP), 1324(DOWN)}
				                     virtual_rc_data[6] = 1552;    	//6-> mode     	{1552(P), 1024(A), 496(F)
                                                      for(;;)
				                         {
				                          double Front;
				                          drone->virtual_rc_control(virtual_rc_data);
		  		                          usleep(20000);
				                          ifstream FileF("Front.txt");
			                                  ifstream FileR("Right.txt");
				                          FileR >> Right;
                                                          //cout<< Front;
                                                          //cout<< Right;
				                          if (Right < rightLower )
                                                             {
                                                              //Automatic hover
				                              drone->virtual_rc_enable();
                                                              usleep(2000);
                                                              cout<< "Drone stopped Rolling right." <<endl;
                                                              cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                              virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                              virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                              virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                              virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                              virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                              virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                              for (int i = 0; i < 200; i++)
                                                                  {
                                                                   drone->virtual_rc_control(virtual_rc_data);
                                                                   usleep(23000);
                                                                   while (kbhit())
                                                                          {
                                                                           killkey =getche();
                                                                           if(killkey == 'q')
                                                                              {
                                                                               return 0;
                                                                              }
                                                                          }
                                                                  }
                                                               cout<< "End of drone hover." <<endl;
                                                               break; //Exit for(;;)	 
				                                  
                                                              } 
				                           }
                                                  }
                                           //......................................................................................................................
                                           if (Front < 0.5)
		                                
                                              {
			                       //Pitch Forward
                                               cout<< "*Empty space at the Front detected*" <<endl;
                                               cout<< "Drone Pitching Forward..." <<endl; 
			                       drone->virtual_rc_enable();
			                       usleep(20000);
			                       virtual_rc_data[0] = 1024;		//0-> roll     	[1024-660,1024+660] 
			                       virtual_rc_data[1] = 1024+ pitch;	//1-> pitch    	[1024-660,1024+660]
			                       virtual_rc_data[2] = 1024;	        //2-> throttle 	[1024-660,1024+660]
			                       virtual_rc_data[3] = 1024;		//3-> yaw      	[1024-660,1024+660]
			                       virtual_rc_data[4] = 1324;	 	//4-> gear	{1684(UP), 1324(DOWN)}
			                       virtual_rc_data[6] = 1552;    	        //6-> mode     	{1552(P), 1024(A), 496(F)
			                       cout << "Pitch forward ";

				                for(;;)
				                  {
				                   double Front;
				                   drone->virtual_rc_control(virtual_rc_data);
		  		                   usleep(20000);
				                   ifstream FileF("Front.txt");
			                           ifstream FileR("Right.txt");
				                   FileF >> Front;
                                                   //cout<< Front;
                                                   //cout<< Right;
				                   if (Front < frontLower )
                                                      {
                                                       //Automatic hover
				                       drone->virtual_rc_enable();
                                                       usleep(2000);
                                                       cout<< "Drone stopped Pitching Forward." <<endl;
                                                       cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                       virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                       virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                       virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                       virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                       virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                       virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                       for (int i = 0; i < 200; i++)
                                                           {
                                                            drone->virtual_rc_control(virtual_rc_data);
                                                            usleep(23000);
                                                            while (kbhit())
                                                                   {
                                                                    killkey =getche();
                                                                    if(killkey == 'q')
                                                                      {
                                                                       return 0;
                                                                      }
                                                                   }	
				                           }
                                                           cout<< "End of drone hover." <<endl;
                                                           break; //exit for(;;)
                                                      }
				                    }
		                               
                                                 }
                                        //......................................................................................................................
                                           FileF >> Front;
			                   FileR >> Right;
                                           cout<< Front;
                                           cout<< Right;
                                           if ( Right < 0.5 )
		                                  
                                               {
				                     //Roll Right
                                                     cout<< "*Empty space on the Right detected*" <<endl;
                                                     cout<< "Drone Rolling right..." <<endl;
				                     drone->virtual_rc_enable();
				                     usleep(20000);
				                     virtual_rc_data[0] = 1024+ roll;   //0-> roll     	[1024-660,1024+660] 
				                     virtual_rc_data[1] = 1024;		//1-> pitch    	[1024-660,1024+660]
				                     virtual_rc_data[2] = 1024;	        //2-> throttle 	[1024-660,1024+660]
				                     virtual_rc_data[3] = 1024;		//3-> yaw      	[1024-660,1024+660]
				                     virtual_rc_data[4] = 1324;	 	//4-> gear	{1684(UP), 1324(DOWN)}
				                     virtual_rc_data[6] = 1552;    	//6-> mode     	{1552(P), 1024(A), 496(F)
                                                      for(;;)
				                         {
				                          double Front;
				                          drone->virtual_rc_control(virtual_rc_data);
		  		                          usleep(20000);
				                          ifstream FileF("Front.txt");
			                                  ifstream FileR("Right.txt");
				                          FileR >> Right;
                                                          //cout<< Front;
                                                          //cout<< Right;
				                          if (Right < rightLower )
                                                             {
                                                              //Automatic hover
				                              drone->virtual_rc_enable();
                                                              usleep(2000);
                                                              cout<< "Drone stopped Rolling right." <<endl;
                                                              cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                              virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                              virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                              virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                              virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                              virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                              virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                              for (int i = 0; i < 200; i++)
                                                                  {
                                                                   drone->virtual_rc_control(virtual_rc_data);
                                                                   usleep(23000);
                                                                   while (kbhit())
                                                                          {
                                                                           killkey =getche();
                                                                           if(killkey == 'q')
                                                                              {
                                                                               return 0;
                                                                              }
                                                                          }
                                                                  }
                                                               cout<< "End of drone hover." <<endl;
                                                               break; //Exit for(;;)	 
				                                  
                                                              } 
				                           }
                                                  }
                                           //......................................................................................................................
                                              FileF >> Front;
			                      FileR >> Right;
                                              //cout<< Front;
                                              //cout<< Right;
                                              if ( Right < rightLower && Front < frontLower  )
		                                  
                                                     {
				                      //Yaw Right
                                                      cout<< "*Inwards bend detected*" <<endl;
                                                      cout<< "Drone Yawing right..." <<endl;
				                      drone->virtual_rc_enable();
				                      usleep(20000);
				                      virtual_rc_data[0] = 1024;       //0-> roll     	[1024-660,1024+660] 
				                      virtual_rc_data[1] = 1024;       //1-> pitch    	[1024-660,1024+660]
				                      virtual_rc_data[2] = 1024;       //2-> throttle 	[1024-660,1024+660]
				                      virtual_rc_data[3] = 1024+ yaw;  //3-> yaw      	[1024-660,1024+660]
				                      virtual_rc_data[4] = 1324;       //4-> gear	{1684(UP), 1324(DOWN)}
				                      virtual_rc_data[6] = 1552;       //6-> mode     	{1552(P), 1024(A), 496(F)

                                                      for (int i = 0; i < 450; i++)
                                                           {
                                                            drone->virtual_rc_control(virtual_rc_data);
                                                            usleep(20000);
                                                            while (kbhit())
                                                                  {
                                                                   killkey =getche();
                                                                   if(killkey == 'q')
                                                                     {
                                                                      return 0;
                                                                     }
                                                                  }
	                                 
				                             }
                                                        cout<< "Drone finish Yawing Right." <<endl;
                                                      }

                                                      //Automatic hover
				                      drone->virtual_rc_enable();
                                                      usleep(2000);
                                                      cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                      virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                      virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                      virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                      virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                      virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                      virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                      for (int i = 0; i < 200; i++)
                                                          {
                                                           drone->virtual_rc_control(virtual_rc_data);
                                                           usleep(23000);
                                                           while (kbhit())
                                                                  {
                                                                   killkey =getche();
                                                                   if(killkey == 'q')
                                                                     {
                                                                      return 0;
                                                                     }
                                                                  }	
				                          }
                                                      cout<< "End of drone hover." <<endl;
                                              //......................................................................................................................
                                                 FileF >> Front;
			                         FileR >> Right;
                                                 //cout<< Front;
                                                 //cout<< Right;
                                                 if ( Right > rightUpper )
		                                   
                                                          {
                                                           cout<< "*Empty space on the Right detected*" <<endl;
                                                           cout<< "Drone Rolling right..." <<endl;
				                           //Roll Right
				                           drone->virtual_rc_enable();
				                           usleep(20000);
				                           virtual_rc_data[0] = 1024+ roll; //0-> roll     	[1024-660,1024+660] 
				                           virtual_rc_data[1] = 1024;	    //1-> pitch    	[1024-660,1024+660]
				                           virtual_rc_data[2] = 1024;	    //2-> throttle 	[1024-660,1024+660]
				                           virtual_rc_data[3] = 1024;       //3-> yaw      	[1024-660,1024+660]
				                           virtual_rc_data[4] = 1324;	    //4-> gear	        {1684(UP), 1324(DOWN)}
				                           virtual_rc_data[6] = 1552;       //6-> mode     	{1552(P), 1024(A), 496(F)
                                                           for(;;)
				                                {
				                                 double Front;
                                                                 double Right;
				                                 drone->virtual_rc_control(virtual_rc_data);
		  		                                 usleep(20000);
				                                 ifstream FileF("Front.txt");
			                                         ifstream FileR("Right.txt");
				                                 FileF >> Front;
                                                                 FileR >> Right;
                                                                 //cout<< Front;
                                                                 //cout<< Right;
				                                 if (Front > frontHighest )
                                                                    {
                                                                     //Automatic hover
				                                     drone->virtual_rc_enable();
                                                                     usleep(2000);
                                                                     cout<< "Drone stopped Rolling right." <<endl;
                                                                     cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                                     virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                                     virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                                     virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                                     virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                                     virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                                     virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                                  for (int i = 0; i < 200; i++)
                                                                      {
                                                                       drone->virtual_rc_control(virtual_rc_data);
                                                                       usleep(23000);
                                                                       while (kbhit())
                                                                          {
                                                                           killkey =getche();
                                                                           if(killkey == 'q')
                                                                             {
                                                                              return 0;
                                                                             }
                                                                          }
                                                                      }
                                                                     cout<< "End of drone hover." <<endl;
                                                                     break; //Exit for(;;)	 
				                                  
                                                                    } 
				                                  }
                                                                }

                                                  //......................................................................................................................
                                                     FileF >> Front;
			                             FileR >> Right;
                                                     //cout<< Front;
                                                     //cout<< Right;
                                                     if ( Right > rightLower && Front > frontLower  )
		                                  
                                                          {
                                                           cout<< "*Outwards bend detected*" <<endl;
                                                           cout<< "Drone Yawing Left..." <<endl;
				                           //Yaw Left
				                           drone->virtual_rc_enable();
				                           usleep(20000);
				                           virtual_rc_data[0] = 1024;      //0-> roll     	[1024-660,1024+660] 
				                           virtual_rc_data[1] = 1024;      //1-> pitch    	[1024-660,1024+660]
				                           virtual_rc_data[2] = 1024;	   //2-> throttle 	[1024-660,1024+660]
				                           virtual_rc_data[3] = 1024- yaw; //3-> yaw      	[1024-660,1024+660]
				                           virtual_rc_data[4] = 1324;	   //4-> gear	{1684(UP), 1324(DOWN)}
				                           virtual_rc_data[6] = 1552;      //6-> mode     	{1552(P), 1024(A), 496(F)
                                                           for (int i = 0; i < 450; i++)
                                                               {
                                                                drone->virtual_rc_control(virtual_rc_data);
                                                                usleep(20000);
                                                                while (kbhit())
                                                                     {
                                                                      killkey =getche();
                                                                      if(killkey == 'q')
                                                                         {
                                                                          return 0;
                                                                         }
                                                                     }
	                                 
				                               }  
                                                          cout<< "Drone finish Yawing Left." <<endl; 

                                                         }

                                                   //......................................................................................................................
                                                      FileF >> Front;
			                              FileR >> Right;
                                                      //cout<< Front;
                                                      //cout<< Right;
                                                      if ( Right > rightUpper )
		                                  
                                                       { 
                                                        cout<< "*Empty space on the Right detected*" <<endl;
                                                        cout<< "Drone Rolling right..." <<endl;
				                        //Roll Right
				                        drone->virtual_rc_enable();
				                        usleep(20000);
				                        virtual_rc_data[0] = 1024+ roll;      //0-> roll     	[1024-660,1024+660] 
				                        virtual_rc_data[1] = 1024;	      //1-> pitch    	[1024-660,1024+660]
				                        virtual_rc_data[2] = 1024;	      //2-> throttle 	[1024-660,1024+660]
				                        virtual_rc_data[3] = 1024;            //3-> yaw      	[1024-660,1024+660]
				                        virtual_rc_data[4] = 1324;	      //4-> gear	{1684(UP), 1324(DOWN)}
				                        virtual_rc_data[6] = 1552;    	      //6-> mode     	{1552(P), 1024(A), 496(F)
                                                        for(;;)
				                           {
				                           double Front;
				                           drone->virtual_rc_control(virtual_rc_data);
		  		                           usleep(20000);
				                           ifstream FileF("Front.txt");
			                                   ifstream FileR("Right.txt");
				                           FileR >> Right;
                                                           //cout<< Front;
                                                           //cout<< Right;
				                           if (Right < rightLower )
                                                              {
                                                               //Automatic hover
				                               drone->virtual_rc_enable();
                                                               usleep(2000);
                                                               cout<< "Drone stopped Rolling right..." <<endl;
                                                               cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                               virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                               virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                               virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                               virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                               virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                               virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                               for (int i = 0; i < 200; i++)
                                                                   {
                                                                    drone->virtual_rc_control(virtual_rc_data);
                                                                    usleep(23000);
                                                                    while (kbhit())
                                                                          {
                                                                           killkey =getche();
                                                                           if(killkey == 'q')
                                                                              {
                                                                               return 0;
                                                                              }
                                                                          }
                                                                   }
                                                               cout<< "End of drone hover..." <<endl;
                                                               break; //Exit for(;;)	 
				                              }
                                                            }
				                          }

                                            }//End of Wall Tracing for(;;)  
                   
		
		                 }
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------                                

			case 26:
				 {
                                   double throttle = 100;
                                   double roll = 75;
                                   double yaw = 100; 
                                   double pitch = 75;
                                   double frontUpper = 2; //IF FRONT MORE THAN 2.5 METERS PITCH FORWARD
                                   double frontLower = 1; //IF FRONT LESS THAN 1 METERS STOP PITCHING FORWARD
                                   double rightUpper = 1;
                                   double rightLower = 0.4;
                                   char killkey; //Variable for killing the code and return to manual flight. 
                                   //Automatic Throttle Up
                                    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "------------------WALL TRACING SELECTED (Inward-Tracing)----------------" <<endl;
                                    cout<< "Drone taking off..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024+ throttle; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey =getche();
                                                  if(killkey == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                         }
                                    
                                    cout<< "End of drone takeoff." <<endl;

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey =getche();
                                                  if(killkey == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover." <<endl;
                                    cout<< "------------------Start WALL TRACING----------------" <<endl;

                                    for ( ;; )

                                        {
                                         ifstream FileF("Front.txt");
			                 ifstream FileR("Right.txt");
			                 ifstream FileL("Left.txt");
			                 sleep(1);
			                 double Front;
			                 double Right;
			                 FileF >> Front;
			                 FileR >> Right;
                                         cout<< Front;
                                         cout<< Right;
                                         
                                        //...............................................................................................................................
                                           if (Front > frontUpper)
		                                
                                              {
			                       //Pitch Forward
                                               cout<< "*Empty space at the Front detected*" <<endl;
                                               cout<< "Drone Pitching Forward..." <<endl; 
			                       drone->virtual_rc_enable();
			                       usleep(20000);
			                       virtual_rc_data[0] = 1024;		//0-> roll     	[1024-660,1024+660] 
			                       virtual_rc_data[1] = 1024+ pitch;	//1-> pitch    	[1024-660,1024+660]
			                       virtual_rc_data[2] = 1024;	        //2-> throttle 	[1024-660,1024+660]
			                       virtual_rc_data[3] = 1024;		//3-> yaw      	[1024-660,1024+660]
			                       virtual_rc_data[4] = 1324;	 	//4-> gear	{1684(UP), 1324(DOWN)}
			                       virtual_rc_data[6] = 1552;    	        //6-> mode     	{1552(P), 1024(A), 496(F)
			                       cout << "Pitch forward ";

				                for(;;)
				                  {
				                   double Front;
				                   drone->virtual_rc_control(virtual_rc_data);
		  		                   usleep(20000);
				                   ifstream FileF("Front.txt");
			                           ifstream FileR("Right.txt");
				                   FileF >> Front;
                                                   //cout<< Front;
                                                   //cout<< Right;
				                   if (Front < frontLower )
                                                      {
                                                       //Automatic hover
				                       drone->virtual_rc_enable();
                                                       usleep(2000);
                                                       cout<< "Drone stopped Pitching Forward." <<endl;
                                                       cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                       virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                       virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                       virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                       virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                       virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                       virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                       for (int i = 0; i < 200; i++)
                                                           {
                                                            drone->virtual_rc_control(virtual_rc_data);
                                                            usleep(23000);
                                                            while (kbhit())
                                                                   {
                                                                    killkey =getche();
                                                                    if(killkey == 'q')
                                                                      {
                                                                       return 0;
                                                                      }
                                                                   }	
				                           }
                                                           cout<< "End of drone hover." <<endl;
                                                           break; //exit for(;;)
                                                      }
				                    }
		                               
                                                 }
                                        //......................................................................................................................
                                           FileF >> Front;
			                   FileR >> Right;
                                           cout<< Front;
                                           cout<< Right;
                                           if ( Right > rightUpper )
		                                  
                                               {
				                     //Roll Right
                                                     cout<< "*Empty space on the Right detected*" <<endl;
                                                     cout<< "Drone Rolling right..." <<endl;
				                     drone->virtual_rc_enable();
				                     usleep(20000);
				                     virtual_rc_data[0] = 1024+ roll;   //0-> roll     	[1024-660,1024+660] 
				                     virtual_rc_data[1] = 1024;		//1-> pitch    	[1024-660,1024+660]
				                     virtual_rc_data[2] = 1024;	        //2-> throttle 	[1024-660,1024+660]
				                     virtual_rc_data[3] = 1024;		//3-> yaw      	[1024-660,1024+660]
				                     virtual_rc_data[4] = 1324;	 	//4-> gear	{1684(UP), 1324(DOWN)}
				                     virtual_rc_data[6] = 1552;    	//6-> mode     	{1552(P), 1024(A), 496(F)
                                                      for(;;)
				                         {
				                          double Front;
				                          drone->virtual_rc_control(virtual_rc_data);
		  		                          usleep(20000);
				                          ifstream FileF("Front.txt");
			                                  ifstream FileR("Right.txt");
				                          FileR >> Right;
                                                          //cout<< Front;
                                                          //cout<< Right;
				                          if (Right < rightLower )
                                                             {
                                                              //Automatic hover
				                              drone->virtual_rc_enable();
                                                              usleep(2000);
                                                              cout<< "Drone stopped Rolling right." <<endl;
                                                              cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                              virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                              virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                              virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                              virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                              virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                              virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                              for (int i = 0; i < 200; i++)
                                                                  {
                                                                   drone->virtual_rc_control(virtual_rc_data);
                                                                   usleep(23000);
                                                                   while (kbhit())
                                                                          {
                                                                           killkey =getche();
                                                                           if(killkey == 'q')
                                                                              {
                                                                               return 0;
                                                                              }
                                                                          }
                                                                  }
                                                               cout<< "End of drone hover." <<endl;
                                                               break; //Exit for(;;)	 
				                                  
                                                              } 
				                           }
                                                  }

                                            //......................................................................................................................
                                              FileF >> Front;
			                      FileR >> Right;
                                              //cout<< Front;
                                              //cout<< Right;
                                              if ( Right < rightLower && Front < frontLower  )
		                                  
                                                     {
				                      //Yaw Right
                                                      cout<< "*Inwards bend detected*" <<endl;
                                                      cout<< "Drone Yawing right..." <<endl;
				                      drone->virtual_rc_enable();
				                      usleep(20000);
				                      virtual_rc_data[0] = 1024;       //0-> roll     	[1024-660,1024+660] 
				                      virtual_rc_data[1] = 1024;       //1-> pitch    	[1024-660,1024+660]
				                      virtual_rc_data[2] = 1024;       //2-> throttle 	[1024-660,1024+660]
				                      virtual_rc_data[3] = 1024+ yaw;  //3-> yaw      	[1024-660,1024+660]
				                      virtual_rc_data[4] = 1324;       //4-> gear	{1684(UP), 1324(DOWN)}
				                      virtual_rc_data[6] = 1552;       //6-> mode     	{1552(P), 1024(A), 496(F)

                                                      for (int i = 0; i < 430; i++)
                                                           {
                                                            drone->virtual_rc_control(virtual_rc_data);
                                                            usleep(20000);
                                                            while (kbhit())
                                                                  {
                                                                   killkey =getche();
                                                                   if(killkey == 'q')
                                                                     {
                                                                      return 0;
                                                                     }
                                                                  }
	                                 
				                             }
                                                        cout<< "Drone finish Yawing Right." <<endl;
                                                      }

                                                      //Automatic hover
				                      drone->virtual_rc_enable();
                                                      usleep(2000);
                                                      cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                      virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                      virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                      virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                      virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                      virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                      virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                      for (int i = 0; i < 200; i++)
                                                          {
                                                           drone->virtual_rc_control(virtual_rc_data);
                                                           usleep(23000);
                                                           while (kbhit())
                                                                  {
                                                                   killkey =getche();
                                                                   if(killkey == 'q')
                                                                     {
                                                                      return 0;
                                                                     }
                                                                  }	
				                          }
                                                      cout<< "End of drone hover." <<endl;
                                        }//End of Wall Tracing for(;;)  
                   
		
		                 }
			case 27:
				{
                                   double throttle = 100;
                                   double roll = 75;
                                   double yaw = 100; 
                                   double pitch = 75;
                                   double frontHighest = 2.8; //STOP ROLLING WHEN THERE'S CLEARANCE AND YAW RIGHT
                                   double frontUpper = 2;   //IF FRONT MORE THAN 2 METERS PITCH FORWARD
                                   double frontLower = 0.8; //IF FRONT LESS THAN 0.8 METERS STOP PITCHING FORWARD
                                   double rightHighest = 1.4; 
                                   double rightUpper = 1.2;
                                   double rightLower = 0.8;
                                   char killkey; //Variable for killing the code and return to manual flight. 
                                   //Automatic Throttle Up
                                    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "------------------WALL TRACING SELECTED (Inward-Tracing)----------------" <<endl;
                                    cout<< "Drone taking off..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024+ throttle; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(20000);
                                          while (kbhit())
                                                 {
                                                  killkey =getche();
                                                  if(killkey == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                         }
                                    
                                    cout<< "End of drone takeoff." <<endl;

                                    //Automatic hover
				    drone->virtual_rc_enable();
                                    usleep(2000);
                                    cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                    virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                    for (int i = 0; i < 200; i++)
                                         {
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(23000);
                                          while (kbhit())
                                                 {
                                                  killkey =getche();
                                                  if(killkey == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }	
				         }
                                    cout<< "End of drone hover." <<endl;
                                    cout<< "------------------Start WALL TRACING----------------" <<endl;
                                    ifstream FileF("Front.txt");
			                 ifstream FileR("Right.txt");
			                 ifstream FileL("Left.txt");
			                 sleep(1);
			                 double Front;
			                 double Right;
			                 FileF >> Front;
			                 FileR >> Right;
                                         cout<< Front;
                                         cout<< Right;
                                         
                                        //...............................................................................................................................
                                           if (Front > frontUpper)
		                                
                                              {
			                       //Pitch Forward
                                               cout<< "*Empty space at the Front detected*" <<endl;
                                               cout<< "Drone Pitching Forward..." <<endl; 
			                       drone->virtual_rc_enable();
			                       usleep(20000);
			                       virtual_rc_data[0] = 1024;		//0-> roll     	[1024-660,1024+660] 
			                       virtual_rc_data[1] = 1024+ pitch;	//1-> pitch    	[1024-660,1024+660]
			                       virtual_rc_data[2] = 1024;	        //2-> throttle 	[1024-660,1024+660]
			                       virtual_rc_data[3] = 1024;		//3-> yaw      	[1024-660,1024+660]
			                       virtual_rc_data[4] = 1324;	 	//4-> gear	{1684(UP), 1324(DOWN)}
			                       virtual_rc_data[6] = 1552;    	        //6-> mode     	{1552(P), 1024(A), 496(F)
			                       cout << "Pitch forward ";

				                for(;;)
				                  {
				                   double Front;
				                   drone->virtual_rc_control(virtual_rc_data);
		  		                   usleep(20000);
				                   ifstream FileF("Front.txt");
			                           ifstream FileR("Right.txt");
				                   FileF >> Front;
                                                   //cout<< Front;
                                                   //cout<< Right;
				                   if (Front < frontLower )
                                                      {
                                                       //Automatic hover
				                       drone->virtual_rc_enable();
                                                       usleep(2000);
                                                       cout<< "Drone stopped Pitching Forward." <<endl;
                                                       cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                       virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                       virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                       virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                       virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                       virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                       virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                       for (int i = 0; i < 200; i++)
                                                           {
                                                            drone->virtual_rc_control(virtual_rc_data);
                                                            usleep(23000);
                                                            while (kbhit())
                                                                   {
                                                                    killkey =getche();
                                                                    if(killkey == 'q')
                                                                      {
                                                                       return 0;
                                                                      }
                                                                   }	
				                           }
                                                           cout<< "End of drone hover." <<endl;
                                                           break; //exit for(;;)
                                                      }
				                    }
		                               
                                                 }
                                            //...............................................................................................................................

                                               for ( ;; )
                                                  {
                                                   FileF >> Front;
			                           FileR >> Right;
                                                   //cout<< Front;
                                                   //cout<< Right;
                                                   if ( Right > rightUpper )
		                                   
                                                          {
                                                           cout<< "*Empty space on the Right detected*" <<endl;
                                                           cout<< "Drone Rolling right..." <<endl;
				                           //Roll Right
				                           drone->virtual_rc_enable();
				                           usleep(20000);
				                           virtual_rc_data[0] = 1024+ roll; //0-> roll     	[1024-660,1024+660] 
				                           virtual_rc_data[1] = 1024;	    //1-> pitch    	[1024-660,1024+660]
				                           virtual_rc_data[2] = 1024;	    //2-> throttle 	[1024-660,1024+660]
				                           virtual_rc_data[3] = 1024;       //3-> yaw      	[1024-660,1024+660]
				                           virtual_rc_data[4] = 1324;	    //4-> gear	        {1684(UP), 1324(DOWN)}
				                           virtual_rc_data[6] = 1552;       //6-> mode     	{1552(P), 1024(A), 496(F)
                                                           for(;;)
				                                {
				                                 double Front;
                                                                 double Right;
				                                 drone->virtual_rc_control(virtual_rc_data);
		  		                                 usleep(20000);
				                                 ifstream FileF("Front.txt");
			                                         ifstream FileR("Right.txt");
				                                 FileF >> Front;
                                                                 FileR >> Right;
                                                                 //cout<< Front;
                                                                 //cout<< Right;
				                                 if (Front > frontHighest )
                                                                    {
                                                                     //Automatic hover
				                                     drone->virtual_rc_enable();
                                                                     usleep(2000);
                                                                     cout<< "Drone stopped Rolling right." <<endl;
                                                                     cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                                     virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                                     virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                                     virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                                     virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                                     virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                                     virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                                  for (int i = 0; i < 200; i++)
                                                                      {
                                                                       drone->virtual_rc_control(virtual_rc_data);
                                                                       usleep(23000);
                                                                       while (kbhit())
                                                                          {
                                                                           killkey =getche();
                                                                           if(killkey == 'q')
                                                                             {
                                                                              return 0;
                                                                             }
                                                                          }
                                                                      }
                                                                     cout<< "End of drone hover." <<endl;
                                                                     break; //Exit for(;;)	 
				                                  
                                                                    } 
				                                  }
                                                                }

                                                  //......................................................................................................................
                                                     FileF >> Front;
			                             FileR >> Right;
                                                     //cout<< Front;
                                                     //cout<< Right;
                                                     if ( Right > rightLower && Front > frontLower  )
		                                  
                                                          {
                                                           cout<< "*Outwards bend detected*" <<endl;
                                                           cout<< "Drone Yawing Left..." <<endl;
				                           //Yaw Left
				                           drone->virtual_rc_enable();
				                           usleep(20000);
				                           virtual_rc_data[0] = 1024;      //0-> roll     	[1024-660,1024+660] 
				                           virtual_rc_data[1] = 1024;      //1-> pitch    	[1024-660,1024+660]
				                           virtual_rc_data[2] = 1024;	   //2-> throttle 	[1024-660,1024+660]
				                           virtual_rc_data[3] = 1024- yaw; //3-> yaw      	[1024-660,1024+660]
				                           virtual_rc_data[4] = 1324;	   //4-> gear	{1684(UP), 1324(DOWN)}
				                           virtual_rc_data[6] = 1552;      //6-> mode     	{1552(P), 1024(A), 496(F)
                                                           for (int i = 0; i < 450; i++)
                                                               {
                                                                drone->virtual_rc_control(virtual_rc_data);
                                                                usleep(20000);
                                                                while (kbhit())
                                                                     {
                                                                      killkey =getche();
                                                                      if(killkey == 'q')
                                                                         {
                                                                          return 0;
                                                                         }
                                                                     }
	                                 
				                               }  
                                                          cout<< "Drone finish Yawing Left." <<endl; 
                                                         }
                                                    }//End of Wall Tracing for(;;)  
		                                }

                                                       
                                    
                               
			case 28:
				int i, a, Rightfile;
		                double q;
		                char killkey; //Variable for killing the code and return to manual flight. 
		                 {
		                  //Throttle Up 
		                  drone->virtual_rc_enable();
		                  usleep(20000);
		                  virtual_rc_data[0] = 1024;	//0-> roll     	[1024-660,1024+660] 
		                  virtual_rc_data[1] = 1024;	//1-> pitch    	[1024-660,1024+660]
		                  virtual_rc_data[2] = 1024+100;	//2-> throttle 	[1024-660,1024+660]
		                  virtual_rc_data[3] = 1024;	//3-> yaw      	[1024-660,1024+660]
		                  virtual_rc_data[4] = 1684;	//4-> gear      {1684(UP), 1324(DOWN)}
		                  virtual_rc_data[6] = 1552;      //6-> mode     	{1552(P), 1024(A), 496(F)}

		for (int i = 0; i < 150; i++)
		{
			drone->virtual_rc_control(virtual_rc_data);
			usleep(20000);
		}
                //Automatic hover
				                       drone->virtual_rc_enable();
                                                       usleep(2000);
                                                       cout<< "Drone stopped Pitching Forward." <<endl;
                                                       cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                       virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                       virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                       virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                       virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                       virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                       virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                       for (int i = 0; i < 200; i++)
                                                           {
                                                            drone->virtual_rc_control(virtual_rc_data);
                                                            usleep(23000);
                                                            while (kbhit())
                                                                   {
                                                                    killkey =getche();
                                                                    if(killkey == 'q')
                                                                      {
                                                                       return 0;
                                                                      }
                                                                   }	
				                           }
		for ( ;; )
		{
			ifstream FileF("Front.txt");
			ifstream FileR("Right.txt");
			ifstream FileL("Left.txt");
			sleep(1);
			double Front;
			double Right;
			double Left;
			double Stop;
			FileF >> Front;
			FileR >> Right;
			
		if ( Front > 1.1)
		{
	 
			
			//Pitch Forward 
			drone->virtual_rc_enable();
			usleep(20000);
			virtual_rc_data[0] = 1024;		//0-> roll     	[1024-660,1024+660] 
			virtual_rc_data[1] = 1024+75;		//1-> pitch    	[1024-660,1024+660]
			virtual_rc_data[2] = 1024;	        //2-> throttle 	[1024-660,1024+660]
			virtual_rc_data[3] = 1024;		//3-> yaw      	[1024-660,1024+660]
			virtual_rc_data[4] = 1324;	 	//4-> gear		{1684(UP), 1324(DOWN)}
			virtual_rc_data[6] = 1552;    	        //6-> mode     	{1552(P), 1024(A), 496(F)
			cout << "Pitch forward ";

				do
				{
				double Front;
				drone->virtual_rc_control(virtual_rc_data);
		  		usleep(20000);
				ifstream FileF("Front.txt");
				FileF >> Front;
				if (Front < 1.1 ) break; 
				}while (1);
		}
                //Automatic hover
				                       drone->virtual_rc_enable();
                                                       usleep(2000);
                                                       cout<< "Drone stopped Pitching Forward." <<endl;
                                                       cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                       virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                       virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                       virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                       virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                       virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                       virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                       for (int i = 0; i < 200; i++)
                                                           {
                                                            drone->virtual_rc_control(virtual_rc_data);
                                                            usleep(23000);
                                                            while (kbhit())
                                                                   {
                                                                    killkey =getche();
                                                                    if(killkey == 'q')
                                                                      {
                                                                       return 0;
                                                                      }
                                                                   }	
				                           }
		
		if ( Right > 1.1)
		{

				do
				{
				//Roll Right
				for (int j = 0; j < 50; j++)
				{
				drone->virtual_rc_enable();
				usleep(20000);
				virtual_rc_data[0] = 1024+75;		//0-> roll     	[1024-660,1024+660] 
				virtual_rc_data[1] = 1024;		//1-> pitch    	[1024-660,1024+660]
				virtual_rc_data[2] = 1024;              //2-> throttle 	[1024-660,1024+660]
				virtual_rc_data[3] = 1024;		//3-> yaw      	[1024-660,1024+660]
				virtual_rc_data[4] = 1324;	        //4-> gear		{1684(UP), 1324(DOWN)}
				virtual_rc_data[6] = 1552;    	        //6-> mode     	{1552(P), 1024(A), 496(F)
				double Right;
				drone->virtual_rc_control(virtual_rc_data);
		  		usleep(20000);
				ifstream FileR("Right.txt");
				FileR >> Right;
				ifstream FileF("Front.txt");
				FileF >> Front;
				cout << "Roll Right ";
				ifstream FileS("Stop.txt");
				FileS >> Stop;
				if ( Stop == 2.0)
				{
				drone->virtual_rc_disable();
				drone->landing();
				
				j = 50;
				}
				}
				if ( Stop == 2.0) break;
                                                       //Automatic hover
				                       drone->virtual_rc_enable();
                                                       usleep(2000);
                                                       cout<< "Drone stopped Pitching Forward." <<endl;
                                                       cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                       virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                       virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                       virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                       virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                       virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                       virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                       for (int i = 0; i < 200; i++)
                                                           {
                                                            drone->virtual_rc_control(virtual_rc_data);
                                                            usleep(23000);
                                                            while (kbhit())
                                                                   {
                                                                    killkey =getche();
                                                                    if(killkey == 'q')
                                                                      {
                                                                       return 0;
                                                                      }
                                                                   }	
				                           }
                if (Front > 2)
                    {
		     for (int j = 0; j < 150; j++)
		        {
		 	drone->virtual_rc_enable();
			usleep(20000);
			virtual_rc_data[0] = 1024;		//0-> roll     	[1024-660,1024+660] 
			virtual_rc_data[1] = 1024;		//1-> pitch    	[1024-660,1024+660]
			virtual_rc_data[2] = 1024;	        //2-> throttle 	[1024-660,1024+660]
			virtual_rc_data[3] = 1024-100;		//3-> yaw      	[1024-660,1024+660]
			virtual_rc_data[4] = 1324;	 	//4-> gear		{1684(UP), 1324(DOWN)}
			virtual_rc_data[6] = 1552;    	        //6-> mode     	{1552(P), 1024(A), 496(F)
				
			drone->virtual_rc_control(virtual_rc_data);
			usleep(20000);
			cout << "Yaw Left";
		        }
                    }
				ifstream FileR("Right.txt");
				FileR >> Right;
				if (Right < 1.1) break;
			
			} while (1);
			//Automatic hover
				                       drone->virtual_rc_enable();
                                                       usleep(2000);
                                                       cout<< "Drone stopped Pitching Forward." <<endl;
                                                       cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                       virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                       virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                       virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                       virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                       virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                       virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                       for (int i = 0; i < 200; i++)
                                                           {
                                                            drone->virtual_rc_control(virtual_rc_data);
                                                            usleep(23000);
                                                            while (kbhit())
                                                                   {
                                                                    killkey =getche();
                                                                    if(killkey == 'q')
                                                                      {
                                                                       return 0;
                                                                      }
                                                                   }	
				                           }	
			

			ifstream FileS("Stop.txt");
			FileS >> Stop;	
			if ( Stop != 2)
			{
			for (int p = 0; p < 150; p++)
			{
			drone->virtual_rc_enable();
			usleep(20000);
			virtual_rc_data[0] = 1024;		//0-> roll     	[1024-660,1024+660] 
			virtual_rc_data[1] = 1024;		//1-> pitch    	[1024-660,1024+660]
			virtual_rc_data[2] = 1024;	        //2-> throttle 	[1024-660,1024+660]
			virtual_rc_data[3] = 1024+100;		//3-> yaw      	[1024-660,1024+660]
			virtual_rc_data[4] = 1324;	 	//4-> gear		{1684(UP), 1324(DOWN)}
			virtual_rc_data[6] = 1552;    	        //6-> mode     	{1552(P), 1024(A), 496(F)
				
			drone->virtual_rc_control(virtual_rc_data);
			usleep(20000);
			cout << "Yaw Right ";
			ofstream Rightfile;
			Rightfile.open("Right.txt");
			Rightfile  << 6.0;
		  	Rightfile.close();
			}
			}
		}
			ifstream FileS("Stop.txt");
			FileS >> Stop;	
			if ( Stop == 2) break;
		}
				drone->virtual_rc_disable();
				drone->landing();
				break;
			//Automatic hover
				                       drone->virtual_rc_enable();
                                                       usleep(2000);
                                                       cout<< "Drone stopped Pitching Forward." <<endl;
                                                       cout<< "Drone Hovering for 5-Seconds..." <<endl;
                                                       virtual_rc_data[0] = 1024; //0-> roll     [1024-660,1024+660]
                                                       virtual_rc_data[1] = 1024; //1-> pitch    [1024-660,1024+660]
                                                       virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                                       virtual_rc_data[3] = 1024; //3-> yaw      [1024-660,1024+660]
                                                       virtual_rc_data[4] = 1684; //4-> gear     {1684(UP), 1324(DOWN)}
                                                       virtual_rc_data[6] = 1552; //6-> mode     {1552(P), 1024(A), 496(F)}

                                                       for (int i = 0; i < 200; i++)
                                                           {
                                                            drone->virtual_rc_control(virtual_rc_data);
                                                            usleep(23000);
                                                            while (kbhit())
                                                                   {
                                                                    killkey =getche();
                                                                    if(killkey == 'q')
                                                                      {
                                                                       return 0;
                                                                      }
                                                                   }	
				                           }


		

		
			
		
		}
			case 29:
				{
                                  //double throttle1 = 25;
                                  //double throttle2 = 50;
                                  //double throttle3 = 75;
                                  //double throttle4 = 100;
                                  double AddThrottle = 0;
                                  double roll = 0;
                                  double yaw = 0; 
                                  double pitch = 0;
                                  char killkey; //Variable for killing the code and return to manual flight. 
                                  //Automatic Throttle Up
                                    drone->virtual_rc_enable();
                                    usleep(20000);
                                    cout<< "Drone taking off..." <<endl;
                                    virtual_rc_data[0] = 1024;           //0-> roll     [1024-660,1024+660]
                                    virtual_rc_data[1] = 1024;           //1-> pitch     [1024-660,1024+660]
                                    virtual_rc_data[2] = 1024; //2-> throttle [1024-660,1024+660]
                                    virtual_rc_data[3] = 1024;           //3-> yaw       [1024-660,1024+660]
                                    virtual_rc_data[4] = 1684;           //4-> gear {1684(UP), 1324(DOWN)}
                                    virtual_rc_data[6] = 1552;           //6-> mode     {1552(P), 1024(A), 496(F)}

                                    cout<< "AddThrottle = 0 " <<endl;
                                   
                                    for (int i = 0; i < 25; i++)
                                         { 
                                          AddThrottle = i;
                                          drone->virtual_rc_enable(); 
                                          virtual_rc_data[2] = virtual_rc_data[2] + AddThrottle ; //2-> throttle [1024-660,1024+660]
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(250000);
                                          while (kbhit())
                                                 {
                                                  killkey =getche();
                                                  if(killkey == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                         }
                                    cout<< "AddThrottle = 25" <<endl;                       

                                    

                                    for (int i = 0; i < 25; i++)
                                         { 
                                          AddThrottle = i;
                                          drone->virtual_rc_enable(); 
                                          virtual_rc_data[2] = virtual_rc_data[2] + AddThrottle ; //2-> throttle [1024-660,1024+660]
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(250000);
                                          while (kbhit())
                                                 {
                                                 killkey =getche();
                                                if(killkey == 'q')
                                                   {
                                                      return 0;
                                                     }
                                                 }
                                         }
                                    cout<< "AddThrottle = 50" <<endl;

                                    for (int i = 0; i < 25; i++)
                                         { 
                                          AddThrottle = i;
                                          drone->virtual_rc_enable(); 
                                          virtual_rc_data[2] = virtual_rc_data[2] + AddThrottle ; //2-> throttle [1024-660,1024+660]
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(250000);
                                          while (kbhit())
                                                 {
                                                  killkey =getche();
                                                  if(killkey == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                         }
                                    cout<< "AddThrottle = 75" <<endl;

                                   for (int i = 0; i < 25; i++)
                                         { 
                                          AddThrottle = i;
                                          drone->virtual_rc_enable(); 
                                          virtual_rc_data[2] = virtual_rc_data[2] + AddThrottle ; //2-> throttle [1024-660,1024+660]
                                          drone->virtual_rc_control(virtual_rc_data);
                                          usleep(250000);
                                          while (kbhit())
                                                 {
                                                  killkey =getche();
                                                  if(killkey == 'q')
                                                     {
                                                      return 0;
                                                     }
                                                 }
                                         }
                                    cout<< "AddThrottle = 100" <<endl;
                                    cout<< "StartSleep = 5" <<endl;
                                    sleep(5);
                                    cout<< "EndSleep = 5" <<endl;
                                    
                                    
                                drone->virtual_rc_enable();
				drone->landing();
				break;
                         

			}
			case 30:
				//mission waypoint set speed
				drone->mission_waypoint_set_speed((float)5);
				break;
			case 31:
				//mission waypoint get speed
				printf("%f", drone->mission_waypoint_get_speed());
				break;
			case 32:
				//mission hotpoint set speed
				drone->mission_hotpoint_set_speed((float)5,(uint8_t)1);
				break;
			case 33:
				//mission hotpoint set radius
				drone->mission_hotpoint_set_radius((float)5);
				break;
			case 34:
				//mission hotpoint reset yaw
				drone->mission_hotpoint_reset_yaw();
				break;
			case 35:
				//mission followme update target
				for (int i = 0; i < 20; i++)
				{
					followme_target.latitude = 22.540091 + i*0.000001;
					followme_target.longitude = 113.946593 + i*0.000001;
					followme_target.altitude = 100;
					drone->mission_followme_update_target(followme_target);
					usleep(20000);
				}
				break;
            case 37:
                printf("Mobile Data Commands mode entered. Use OSDK Mobile App to use this feature \n");
                printf("End program to exit this mode \n");
                while(1)
                {
                ros::spinOnce();
                }
			case 36:
				hotpoint_task = drone->mission_hotpoint_download();

            default:
                break;
        }
        main_operate_code = -1;
        Display_Main_Menu();
    }
    return 0;
}

//! Callback functions for Mobile Commands
    void ObtainControlMobileCallback(DJIDrone *drone)
    {
      drone->request_sdk_permission_control();
    }

    void ReleaseControlMobileCallback(DJIDrone *drone)
    {
      drone->release_sdk_permission_control();
    }

    void TakeOffMobileCallback(DJIDrone *drone)
    {
      drone->takeoff();
    }

    void LandingMobileCallback(DJIDrone *drone)
    {
      drone->landing();
    }

    void GetSDKVersionMobileCallback(DJIDrone *drone)
    {
      drone->check_version();
    }

    void ArmMobileCallback(DJIDrone *drone)
    {
      drone->drone_arm();
    }

    void DisarmMobileCallback(DJIDrone *drone)
    {
      drone->drone_disarm();
    }

    void GoHomeMobileCallback(DJIDrone *drone)
    {
      drone->gohome();
    }

    void TakePhotoMobileCallback(DJIDrone *drone)
    {
      drone->take_picture();
    }

    void StartVideoMobileCallback(DJIDrone *drone)
    {
      drone->start_video();
    }

    void StopVideoMobileCallback(DJIDrone *drone)
    {
      drone->stop_video();
    }

    void DrawCircleDemoMobileCallback(DJIDrone *drone)
    {
        static float R = 2;
        static float V = 2;
        static float x;
        static float y;
        int circleRadius;
        int circleHeight;
        float Phi =0, circleRadiusIncrements;
        int x_center, y_center, yaw_local;

        circleHeight = 7;
        circleRadius = 7;

        x_center = drone->local_position.x;
        y_center = drone->local_position.y;
        circleRadiusIncrements = 0.01;

        for(int j = 0; j < 1000; j ++)
        {
            if (circleRadiusIncrements < circleRadius)
            {
                x =  x_center + circleRadiusIncrements;
                y =  y_center;
                circleRadiusIncrements = circleRadiusIncrements + 0.01;
                drone->local_position_control(x ,y ,circleHeight, 0);
                usleep(20000);
            }
                else
            {
                break;
            }
        }


        /* start to draw circle */
        for(int i = 0; i < 1890; i ++)
        {
            x =  x_center + circleRadius*cos((Phi/300));
            y =  y_center + circleRadius*sin((Phi/300));
            Phi = Phi+1;
            drone->local_position_control(x ,y ,circleHeight, 0);
            usleep(20000);
        }

    }
    void DrawSquareDemoMobileCallback(DJIDrone *drone)
    {
    /*draw square sample*/
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            3, 3, 0, 0 );
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            -3, 3, 0, 0);
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            -3, -3, 0, 0);
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            3, -3, 0, 0);
            usleep(20000);
        }
    }

     void GimbalControlDemoMobileCallback(DJIDrone *drone)
        {
        drone->gimbal_angle_control(0, 0, 1800, 20);
        sleep(2);
        drone->gimbal_angle_control(0, 0, -1800, 20);
        sleep(2);
        drone->gimbal_angle_control(300, 0, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(-300, 0, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(0, 300, 0, 20);
        sleep(2);
        drone->gimbal_angle_control(0, -300, 0, 20);
        sleep(2);
        drone->gimbal_speed_control(100, 0, 0);
        sleep(2);
        drone->gimbal_speed_control(-100, 0, 0);
        sleep(2);
        drone->gimbal_speed_control(0, 0, 200);
        sleep(2);
        drone->gimbal_speed_control(0, 0, -200);
        sleep(2);
        drone->gimbal_speed_control(0, 200, 0);
        sleep(2);
        drone->gimbal_speed_control(0, -200, 0);
        sleep(2);
        drone->gimbal_angle_control(0, 0, 0, 20);
        }

    void AttitudeControlDemoMobileCallback(DJIDrone *drone)
    {
        /* attitude control sample*/
        drone->takeoff();
        sleep(8);


        for(int i = 0; i < 100; i ++)
        {
            if(i < 90)
                drone->attitude_control(0x40, 0, 2, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 2, 0, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, -2, 0, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, 2, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, -2, 0, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, 0, 0.5, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0x40, 0, 0, -0.5, 0);
            else
                drone->attitude_control(0x40, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0xA, 0, 0, 0, 90);
            else
                drone->attitude_control(0xA, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        for(int i = 0; i < 200; i ++)
        {
            if(i < 180)
                drone->attitude_control(0xA, 0, 0, 0, -90);
            else
                drone->attitude_control(0xA, 0, 0, 0, 0);
            usleep(20000);
        }
        sleep(1);

        drone->landing();

    }
    void LocalNavigationTestMobileCallback(DJIDrone *drone)
    {

    }
    void GlobalNavigationTestMobileCallback(DJIDrone *drone)
    {

    }
    void WaypointNavigationTestMobileCallback(DJIDrone *drone)
    {

    }
    void VirtuaRCTestMobileCallback(DJIDrone *drone)
    {
        //virtual RC test data
        uint32_t virtual_rc_data[16];
        //virtual rc test 1: arm & disarm
        drone->virtual_rc_enable();
        usleep(20000);

        virtual_rc_data[0] = 1024;  //0-> roll      [1024-660,1024+660]
        virtual_rc_data[1] = 1024;  //1-> pitch     [1024-660,1024+660]
        virtual_rc_data[2] = 1024+660;  //2-> throttle  [1024-660,1024+660]
        virtual_rc_data[3] = 1024;  //3-> yaw       [1024-660,1024+660]
        virtual_rc_data[4] = 1684;      //4-> gear      {1684(UP), 1324(DOWN)}
        virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

        for (int i = 0; i < 100; i++){
            drone->virtual_rc_control(virtual_rc_data);
            usleep(20000);
        }

        //virtual rc test 2: yaw
        drone->virtual_rc_enable();
        virtual_rc_data[0] = 1024;      //0-> roll      [1024-660,1024+660]
        virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
        virtual_rc_data[2] = 1024-200;  //2-> throttle  [1024-660,1024+660]
        virtual_rc_data[3] = 1024;      //3-> yaw       [1024-660,1024+660]
        virtual_rc_data[4] = 1324;      //4-> gear      {1684(UP), 1324(DOWN)}
        virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

        for(int i = 0; i < 100; i++) {
            drone->virtual_rc_control(virtual_rc_data);
            usleep(20000);
        }
        drone->virtual_rc_disable();
    }

void StartMapLASLoggingMobileCallback(DJIDrone *drone)
{
  system("roslaunch point_cloud_las start_velodyne_and_loam.launch &");
  system("rosrun point_cloud_las write _topic:=/laser_cloud_surround _folder_path:=. &");
}

void StopMapLASLoggingMobileCallback(DJIDrone *drone)
{
  system("rosnode kill /write_LAS /scanRegistration /laserMapping /transformMaintenance /laserOdometry  &");
}

void StartCollisionAvoidanceCallback(DJIDrone *drone)
{
  uint8_t freq[16];
  freq[0] = 1;    // 0 - Timestamp
  freq[1] = 4;    // 1 - Attitude Quaterniouns
  freq[2] = 1;    // 2 - Acceleration
  freq[3] = 4;    // 3 - Velocity (Ground Frame)
  freq[4] = 4;    // 4 - Angular Velocity (Body Frame)
  freq[5] = 3;    // 5 - Position
  freq[6] = 0;    // 6 - Magnetometer
  freq[7] = 3;    // 7 - M100:RC Channels Data, A3:RTK Detailed Information
  freq[8] = 0;    // 8 - M100:Gimbal Data, A3: Magnetometer
  freq[9] = 3;    // 9 - M100:Flight Status, A3: RC Channels
  freq[10] = 0;   // 10 - M100:Battery Level, A3: Gimble Data
  freq[11] = 2;   // 11 - M100:Control Information, A3: Flight Status

  drone->set_message_frequency(freq);
  usleep(1e4);
  system("roslaunch dji_collision_avoidance from_DJI_ros_demo.launch &");
}

void StopCollisionAvoidanceCallback(DJIDrone *drone)
{
  drone->release_sdk_permission_control();
  system("rosnode kill /drone_tf_builder /dji_occupancy_grid_node /dji_collision_detection_node /collision_velodyne_nodelet_manager /manual_fly");
  usleep(1e4);
  drone->request_sdk_permission_control();
}

