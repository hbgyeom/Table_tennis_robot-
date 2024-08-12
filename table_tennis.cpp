#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>
#include <ctime>
#include <mutex>
#include "dynamixel_sdk.h" // Uses Dynamixel SDK library
#include "CLinear_actu.h"
#include <time.h>


//#define TOP_W = 270
//#define TOP_H = 170
//#define SIDE_W = 160
//#define SIDE_H = 100
//#define MAX_HALF_WIDTH = 70

/*

*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*   
*
*
    
*/
char mot_file[] = "C:/Users/user/Desktop/DynamixelSDK-master/DynamixelSDK-master/c++/example/protocol1.0/read_write/win64/src/mot/ajin20190628.mot";        // *.mot file Path

int vel = 2000;
int accel = 700;
int current_position = 65;   // current position�� ��ġ�� �׻� �����
int shared_position = 65;
float topcam_m = 0;
int fire = 0;
int delta = 0;    // for velocity calculation
int ball_position = 0;  // for velocity calc
int serve = 1;
int retry = 0;
int reset_linear = 0;
std::chrono::system_clock::time_point abs_start = std::chrono::system_clock::now();



// Dynamixel settings
#define ADDR_MX_TORQUE_ENABLE           24
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_MOVING                  46

#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_MX_MOVING                   1

#define PROTOCOL_VERSION                1.0

#define DXL12_ID                        12  // �ٴ�
#define DXL3_ID                         3   //���Ʒ�
#define DXL2_ID                         2   //�߻�

#define BAUDRATE                        115200
#define DEVICENAME                      "COM4"

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MOVING_STATUS_THRESHOLD     10

#define ESC_ASCII_VALUE                 0x1b

int getch() {
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}

int kbhit(void) {
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
#elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
#endif
}


CLinear_actu::CLinear_actu()
{
    DWORD Code = AxlOpen(7);
    if (Code == AXT_RT_SUCCESS)
    {
        printf("Library Reset. \n");
        //Check for Motion Module
        DWORD uStatus;
        Code = AxmInfoIsMotionModule(&uStatus);
        if (Code == AXT_RT_SUCCESS)
        {
            printf("Library Reset.\n");
            if (uStatus == STATUS_EXIST)
            {
                printf("Library Reset. \n");

                AxmMotLoadParaAll(mot_file);

                AxmStatusSetActPos(0, 0.0);
                AxmStatusSetCmdPos(0, 0.0);

                AxmSignalServoOn(0, ENABLE);

                AxmMotSetAbsRelMode(0, 1); //0->abs, 1->Rel
                AxmMotSetProfileMode(0, 3);   //0->symetric trapezode, 1->unsymetric trapezode, 2->reserved, 3->symetric S Curve, 4->unsymetric S Cuve
            }
        }
    }
}


CLinear_actu::~CLinear_actu()
{
    AxmSignalServoOn(0, 0);
    AxlClose();
}


void CLinear_actu::move_actu(int pos)
{
    if (pos > 130) {
        pos = 130 - current_position;
        current_position = 130;
    }
    else if (pos < 0) {
        pos = -current_position;
        current_position = 0;
    }
    else {
        pos = pos - current_position;
        current_position = current_position + pos;
    }
    shared_position = current_position;
    AxmMovePos(0, pos, vel, accel, accel);
    //printf("moving lin actu\n");
    DWORD uStatus;
    AxmStatusReadInMotion(0, &uStatus);
    while (uStatus)
    {
        AxmStatusReadInMotion(0, &uStatus);
    }
    printf("----- finish moving actu -----\n");
}


int thread1Function() {
    //std::chrono::system_clock::time_point top_abs_time = std::chrono::system_clock::now();      // buffer�� ����ϸ� 0.032�� ���� �� ����

    int frame_width = 640;
    int frame_height = 480;
    int top_exp = -8;
    int side_exp = -7;
    int top_desired_fps = 90;
    int side_desired_fps = 90;
    //float top_abs_time = 0;   // �������� �޾ƿ��� ���� �ð��� ������ �ƴϸ� �������� �޾ƿ� ���� �ð��� ������ 
    float side_abs_time = 0;   // �������� �޾ƿ��� delay�� 0.003 ~ 0.016 ���� 
    // �׷��� ��û ������ 0.06�� delay�� �ɸ��� �͵� ����
    float prev_side_abs_time = 0;
    int current_state = 0;   // ���� ���� ���°� 0, ���� ġ�� ���°� 1, ���� ���� ���°� 2, ( ���� ����ϴ� ���°� 3 )
    // ���� ����ϴ� ���� 0 , ���� ���� ���� 1, ���� ġ�� ���� 2, ( ���� ���� ���� 3 )
    // state�� �����ϴ� ����� top cam�� y��ǥ ��ȭ�� �̿��ؼ� ���ϰų� 
    // state�� �̿��ؼ� �����Ѵٸ� state transition�� ������ ���� ���� y��ǥ�� �̿��ؼ� ���ϴ� ����� ����
    // �̹���� ����ϸ� �� ���������ε� �ٷ� state�� ���� �� �ִٴ� ������ ����   
    float top_k = 0.365f;   // Ź���� �ٴڸ鿡���� �ȼ� cm ����. top cam���κ��� 140cm ����.   46.6cm / 171px
    float side_k = 0.6202f;   // Ź���� �߾Ӹ鿡���� �ȼ� cm ����. side cam���κ��� 287cm ����   80cm / 129px
    float prev_h = 0, h = 0;
    int top_x = 0, top_y = 0, top_w = 0, top_h = 0; // 2���� 1�� ���߿� �Ѳ����� �ϸ� ������ float�� 
    int prev_top_y = 0;
    int side_x = 0, side_y = 0, side_w = 0, side_h = 0;
    int top_center_x = 0, top_center_y = 0, side_center_x = 0, side_center_y = 0;
    int prev_top_center_x = 0, prev_top_center_y = 0, prev_side_center_x = 0, prev_side_center_y = 0;
    int estimated_position = 0;
    int delta_y = 0;
    int pp_top_center_x = 0, pp_top_center_y = 0;
    int pppx = 0, pppy = 0;


    CLinear_actu actuator;

    cv::Scalar top_low(0, 50, 100);
    cv::Scalar top_upp(40, 200, 255);
    cv::Scalar side_low(0, 50, 60);
    cv::Scalar side_upp(40, 255, 255);

    cv::Mat top_frame;
    cv::Mat top_mask;
    cv::Mat side_frame;
    cv::Mat side_mask;

    //cv::VideoCapture topcam(0, cv::CAP_DSHOW);
    //cv::VideoCapture sidecam(1, cv::CAP_DSHOW);
    cv::VideoCapture topcam(0);
    //cv::VideoCapture sidecam(0);   // ���ڸ� ���� �����ϴ� ��� ����

    // Set the desired frame rate         initializing camera 
    topcam.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
    topcam.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
    topcam.set(cv::CAP_PROP_FPS, top_desired_fps);
    topcam.set(cv::CAP_PROP_EXPOSURE, top_exp);

    //sidecam.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
    //sidecam.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
    //sidecam.set(cv::CAP_PROP_FPS, side_desired_fps);
    //sidecam.set(cv::CAP_PROP_EXPOSURE, side_exp);

    printf("\n----- top cam fps  : %f -----", topcam.get(cv::CAP_PROP_FPS));
    //printf("\n----- side cam fps : %f -----\n", sidecam.get(cv::CAP_PROP_FPS));

    // ù�����ӿ��� ���̵带 �����ؼ� ���̵��� ù�������� ��������  top -> side -> side


    while (true) {      //  ����
        std::chrono::system_clock::time_point top_abs_time = std::chrono::system_clock::now();
        topcam >> top_frame;
        top_frame = top_frame(cv::Range(80, 480), cv::Range(145, 493));      // 260 ���Ʒ��� ������ �� Ÿ��Ʈ�ϰ�, top�� ��Ī�ǰ� �߶���
        cv::inRange(top_frame, top_low, top_upp, top_mask);
        cv::Rect boundingRect = cv::boundingRect(top_mask);

        top_x = boundingRect.x;
        top_y = boundingRect.y;
        top_w = boundingRect.width;
        top_h = boundingRect.height;

        if (prev_top_y == top_y || top_w == 0 || top_h > 30) {
            if (reset_linear) {
                reset_linear = 0;
                printf("==== linear actu reset ====\n");
                actuator.move_actu(65);
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
            continue;  // ���ǿ� ���� ������ �ٷ� ���� ������ �Ѿ
        }

        top_center_x = 2 * top_x + top_w;   // ���� �����̸鼭 �������� ���� ���ؼ� �� �κ��� �����ϴ� ���� ������
        top_center_y = 2 * top_y + top_h;   // calculate center of rectangle
        prev_top_y = top_y;

        printf("ball detected, ppp = (%d, %d),  pp = (%d, %d),  p = (%d, %d), current = (%d, %d)\n", pppx, pppy, pp_top_center_x, pp_top_center_y, prev_top_center_x, prev_top_center_y, top_center_x, top_center_y);
        printf("B Radius w = %d, h = %d\n", top_w, top_h);
        delta_y = pp_top_center_y - pppy;

        if (delta_y > 100) {  // use p and current
            abs_start = top_abs_time;
            topcam_m = -(top_center_y - pp_top_center_y) / (top_center_x - pp_top_center_x + 0.0000019073486328125f);
            //printf("^^^^^^^^^^ ���� first ball detected ^^^^^^^^^^^^  m is %f\n\n", topcam_m);
            if (top_center_y - prev_top_center_y > 60) {
                delta = top_center_y - prev_top_center_y;    // pass over? (gyeom's expression) to global va
            }
            else {
                pppx = pp_top_center_x;
                pppy = pp_top_center_y;
                pp_top_center_x = prev_top_center_x;    // use p and current. pp can be return ball
                pp_top_center_y = prev_top_center_y;
                prev_top_center_x = top_center_x;
                prev_top_center_y = top_center_y;
                continue;
            }
            fire = 1;
            delta = top_center_y - prev_top_center_y;    // pass over? (gyeom's expression) to global var
            ball_position = top_center_y;

            //linear actuator
            //estimated_position = 131 - 0.4f * top_center_x + (289 - 0.4f * top_center_y) / topcam_m;
            estimated_position = 65 + 175.5f * top_k - top_center_x * top_k / 2 + (225 + 160.5f * top_k - top_center_y * top_k / 2) / topcam_m;
            actuator.move_actu(static_cast<int>(estimated_position));

            printf("estimated location is %d\n", static_cast<int>(estimated_position));
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            topcam >> top_frame;
            topcam >> top_frame;

            delta_y = 0;
            retry = 1;
            top_center_x = 2048;
            top_center_y = 2048;
            pp_top_center_x = 2048;    // use p and current. pp can be return ball
            pp_top_center_y = 2048;
            prev_top_center_x = 2048;
            prev_top_center_y = 2048;
            pppx = 2048;
            pppy = 2048;
        }
        else {
            pppx = pp_top_center_x;
            pppy = pp_top_center_y;
            pp_top_center_x = prev_top_center_x;    // use p and current. pp can be return ball
            pp_top_center_y = prev_top_center_y;
            prev_top_center_x = top_center_x;
            prev_top_center_y = top_center_y;

        }

    }

    while (true) {
        //top_frame = top_frame(cv::Range(0, 480), cv::Range(0, 640));
        //side_frame = side_frame(cv::Range(0, 480), cv::Range(0, 640));
        std::chrono::system_clock::time_point top_abs_time = std::chrono::system_clock::now();
        topcam >> top_frame;
        //std::chrono::system_clock::time_point side_abs_time = std::chrono::system_clock::now();
        //sidecam >> side_frame;      // ���� top w != 0�� Ȯ�εǰ� ���� �������� �޾ƿ��°� ������ ����.... 
        // �ٵ� side �ð� ������ Ȯ���ϱ� ���ؼ��� side�� ���� �޾ƿ��°� �����Ҽ���?

        //top_frame = top_frame(cv::Range(70, 410), cv::Range(50, 590));      // ���Ʒ��� ������ �� Ÿ��Ʈ�ϰ�, top�� ��Ī�ǰ� �߶���
        //top_frame = top_frame(cv::Range(70, 410), cv::Range(70, 570));      // 250 ���Ʒ��� ������ �� Ÿ��Ʈ�ϰ�, top�� ��Ī�ǰ� �߶���
        top_frame = top_frame(cv::Range(80, 440), cv::Range(145, 493));      // 260 ���Ʒ��� ������ �� Ÿ��Ʈ�ϰ�, top�� ��Ī�ǰ� �߶���
        cv::inRange(top_frame, top_low, top_upp, top_mask);
        //cv2.imwrite('R-RGB.jpg', image[:, : , 2]) // use only red channel for faster code
        cv::Rect boundingRect = cv::boundingRect(top_mask);

        top_x = boundingRect.x;
        top_y = boundingRect.y;
        top_w = boundingRect.width;
        top_h = boundingRect.height;
        if (prev_top_y == top_y || top_w == 0 || top_h > 30) {
            continue;  // ���ǿ� ���� ������ �ٷ� ���� ������ �Ѿ
        }

        top_center_x = top_x + top_w / 2;   // ���� �����̸鼭 �������� ���� ���ؼ� �� �κ��� �����ϴ� ���� ������
        top_center_y = top_y + top_h / 2;   // calculate center of rectangle
        prev_top_y = top_y;
        printf("B Radius w = %d, h = %d\n", top_w, top_h);
        printf("ball detected,  pp = (%d, %d),  p = (%d, %d), current = (%d, %d)\n", pp_top_center_x, pp_top_center_y, prev_top_center_x, prev_top_center_y, top_center_x, top_center_y);

        delta_y = pp_top_center_y - pppy;

        if (delta_y > 55) {  // use p and current
            abs_start = top_abs_time;
            topcam_m = -(top_center_y - pp_top_center_y) / (top_center_x - pp_top_center_x + 0.0000019073486328125f);
            //printf("^^^^^^^^^^  first ball detected ^^^^^^^^^^^^^  m is %f\n\n", topcam_m);
            if (top_center_y - prev_top_center_y > 30) {
                retry = 1;
                delta = top_center_y - prev_top_center_y;    // pass over? (gyeom's expression) to global va
            }
            else {
                continue;
            }
            fire = 1;
            ball_position = top_center_y;

            //linear actuator
            estimated_position = 131 - 0.3773f * top_center_x + (285.4f - 0.3773f * top_center_y) / topcam_m;
            actuator.move_actu(static_cast<int>(estimated_position));

            //("estimated location is %d\n", static_cast<int>(estimated_position));
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            topcam >> top_frame;
            topcam >> top_frame;

            delta_y = 0;
            top_center_x = 500;
            top_center_y = 500;
            pp_top_center_x = 500;    // use p and current. pp can be return ball
            pp_top_center_y = 500;
            prev_top_center_x = 500;
            prev_top_center_y = 500;
            pppx = 500;
            pppy = 500;
        }
        else {
            pppx = pp_top_center_x;
            pppy = pp_top_center_y;
            pp_top_center_x = prev_top_center_x;    // use p and current. pp can be return ball
            pp_top_center_y = prev_top_center_y;
            prev_top_center_x = top_center_x;
            prev_top_center_y = top_center_y;
        }
    }
    return 0;
}








int thread2Function() {
    dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t dxl_present_position = 0;              // Present position

    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        _getch();
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        _getch();
        return 0;
    }

    // Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("2 : %s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("2 : %s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel 2 has been successfully connected \n");
    }



    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL12_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("12 : %s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("12 : %s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel 12 has been successfully connected \n");
    }



    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("3 : %s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("3 : %s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel 3 has been successfully connected \n");
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, 2700, &dxl_error); // ���� ���·� �ٲ���
    //dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL3_ID, ADDR_MX_GOAL_POSITION, 1565, &dxl_error); // ���Ʒ� ���� ����     ���� ����Ʈ
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL3_ID, ADDR_MX_GOAL_POSITION, 1590, &dxl_error); // ���Ʒ� ���� ����
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL12_ID, ADDR_MX_GOAL_POSITION, 740, &dxl_error); // �¿� ���� ����





    float theta = 0;
    float beta = 0;
    int tilt = 0;
    float kk = 23; // air resistance constant(>1) * 0.032           ms  
    float cc = 0;   // delay offset constant                
    int estimated_time = 0;
    int elapsed_time = 0;
    float get_delta = 0;


    while (true) {
        if (fire) {
            // �߰��� �� : ���� ��� �ð� ���
            // ��ũź��Ʈ �߰��ϱ� 
            // get ball position and delta_y. calculate velocity and time.
            get_delta = static_cast<float>(delta) / 2;
            theta = atan(topcam_m);
            beta = atan(400 / (shared_position - 65.0000019073486328125f));
            //beta = 1.5708;

            if (theta < 0) {
                theta = 3.141592f + theta;
            }
            if (beta < 0) {
                beta = 3.141592f + beta;
            }
            tilt = static_cast<int>((theta + beta) * 326 - 284);

            printf("theta is %f, beta is %f, tilt value is %d\n", theta, beta, tilt);

            // �޴� ���� ����
            // dxl_comm_result = packetHandler->write2ByteTxRx(portHandler,             , ADDR_MX_GOAL_POSITION, 480, &dxl_error); 

            // �޴� ���Ʒ� ���� ����
            // dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL3_ID, ADDR_MX_GOAL_POSITION, 480, &dxl_error); 

           // dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL12_ID, ADDR_MX_GOAL_POSITION, static_cast<int>((theta + beta) * 325.95 - 1024 + 740), &dxl_error);    // tilt
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL12_ID, ADDR_MX_GOAL_POSITION, static_cast<int>((theta + beta) * 322 - 1024 + 740), &dxl_error);    // tilt
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL12_ID, ADDR_MX_GOAL_POSITION, tilt, &dxl_error);    // tilt


            printf("BP = %d, delta = %f, delta (cm) = %f\n", ball_position, get_delta, get_delta * 0.3773f);
            //estimated_time = static_cast<int>(kk * (317 - 0.2725 * ball_position) / delta);
            //estimated_time = static_cast<int>(1000 * (std::exp((295.4 - 0.2725 * ball_position) * 0.00134) - 1) / (0.00134 * 0.2725 * delta / 0.016)); //�������� �� (e^sk - 1) / k * v_0�� ������ ��
            //estimated_time = static_cast<int>(1000 * (std::exp((225 + 160*.3373   ->    285.4 - 0.3773 * ball_position) * 0.00134) - 1) / (0.00134 * 0.3773 * delta / 0.016)); //�������� �� (e^sk - 1) / k * v_0�� ������ ��
            estimated_time = static_cast<int>((std::exp(0.396 - 0.000505582 * static_cast<int>(ball_position)) - 1) * 31646.7f / get_delta); //�������� �� (e^sk - 1) / k * v_0�� ������ ��
            std::chrono::system_clock::time_point abs_end = std::chrono::system_clock::now();
            std::chrono::duration<int, std::milli> elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(abs_end - abs_start);
            printf("�߻�~~~~~~ \nelapsed time is %d, estimated time is %d, delta = %f\n\n", elapsed_time.count(), estimated_time, get_delta);
            std::this_thread::sleep_for(std::chrono::milliseconds(estimated_time - elapsed_time.count()));

            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, 1405, &dxl_error);
            std::this_thread::sleep_for(std::chrono::milliseconds(800));
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, 2700, &dxl_error);
            printf("�ǵ��ư�����~~~~~~~~~~~~~~~~~~~\n\n");
            fire = 0;
            //serve = 0;






            // linear return code

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ������ ��� �����ؼ� �����̰� �߻��ϴµ� polling
        //if (!serve) {
        //    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL3_ID, ADDR_MX_GOAL_POSITION, 1580, &dxl_error); // ���Ʒ� ���� ����

        //}
    }



    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Close port
    portHandler->closePort();

}


int thread3Function() {
    //CLinear_actu actuatorp;
    while (true) {
        if (retry) {
            retry = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            if (!retry) {
                reset_linear = 1;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300)); // ������ ��� �����ؼ� �����̰� �߻��ϴµ� polling
    }
}



int main() {
    std::thread thread1(thread1Function);   // camera and linear motor move
    std::thread thread2(thread2Function);   // fire motor and tilt and height
    std::thread thread3(thread3Function);   // reset 

    thread1.join();
    thread2.join();
    thread3.join();

    return 0;
}



