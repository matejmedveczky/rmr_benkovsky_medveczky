 #include "robot.h"
#include <cmath>

robot::robot(QObject *parent) : QObject(parent)
{
    qRegisterMetaType<LaserMeasurement>("LaserMeasurement");
#ifndef DISABLE_OPENCV
    qRegisterMetaType<cv::Mat>("cv::Mat");
#endif
#ifndef DISABLE_SKELETON
    qRegisterMetaType<skeleton>("skeleton");
#endif
}

void robot::initAndStartRobot(std::string ipaddress)
{
    forwardspeed=0;
    rotationspeed=0;

    first_tick = true;
    old_left_encoder = 0.0;
    old_right_encoder = 0.0;
    angle_old = 0.0;
    old_timestamp = 0.0;

    x = 0;
    y = 0;
    fi = 0;

    prev_v = 0.0;
    prev_w = 0.0;


    robotCom.setLaserParameters([this](const std::vector<LaserData>& dat)->int{return processThisLidar(dat);},ipaddress);
    robotCom.setRobotParameters([this](const TKobukiData& dat)->int{return processThisRobot(dat);},ipaddress);
#ifndef DISABLE_OPENCV
    robotCom.setCameraParameters(std::bind(&robot::processThisCamera,this,std::placeholders::_1),"http://"+ipaddress+":8000/stream.mjpg");
#endif
#ifndef DISABLE_SKELETON
    robotCom.setSkeletonParameters(std::bind(&robot::processThisSkeleton,this,std::placeholders::_1));
#endif
    robotCom.robotStart();
}

void robot::setSpeedVal(double forw, double rots)
{
    forwardspeed=forw;
    rotationspeed=rots;
    useDirectCommands=0;
}

void robot::setSpeed(double forw, double rots)
{
    if(forw==0 && rots!=0)
        robotCom.setRotationSpeed(rots);
    else if(forw!=0 && rots==0)
        robotCom.setTranslationSpeed(forw);
    else if((forw!=0 && rots!=0))
        robotCom.setArcSpeed(forw,forw/rots);
    else
        robotCom.setTranslationSpeed(0);
    useDirectCommands=1;
}

void robot::resetRobot(){
    x = 0;
    y = 0;
    fi = 0;
}

void robot::returnHome(){
    x_des = 0.0;
    y_des = 0.0;
}

int robot::processThisRobot(const TKobukiData &robotdata)
{
    long double tick = robotCom.getTickToMeter();

    if(first_tick) {
        old_left_encoder  = robotdata.EncoderLeft;
        old_right_encoder = robotdata.EncoderRight;
        angle_old         = robotdata.GyroAngle/100;
        first_tick        = false;
        return 0;
    }

    double delta_left  = robotdata.EncoderLeft  - old_left_encoder;
    double delta_right = robotdata.EncoderRight - old_right_encoder;

    unsigned timestamp = robotdata.timestamp;
    unsigned dt = timestamp - old_timestamp;

    if(delta_left  >  32767) delta_left  -= 65536;
    if(delta_left  < -32767) delta_left  += 65536;
    if(delta_right >  32767) delta_right -= 65536;
    if(delta_right < -32767) delta_right += 65536;

    //double left_distance  = tick * (robotdata.EncoderLeft  - old_left_encoder);
    //double right_distance = tick * (robotdata.EncoderRight - old_right_encoder);

    double left_distance  = tick * delta_left;
    double right_distance = tick * delta_right;

    old_left_encoder  = robotdata.EncoderLeft;
    old_right_encoder = robotdata.EncoderRight;

    double gyro_now  = robotdata.GyroAngle / 100.0;
    double delta_deg = gyro_now - angle_old;
    angle_old        = gyro_now;

    while(delta_deg >  180.0) delta_deg -= 360.0;
    while(delta_deg < -180.0) delta_deg += 360.0;

    fi += delta_deg * M_PI / 180.0;

    while(fi >  M_PI) fi -= 2*M_PI;
    while(fi < -M_PI) fi += 2*M_PI;

    double step_dist = (left_distance + right_distance) / 2.0;
    x += step_dist * cos(fi);
    y += step_dist * sin(fi);

    double dx      = x_des - x;
    double dy      = y_des - y;
    double err_lin = sqrt(dx*dx + dy*dy);
    double err_ang = atan2(dy, dx) - fi;

    while(err_ang >  M_PI) err_ang -= 2*M_PI;
    while(err_ang < -M_PI) err_ang += 2*M_PI;

    double Kp_lin = 0.3;
    double Kp_ang = 1.2;

    double v = 0, w = 0;

    if(err_lin < 0.02) {
        v = 0;
        w = 0;
    } else {
        if(std::abs(err_ang) > 0.9) {
            v = 0;
            w = Kp_ang * err_ang;
        }
        else {
            v = Kp_lin * err_lin * 1000.0;
            w = Kp_ang * err_ang;
        }

        if ((v - prev_v) > 5){
            v = prev_v + 5;
        }

        if ((w - prev_w) > 0.3){
            w = prev_w + 0.3;
        }
        else if ((w - prev_w) < -0.3){
            w = prev_w - 0.3;
        }
    }

    // dt = 40hz

    prev_v = v;
    prev_w = w;

    v = std::clamp(v,  -400.0, 400.0); // mm/s
    w = std::clamp(w, -2.0,   2.0); // rad/s

    setSpeedVal(v, w);

    if(datacounter % 5 == 0)
    {
        cout << "\nRobot pos x/y/angle: " << x << " " << y << " " << fi;
        cout << "\nGyro raw/delta_deg:  " << gyro_now << " " << delta_deg;
        cout << "\nErrors lin/ang:      " << err_lin  << " " << err_ang;
        cout << "\nCommands v/w:        " << v        << " " << w<<std::endl;
        emit publishPosition(x, y, fi);
    }

    // --- dispatch speeds to robot ---
    if(useDirectCommands==0)
    {
        if(forwardspeed==0 && rotationspeed!=0)
            robotCom.setRotationSpeed(rotationspeed);
        else if(forwardspeed!=0 && rotationspeed==0)
            robotCom.setTranslationSpeed(forwardspeed);
        else if((forwardspeed!=0 && rotationspeed!=0))
            robotCom.setArcSpeed(forwardspeed, forwardspeed/rotationspeed);
        else
            robotCom.setTranslationSpeed(0);
    }

    datacounter++;
    return 0;
}

int robot::processThisLidar(const std::vector<LaserData>& laserData)
{
    copyOfLaserData = laserData;
    emit publishLidar(copyOfLaserData);
    return 0;
}

#ifndef DISABLE_OPENCV
int robot::processThisCamera(cv::Mat cameraData)
{
    cameraData.copyTo(frame[(actIndex+1)%3]);
    actIndex = (actIndex+1)%3;
    emit publishCamera(frame[actIndex]);
    return 0;
}
#endif

#ifndef DISABLE_SKELETON
int robot::processThisSkeleton(skeleton skeledata)
{
    memcpy(&skeleJoints, &skeledata, sizeof(skeleton));
    emit publishSkeleton(skeleJoints);
    return 0;
}
#endif
