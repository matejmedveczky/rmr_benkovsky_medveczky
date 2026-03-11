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
    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robotCom.setLaserParameters([this](const std::vector<LaserData>& dat)->int{return processThisLidar(dat);},ipaddress);
    robotCom.setRobotParameters([this](const TKobukiData& dat)->int{return processThisRobot(dat);},ipaddress);
  #ifndef DISABLE_OPENCV
    robotCom.setCameraParameters(std::bind(&robot::processThisCamera,this,std::placeholders::_1),"http://"+ipaddress+":8000/stream.mjpg");
#endif
   #ifndef DISABLE_SKELETON
      robotCom.setSkeletonParameters(std::bind(&robot::processThisSkeleton,this,std::placeholders::_1));
#endif
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
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

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int robot::processThisRobot(const TKobukiData &robotdata)
{
    double wheelbase = 0.23;
    long double tick = robotCom.getTickToMeter();

    static double x_old = 0;
    static double y_old = 0;
    static double angle_old = 0;
    static double old_left_encoder = 0;
    static double old_right_encoder = 0;

    double left_distance = tick*(robotdata.EncoderLeft - old_left_encoder);
    double right_distance = tick*(robotdata.EncoderRight - old_right_encoder);

    old_left_encoder  = robotdata.EncoderLeft;
    old_right_encoder = robotdata.EncoderRight;

    double gyro_angle = robotdata.GyroAngle;
    double angle_change = gyro_angle - angle_old;
    angle_old = gyro_angle;

    //double step_dist = (wheelbase/2)*((left_distance+right_distance)/(left_distance-right_distance));

    double step_dist = (left_distance + right_distance) / 2.0;

    //double x_new = x_old + step_dist * sin(gyro_angle) - sin(angle_old);
    //double y_new = y_old - step_dist * cos(gyro_angle) - cos(angle_old);

    //double x_new = x_old + step_dist * cos(gyro_angle);
    //double y_new = y_old + step_dist * sin(gyro_angle);

    //x_old = x_new;
    //y_old = y_new;
    //angle_old = gyro_angle;

    ///tu mozete robit s datami z robota
    ///
    /// max speed 400mm/s ideal 200mm/s

    // --- fix odometry ---
    double angle_rad = gyro_angle * M_PI / 180.0;
    x += step_dist * cos(angle_rad);
    y += step_dist * sin(angle_rad);
    fi  = angle_rad;

    // --- errors ---
    double dx = x_des - x;
    double dy = y_des - y;
    double err_lin = sqrt(dx*dx + dy*dy);
    double err_ang = atan2(dy, dx) - fi;

    // normalize angle to [-pi, pi]
    while(err_ang >  M_PI) err_ang -= 2*M_PI;
    while(err_ang < -M_PI) err_ang += 2*M_PI;

    // --- PID (dt is implicit — callback fires at fixed rate) ---
    integral_lin += err_lin;
    integral_ang += err_ang;

    double Kp_lin=0.3,  Ki_lin=0.0001, Kd_lin=0.01;
    double Kp_ang=1.2,  Ki_ang=0.0,    Kd_ang=0.01;

    // double Kp_lin=2, Ki_lin=0.1, Kd_lin=0.5;

    double v = Kp_lin*err_lin + Ki_lin*integral_lin + Kd_lin*(err_lin - err_lin_prev);
    double w = Kp_ang*err_ang + Ki_ang*integral_ang + Kd_ang*(err_ang - err_ang_prev);

    // couple them — slow down if misaligned
    v *= std::max(0.0, cos(err_ang));

    err_lin_prev = err_lin;
    err_ang_prev = err_ang;

    // stop when close enough
    if(err_lin < 0.02) v = 0, w = 0; // 20mm tolerance

    v = std::clamp(v * 1000.0, 0.0, 200.0); // mm/s, conservative max
    w = std::clamp(w, -180.0, 180.0);              // deg/s
    setSpeedVal(v, w);

    setSpeedVal(v, w); // plugs into existing dispatch


///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    ///kazdy piaty krat, aby to ui moc nepreblikavalo..
    if(datacounter%5==0)
    {
        cout << "\nRobot pos x/y/dist: ";
        cout << x << " ";
        cout << y << " ";
        cout << step_dist;
        cout << "\nCommands v/w: ";
        cout << v << " ";
        cout << w << " ";
        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
        // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
        //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
        //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
        /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
        /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
        /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        emit publishPosition(robotdata.EncoderLeft,y,fi);
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    ///---tu sa posielaju rychlosti do robota... vklude zakomentujte ak si chcete spravit svoje
    if(useDirectCommands==0)
    {
        if(forwardspeed==0 && rotationspeed!=0)
            robotCom.setRotationSpeed(rotationspeed);
        else if(forwardspeed!=0 && rotationspeed==0)
            robotCom.setTranslationSpeed(forwardspeed);
        else if((forwardspeed!=0 && rotationspeed!=0))
            robotCom.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
        else
            robotCom.setTranslationSpeed(0);
    }
    datacounter++;

    return 0;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z lidaru
int robot::processThisLidar(const std::vector<LaserData>& laserData)
{

    copyOfLaserData=laserData;

    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;
    emit publishLidar(copyOfLaserData);
   // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}

  #ifndef DISABLE_OPENCV
///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z kamery
int robot::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka

    emit publishCamera(frame[actIndex]);
    return 0;
}
#endif

  #ifndef DISABLE_SKELETON
/// vola sa ked dojdu nove data z trackera
int robot::processThisSkeleton(skeleton skeledata)
{

    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    emit publishSkeleton(skeleJoints);
    return 0;
}
#endif
