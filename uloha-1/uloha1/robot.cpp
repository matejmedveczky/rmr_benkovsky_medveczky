 #include "robot.h"
 #include <cmath>
 #include <cstring>
 #include <queue>
 #include "map.h"
 #include "mcl.h"



/*
 * Store pairs as {x, y}, access grid as grid[y][x].
*/

 robot::robot(QObject *parent) : QObject(parent)
 {
     cout << "[Robot] Constructor start\n" << std::flush;
     qRegisterMetaType<LaserMeasurement>("LaserMeasurement");
     #ifndef DISABLE_OPENCV
     qRegisterMetaType<cv::Mat>("cv::Mat");
     #endif
     #ifndef DISABLE_SKELETON
     qRegisterMetaType<skeleton>("skeleton");
     #endif
     cout << "[Robot] Constructor done\n" << std::flush;
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

     x_des = 0;
     y_des = 0;

     memset(grid, 0, sizeof(grid));

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



 void robot::loadMap(const std::string& filename)
 {
     cout << "Loading Map\n";
     map.loadMap(filename);
     emit publishMap(map.grid);
     mcl.init(map);
     cout << "Map Loaded\n\n";
 }

 void robot::saveMap(const std::string& filename)
 {
     cout << "Saving Map\n";
     map.saveMap(filename);
     emit publishMap(map.grid);
     cout << "Map saved\n\n";
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
     // x = 0;
     // y = 0;
     // fi = 0;
     x_des = 0.0;
     y_des = 0.0;
     path_point = 0;
     path.clear();
 }

 void robot::returnHome(){
     setDesiredPosition(0.0, 0.0);
 }

 void robot::setDesiredPosition(double x_des, double y_des)
 {
     map.floodMap(x_des, y_des, x, y);

     path = map.calculatePath(x, y); //x_des, y_des
     path_point = 0;

     int next_idx = path.size() > 1 ? 1 : 0;
     auto [next_x, next_y] = path[next_idx];

     this->x_des = next_x;
     this->y_des = next_y;

     cout << "First waypoint world (" << this->x_des << "," << this->y_des << ")" << endl;

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

     if(delta_left  >  32767) delta_left  -= 65536;
     if(delta_left  < -32767) delta_left  += 65536;
     if(delta_right >  32767) delta_right -= 65536;
     if(delta_right < -32767) delta_right += 65536;

     double left_distance  = tick * delta_left;
     double right_distance = tick * delta_right;

     old_left_encoder  = robotdata.EncoderLeft;
     old_right_encoder = robotdata.EncoderRight;

     double gyro_now  = robotdata.GyroAngle / 100.0;
     double delta_deg = gyro_now - angle_old;
     angle_old        = gyro_now;

     while(delta_deg >  180.0) delta_deg -= 360.0;
     while(delta_deg < -180.0) delta_deg += 360.0;

     double delta_fi  = delta_deg * M_PI / 180.0;
     double step_dist = (left_distance + right_distance) / 2.0;

     Map::PoseStamp ps = {x, y, fi, robotdata.synctimestamp};
     map.addPose(ps);

     if (mcl.isActive()) {
         mcl.motionUpdate(step_dist, delta_fi);
         auto pose = mcl.estimatePose();
         x = pose.x;  y = pose.y;  fi = pose.fi;
     } else {
         fi += delta_fi;
         while(fi >  M_PI) fi -= 2*M_PI;
         while(fi < -M_PI) fi += 2*M_PI;
         x += step_dist * std::cos(fi);
         y += step_dist * std::sin(fi);
     }

     double dx      = x_des - x;
     double dy      = y_des - y;
     double err_lin = sqrt(dx*dx + dy*dy);
     double err_ang = atan2(dy, dx) - fi;


     while(err_ang >  M_PI) err_ang -= 2*M_PI;
     while(err_ang < -M_PI) err_ang += 2*M_PI;

     double Kp_lin = 1; // 0.3
     double Kp_ang = 1.2;

     double v = 0, w = 0;


     bool is_last_waypoint = path.empty() || path_point >= path.size() - 1;
     double tolerance = is_last_waypoint ? 0.02 : 0.15;

     if(err_lin < tolerance) {
         if(is_last_waypoint) {
             v = 0; w = 0;
         } else {
             path_point++;
             auto [next_grid_x, next_grid_y] = path[path_point];

             this->x_des = next_grid_x;
             this->y_des = next_grid_y;

             cout << "Advancing to waypoint " << path_point
                  << ": grid[" << next_grid_x << "," << next_grid_y
                  << "] = world(" << this->x_des << "," << this->y_des << ")" << endl;
         }
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

         if ((w - prev_w) > 0.05){
             w = prev_w + 0.05;
         }
         else if ((w - prev_w) < -0.05){
             w = prev_w - 0.05;
         }
     }

     prev_v = v;
     prev_w = w;

     if (mcl.isActive()) {
         double var = mcl.particleVariance();
         double confidence = std::exp(-var / MCL::VAR_HIGH);
         v = v * confidence;
         v = std::clamp(v, -80.0, 80.0);
         w = std::clamp(w, -0.2, 0.2);
     }
     else {
         v = std::clamp(v, -400.0, 400.0);
         w = std::clamp(w, -0.5, 0.5);
     }

     if (!copyOfLaserData.empty() && mcl.isActive()) {
         // Check forward cone (±30°) and sides
         float min_front = 2.5f, min_left = 2.5f, min_right = 2.5f;
         for (const auto& p : copyOfLaserData) {
             float d = p.scanDistance / 1000.0f;
             if (d < 0.05f || d > 2.5f) continue;
             float a = p.scanAngle;
             if      (a < 30  || a > 330) min_front = std::min(min_front, d);
             else if (a >= 30 && a < 180) min_left  = std::min(min_left,  d);
             else                          min_right = std::min(min_right, d);
         }

         const float STOP_DIST  = 0.40f;  // m — hard stop
         const float SLOW_DIST  = 0.80f;  // m — start slowing

         if (min_front < STOP_DIST) {
             v = 0;
             w = (min_left > min_right) ? 0.4 : -0.4;
         } else if (min_front < SLOW_DIST) {
             v *= (min_front - STOP_DIST) / (SLOW_DIST - STOP_DIST);
         }
     }

     setSpeedVal(v, w);

     if(datacounter % 50 == 0)
     {
         cout << "\nDesired position     " << x_des    << " " << y_des;
         cout << "\nRobot pos x/y/angle: " << x << " " << y << " " << fi;
         cout << "\nGyro raw/delta_deg:  " << gyro_now << " " << delta_deg;
         cout << "\nErrors lin/ang:      " << err_lin  << " " << err_ang;
         cout << "\nCommands v/w:        " << v        << " " << w<<std::endl;
         emit publishPosition(x, y, fi);
     }

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
     //map.processLidarScan(copyOfLaserData);
     emit publishMap(map.grid);

     auto pose = mcl.estimatePose();

     if (mcl.isActive()) {
         mcl.weightUpdate(laserData);
         mcl.resample();

         x = pose.x;  y = pose.y;  fi = pose.fi;
         if (mcl.hasConverged()) {
             mcl_converged_streak++;
             cout << "[MCL] Converge streak: " << mcl_converged_streak
                  << "/" << MCL_CONVERGE_REQUIRED << "\n";

             if (mcl_converged_streak >= MCL_CONVERGE_REQUIRED) {
                 auto pose = mcl.estimatePose();
                 x = pose.x;  y = pose.y;  fi = pose.fi;
                 mcl.deactivate();
                 mcl_converged_streak = 0;
                 setDesiredPosition(0.0, 0.0);
             }
         } else {
             mcl_converged_streak = 0;
         }
     }

     double var = mcl.particleVariance();

     if (lidarcounter % 10 == 0 && mcl.isActive()) {

         cout << "\n--- MCL ---";
         cout << "\nParticles:  " << mcl.particleCount() << " | Variance: " << var;
         cout << "\nBest pose:  x=" << pose.x << " y=" << pose.y << " fi=" << pose.fi;
         cout << "\nConverged:  " << (var < MCL::VAR_CONVERGED ? "YES" : "NO")
              << " | Dist to start: "
              << std::sqrt(std::pow(x_des - pose.x, 2) + std::pow(y_des - pose.y, 2));
         cout << "\n-----------\n" << std::endl;

     }

     emit publishVariance(var);
     copyOfLaserData = laserData;
     emit publishLidar(copyOfLaserData);


     lidarcounter++;
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
