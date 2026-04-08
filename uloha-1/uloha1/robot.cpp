 #include "robot.h"
 #include <cmath>
 #include <cstring>

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

 void robot::saveMap(const std::string& filename)
 {
     cout << "Saving Map";
     std::ofstream file(filename);
     if (!file.is_open()) return;

     for (int r = 0; r < GRID_SIZE; r++) {
         for (int c = 0; c < GRID_SIZE; c++) {
             file << grid[r][c];
             if (c < GRID_SIZE - 1) file << " ";
         }
         file << "\n";
     }
     cout << "Map Saved";
     file.close();
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
     x_des = 0.0;
     y_des = 0.0;
 }

 void robot::returnHome(){
     x_des = 0.0;
     y_des = 0.0;
 }

void robot::setDesiredPosition(double xDes, double yDes)
{
    x_des = xDes;
    y_des = yDes;
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

     PoseStamp ps = {x, y, fi, robotdata.timestamp};
     poseHistory.push_back(ps);
     if (poseHistory.size() > 200)
         poseHistory.pop_front();

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

     prev_v = v;
     prev_w = w;

     v = std::clamp(v,  -400.0, 400.0);
     w = std::clamp(w, -2.0,   2.0);

     setSpeedVal(v, w);

     if(datacounter % 5 == 0)
     {
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
     copyOfLaserData = laserData;
     emit publishLidar(copyOfLaserData);

     if (poseHistory.empty()) return 0;

     qDebug() << "Lidar called, poses:" << poseHistory.size()
              << "points:" << laserData.size();

     for (const auto& point : laserData) {
         double dist_m = point.scanDistance / 1000.0;
         if (dist_m < 0.05 || dist_m > 5.0) continue;
         PoseStamp pose = interpolatePose(point.timestamp);
         updateGrid(point, pose);
     }

     int occupiedCount = 0;
     for (int r = 0; r < GRID_SIZE; r++)
         for (int c = 0; c < GRID_SIZE; c++)
             if (grid[r][c] == OCCUPIED) occupiedCount++;

     qDebug() << "Occupied cells:" << occupiedCount;

     emit publishMap(grid);
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

 // Occupancy grid mapping methods

 void robot::worldToGrid(double wx, double wy, int &col, int &row)
 {
     col = (int)std::floor((wx - gridOriginX) / CELL_SIZE);
     row = (int)std::floor((wy - gridOriginY) / CELL_SIZE);
 }

 robot::PoseStamp robot::interpolatePose(unsigned ts)
 {
     if (poseHistory.empty()) return {x, y, fi, ts};
     if (ts <= poseHistory.front().timestamp) return poseHistory.front();
     if (ts >= poseHistory.back().timestamp)  return poseHistory.back();

     for (size_t i = 1; i < poseHistory.size(); i++) {
         if (poseHistory[i].timestamp >= ts) {
             PoseStamp a = poseHistory[i-1];
             PoseStamp b = poseHistory[i];
             double ratio = (double)(ts - a.timestamp) / (double)(b.timestamp - a.timestamp);
             double dfi = b.fi - a.fi;
             while (dfi >  M_PI) dfi -= 2*M_PI;
             while (dfi < -M_PI) dfi += 2*M_PI;
             return { a.x + ratio*(b.x - a.x),
                      a.y + ratio*(b.y - a.y),
                      a.fi + ratio*dfi,
                      ts };
         }
     }
     return poseHistory.back();
 }

 void robot::bresenham(int c0, int r0, int c1, int r1)
 {
     int dc = std::abs(c1-c0), dr = std::abs(r1-r0);
     int sc = (c0 < c1) ? 1 : -1;
     int sr = (r0 < r1) ? 1 : -1;
     int err = dc - dr;

     while (true) {
         if (c0 == c1 && r0 == r1) break;
         if (c0 >= 0 && c0 < GRID_SIZE && r0 >= 0 && r0 < GRID_SIZE)
             if (grid[r0][c0] != OCCUPIED)
                 grid[r0][c0] = FREE;
         int e2 = 2 * err;
         if (e2 > -dr) { err -= dr; c0 += sc; }
         if (e2 <  dc) { err += dc; r0 += sr; }
     }
 }

 void robot::updateGrid(const LaserData& point, const PoseStamp& pose)
 {
     double dist_m = point.scanDistance / 1000.0;
     double global_angle = pose.fi - (point.scanAngle * M_PI / 180.0);
     double x_gi = pose.x + dist_m * std::cos(global_angle);  // ← was point.scanDistance
     double y_gi = pose.y + dist_m * std::sin(global_angle);  // ← was point.scanDistance

     int col_r, row_r, col_h, row_h;
     worldToGrid(pose.x, pose.y, col_r, row_r);
     worldToGrid(x_gi,   y_gi,   col_h, row_h);

     bresenham(col_r, row_r, col_h, row_h);

     if (col_h >= 0 && col_h < GRID_SIZE && row_h >= 0 && row_h < GRID_SIZE)
         grid[row_h][col_h] = OCCUPIED;
 }
