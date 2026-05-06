 #include "robot.h"
 #include <cmath>
 #include <cstring>
 #include <queue>

/*
 * Store pairs as {x, y}, access grid as grid[y][x].
*/

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
     emit publishMap(grid);
 }

 void robot::loadMap(const std::string& filename)
 {
     cout << "Loading Map\n";
     std::ifstream file(filename);
     if (!file.is_open()) return;

     for (int r = 0; r < GRID_SIZE; r++) {
         for (int c = 0; c < GRID_SIZE; c++) {
             file >> grid[r][c];
             if (grid[r][c] > 3){ grid[r][c] = FREE;}
         }
     }
     cout << "Map Loaded\n\n";
     file.close();
     emit publishMap(grid);
 }

 void robot::bufferMap()
 {
     // First pass: identify all obstacles
     vector<pair<int, int>> obstacles;
     for (int r = 0; r < GRID_SIZE; r++) {
         for (int c = 0; c < GRID_SIZE; c++) {
             if (grid[r][c] == OCCUPIED) {
                 obstacles.push_back({r, c});
             }
         }
     }

     // 8 directions: N, S, E, W, NE, NW, SE, SW
     int dr[] = {-1, 1, 0, 0, -1, -1, 1, 1};
     int dc[] = {0, 0, 1, -1, 1, -1, 1, -1};

     // Second pass: create buffers around each obstacle in all 8 directions
     for (auto [r, c] : obstacles) {
         for (int dir = 0; dir < 8; dir++) {
             for (int p = 1; p <= BUFFER_SIZE; p++) {
                 int nr = r + dr[dir] * p;
                 int nc = c + dc[dir] * p;

                 // Bounds check
                 if (nr < 0 || nr >= GRID_SIZE || nc < 0 || nc >= GRID_SIZE) {
                     break;  // Stop extending in this direction
                 }

                 // Only buffer FREE cells, don't overwrite obstacles
                 if (grid[nr][nc] == FREE) {
                     grid[nr][nc] = BUFFER;
                 } else if (grid[nr][nc] == OCCUPIED) {
                     break;  // Hit another obstacle, stop extending
                 }
                 // If already BUFFER, keep going
             }
         }
     }
 }

 void robot::floodMap(int x_des, int y_des, int x_start, int y_start){
     double x_offset = GRID_OFFSET_X * CELL_SIZE;
     double y_offset = GRID_OFFSET_Y * CELL_SIZE;

     int x_des_grid = (x_des + x_offset) / CELL_SIZE;
     int y_des_grid = (y_des + y_offset) / CELL_SIZE;

     int x_grid = (x + x_offset) / CELL_SIZE;
     int y_grid = (y + y_offset) / CELL_SIZE;

     for (int i = 0; i < GRID_SIZE; i++){
         for (int j = 0; j < GRID_SIZE; j++){
             if(grid[i][j] > BUFFER) {grid[i][j] = FREE;}
         }
     }

     bufferMap();

     queue<pair<int, int>> q;
     grid[y_des_grid][x_des_grid] = 4;
     q.push({x_des_grid, y_des_grid});

     while (!q.empty()) {
         auto [x, y] = q.front();  // FIXED: Call them x, y
         q.pop();

         int current_value = grid[y][x];  // FIXED: Access grid[y][x]

         int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
         int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

         for (int i = 0; i < 8; i++) {
             int nx = x + dx[i];
             int ny = y + dy[i];

             if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE) {
                 if (grid[ny][nx] == FREE) {  // FIXED: grid[y][x]
                     grid[ny][nx] = current_value + 1;
                     q.push({nx, ny});

                     if (nx == x_grid && ny == y_grid) {
                         return;
                     }
                 }
             }
         }
     }

     cout << "Grid value at start (" << x_grid << "," << y_grid << "): "
          << (int)grid[y_grid][x_grid] << endl;
     cout << "Grid value at dest (" << x_des_grid << "," << y_des_grid << "): "
          << (int)grid[y_des_grid][x_des_grid] << endl;

     if(grid[y_grid][x_grid] < 4) {
         cout << "ERROR: Start position not reachable! No path exists." << endl;
         return;
     }
 }

 vector<int> robot::occDir(int r, int c){
     vector<int> arr = {0, 0, 0, 0};

     if(grid[r - 1][c] == OCCUPIED){
         arr[0] = 1;
     }
     if(grid[r][c - 1] == OCCUPIED){
         arr[1] = 1;
     }
     if(grid[r + 1][c] == OCCUPIED){
         arr[2] = 1;
     }
     if(grid[r][c + 1] == OCCUPIED){
         arr[3] = 1;
     }
     return arr;
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
     floodMap(x_des, y_des, x, y);

     path = calculatePath(x, y); //x_des, y_des
     path_point = 0;

     int next_idx = path.size() > 1 ? 1 : 0;
     auto [next_x, next_y] = path[next_idx];

     this->x_des = next_x;
     this->y_des = next_y;

     cout << "First waypoint world (" << this->x_des << "," << this->y_des << ")" << endl;

 }

 vector<pair<double, double>> robot::calculatePath(double start_x_world, double start_y_world){
     double x_offset = GRID_OFFSET_X * CELL_SIZE;
     double y_offset = GRID_OFFSET_Y * CELL_SIZE;

     int start_x = (start_x_world + x_offset) / CELL_SIZE;
     int start_y = (start_y_world + y_offset) / CELL_SIZE;

     vector<pair<int, int>> grid_path;
     grid_path.push_back({start_x, start_y});  // {x, y}

     while(grid[start_y][start_x] != 4){  // Access: grid[y][x]
         int min_value = grid[start_y][start_x];
         int next_x = start_x;
         int next_y = start_y;

         int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
         int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

         for(int i = 0; i < 8; i++){
             int nx = start_x + dx[i];
             int ny = start_y + dy[i];

             if(nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE){
                 if(grid[ny][nx] < min_value && grid[ny][nx] >= 4){
                     min_value = grid[ny][nx];
                     next_x = nx;
                     next_y = ny;
                 }
             }
         }

         start_x = next_x;
         start_y = next_y;
         grid_path.push_back({start_x, start_y});  // FIXED: {x, y}

         if(grid_path.size() > GRID_SIZE * GRID_SIZE){
             cout << "No path found!" << endl;
             break;
         }
     }

     cout << "Full grid path (" << grid_path.size() << " points):" << endl;
     for(int i = 0; i < min(20, (int)grid_path.size()); i++) {
         cout << "[" << grid_path[i].first << "," << grid_path[i].second << "] ";
     }
     cout << "\n...\n";
     for(int i = max(0, (int)grid_path.size() - 5); i < grid_path.size(); i++) {
         cout << "[" << grid_path[i].first << "," << grid_path[i].second << "] ";
     }
     cout << endl;

     vector<pair<double, double>> world_path;
     world_path.push_back({(grid_path[0].first - GRID_OFFSET_X) * CELL_SIZE,
                           (grid_path[0].second - GRID_OFFSET_Y) * CELL_SIZE});

     for(int i = 1; i < grid_path.size() - 1; i++){
         int dx_prev = grid_path[i].first - grid_path[i-1].first;
         int dy_prev = grid_path[i].second - grid_path[i-1].second;

         int dx_next = grid_path[i+1].first - grid_path[i].first;
         int dy_next = grid_path[i+1].second - grid_path[i].second;

         if(dx_prev != dx_next || dy_prev != dy_next){
             world_path.push_back({(grid_path[i].first - GRID_OFFSET_X) * CELL_SIZE,
                                   (grid_path[i].second - GRID_OFFSET_Y) * CELL_SIZE});
         }
     }

     world_path.push_back({(grid_path.back().first - GRID_OFFSET_X) * CELL_SIZE,
                           (grid_path.back().second - GRID_OFFSET_Y) * CELL_SIZE});

     cout << "Simplified path (" << world_path.size() << " waypoints):" << endl;
     for(auto [wx, wy] : world_path) {
         cout << "(" << wx << "," << wy << ") ";
     }
     cout << endl;

     return world_path;
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

     PoseStamp ps = {x, y, fi, robotdata.synctimestamp};
     poseHistory.push_back(ps);
     if (poseHistory.size() > 200)
         poseHistory.pop_front();

     double dx      = x_des - x;
     double dy      = y_des - y;
     double err_lin = sqrt(dx*dx + dy*dy);
     double err_ang = atan2(dy, dx) - fi;

     while(err_ang >  M_PI) err_ang -= 2*M_PI;
     while(err_ang < -M_PI) err_ang += 2*M_PI;

     double Kp_lin = 1; // 0.3
     double Kp_ang = 1.2;

     double v = 0, w = 0;


     if(err_lin < 0.02) {
         if(path.empty() || path_point >= path.size() - 1) {
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

     v = std::clamp(v, -400.0, 400.0);
     w = std::clamp(w, -0.5, 0.5);

     setSpeedVal(v, w);

     if(datacounter % 5 == 0)
     {
         cout << "\nRobot pos x/y/angle: " << x << " " << y << " " << fi;
         cout << "\nGyro raw/delta_deg:  " << gyro_now << " " << delta_deg;
         cout << "\nErrors lin/ang:      " << err_lin  << " " << err_ang;
         cout << "\nCommands v/w:        " << v        << " " << w<<std::endl;
         cout << "\nDesired position     " << x_des    << " " << y_des;
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

     if (poseHistory.empty()) return 0;

     // qDebug() << "Lidar called, poses:" << poseHistory.size()
     //          << "points:" << copyOfLaserData.size();

     for (const auto& point : copyOfLaserData) {
         double dist_m = point.scanDistance / 1000.0;
         if (dist_m < 0.05 || dist_m > 2.5 || (dist_m > 0.5 && dist_m < 0.7)) continue;
         PoseStamp pose = interpolatePose(point.timestamp);
         updateGrid(point, pose);
     }

     int occupiedCount = 0;
     for (int r = 0; r < GRID_SIZE; r++)
         for (int c = 0; c < GRID_SIZE; c++)
             if (grid[r][c] == OCCUPIED) occupiedCount++;

     // qDebug() << "Occupied cells:" << occupiedCount;

     emit publishMap(grid);
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
             if (grid[r0][c0] == UNKNOWN) //grid[r0][c0] != OCCUPIED || grid[r0][c0] != BUFFER
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
     double x_gi = pose.x + dist_m * std::cos(global_angle);
     double y_gi = pose.y + dist_m * std::sin(global_angle);

     int col_r, row_r, col_h, row_h;
     worldToGrid(pose.x, pose.y, col_r, row_r);
     worldToGrid(x_gi,   y_gi,   col_h, row_h);

     bresenham(col_r, row_r, col_h, row_h);

     if (col_h >= 0 && col_h < GRID_SIZE && row_h >= 0 && row_h < GRID_SIZE)
         grid[row_h][col_h] = OCCUPIED;
 }
