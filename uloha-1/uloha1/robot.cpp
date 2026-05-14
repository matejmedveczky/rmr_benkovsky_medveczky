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
     goal_final_x = 0.0;
     goal_final_y = 0.0;
     going_home_after_localization = false;
     nav_state = NavState::NAVIGATE_PATH;
     avoid_front_clear_cycles = 0;
     path_point = 0;
     path.clear();
 }

 void robot::returnHome(){
      setDesiredPosition(0.0, 0.0);
  }

 void robot::planPathTo(double x_goal, double y_goal, bool update_final_goal)
 {
     nav_state = NavState::NAVIGATE_PATH;
     avoid_front_clear_cycles = 0;

     if (update_final_goal) {
         goal_final_x = x_goal;
         goal_final_y = y_goal;
     }

     map.floodMap(x_goal, y_goal, x, y);

     path = map.calculatePath(x, y);
     path_point = 0;

     if (path.empty()) {
         this->x_des = x_goal;
         this->y_des = y_goal;
         cout << "No path found, direct target world (" << this->x_des << "," << this->y_des << ")" << endl;
         return;
     }

     int next_idx = path.size() > 1 ? 1 : 0;
     auto [next_x, next_y] = path[next_idx];

     this->x_des = next_x;
     this->y_des = next_y;

     cout << "First waypoint world (" << this->x_des << "," << this->y_des << ")" << endl;
 }

 bool robot::isNear(double tx, double ty, double tol) const
 {
     double dx = tx - x;
     double dy = ty - y;
     return std::sqrt(dx * dx + dy * dy) < tol;
 }

  void robot::setDesiredPosition(double x_des, double y_des)
  {
      planPathTo(x_des, y_des, true);
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

      bool mcl_active_for_motion = false;
      {
          std::lock_guard<std::mutex> lock(mclMutex);
          mcl_active_for_motion = mcl.isActive();
          if (mcl_active_for_motion) {
              mcl.motionUpdate(step_dist, delta_fi);
          }
      }

      if (!mcl_active_for_motion) {
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

      bool mcl_active = false;
      {
          std::lock_guard<std::mutex> lock(mclMutex);
          mcl_active = mcl.isActive();
      }

      if (mcl_active) {
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

         v = std::clamp(v, -80.0, 80.0);
         w = std::clamp(w, -0.4, 0.4);


      } else {
          int close_count = 0;
          int front_block_count = 0;
          float min_front = 2.5f, min_left = 2.5f, min_right = 2.5f;
          for (const auto& p : copyOfLaserData) {
              float d = p.scanDistance / 1000.0f;
              if (d < 0.05f || d > 2.5f) continue;
              float a = p.scanAngle;
              if      (a < 30  || a > 330) {
                  min_front = std::min(min_front, d);
                  if (d < 0.25f) close_count++;
                  if (d < 0.30f) front_block_count++;
              }
              else if (a >= 30 && a < 180) {
                  min_right = std::min(min_right, d);
              }
              else {
                  min_left = std::min(min_left, d);
              }
          }

          bool should_reinit_mcl = false;
          bool map_expects_wall = false;
          bool evaluated_reinit = false;
          bool cooldown_ready = false;
          int cycles_since_last_reinit = 0;
          {
              std::lock_guard<std::mutex> lock(mclMutex);
              if (!mcl.isActive()) {
                  evaluated_reinit = true;
                  // Check if close front lidar points correspond to map walls
                  // If any close point's hit cell is NOT a map wall -> unexpected obstacle
                  map_expects_wall = true;  // assume expected until proven otherwise
                  for (const auto& p : copyOfLaserData) {
                      float d = p.scanDistance / 1000.0f;
                      if (d < 0.05f || d > 0.35f) continue;  // only check close points
                      float a = p.scanAngle;
                      if (a > 30 && a < 330) continue;  // front arc only

                      // Compute hit position in world coordinates
                      double angle_rad = a * M_PI / 180.0;
                      double global_angle = fi - angle_rad;
                      double hit_x = x + d * std::cos(global_angle);
                      double hit_y = y + d * std::sin(global_angle);

                      int col, row;
                      map.worldToGrid(hit_x, hit_y, col, row);
                      bool is_map_wall = (col >= 0 && col < Map::GRID_SIZE && row >= 0 && row < Map::GRID_SIZE &&
                                          (map.grid[row][col] == Map::OCCUPIED || map.grid[row][col] == Map::BUFFER));
                      if (!is_map_wall) {
                          map_expects_wall = false;  // unexpected obstacle detected
                          break;
                      }
                  }

                  cycles_since_last_reinit = datacounter - last_mcl_reinit_datacounter;
                  cooldown_ready = (cycles_since_last_reinit >= MCL_REINIT_COOLDOWN_CYCLES);

                  if (close_count >= 5 && !map_expects_wall && cooldown_ready) {
                      mcl.init(map);
                      mcl_converged_streak = 0;
                      last_mcl_reinit_datacounter = datacounter;
                      should_reinit_mcl = true;
                  }
              }
          }

          if (should_reinit_mcl) {
              cout << "[MCL] Unexpected close obstacle - " << close_count
                   << " points under 25cm, map expects wall: NO - reinitializing\n";
          } else if (close_count >= 5 && evaluated_reinit) {
              cout << "[MCL] Reinit skipped - " << close_count
                   << " points under 25cm, map expects wall: " << (map_expects_wall ? "YES" : "NO")
                   << ", cooldown: " << cycles_since_last_reinit << "/" << MCL_REINIT_COOLDOWN_CYCLES
                   << " cycles\n";
          }

          bool front_blocked_strong = (front_block_count >= 5 && min_front < 0.35f);
          if (!going_home_after_localization && nav_state == NavState::NAVIGATE_PATH && front_blocked_strong && !map_expects_wall) {
              nav_state = NavState::AVOID_OBSTACLE;
              avoid_side = (min_left >= min_right) ? AvoidSide::LEFT : AvoidSide::RIGHT;
              avoid_front_clear_cycles = 0;
              cout << "[NAV] Entering AVOID_OBSTACLE: front blocked, map expects wall=NO, side="
                   << (avoid_side == AvoidSide::LEFT ? "LEFT" : "RIGHT") << "\n";
          }

          if (nav_state == NavState::AVOID_OBSTACLE) {
              bool front_too_close = (min_front < 0.28f || front_block_count >= 5);
              if (front_too_close) {
                  v = 0.0;
                  w = (avoid_side == AvoidSide::LEFT) ? -0.55 : 0.55;
              } else {
                  v = 90.0;
                  w = (avoid_side == AvoidSide::LEFT) ? -0.18 : 0.18;
              }

              if (min_front > 0.55f && close_count <= 1) {
                  avoid_front_clear_cycles++;
              } else {
                  avoid_front_clear_cycles = 0;
              }

              if (avoid_front_clear_cycles >= 8) {
                  nav_state = NavState::NAVIGATE_PATH;
                  avoid_front_clear_cycles = 0;
                  cout << "[NAV] Leaving AVOID_OBSTACLE: front clear, replanning to final goal ("
                       << goal_final_x << ", " << goal_final_y << ")\n";
                  setDesiredPosition(goal_final_x, goal_final_y);
              }
          } else {
              bool is_last_waypoint = path.empty() || path_point >= (int)path.size() - 1;
              double tolerance = is_last_waypoint ? 0.02 : 0.15;

              if (err_lin < tolerance) {
                  if (is_last_waypoint) {
                      v = 0; w = 0;
                  } else {
                      path_point++;
                      auto [next_x, next_y] = path[path_point];
                      this->x_des = next_x;
                      this->y_des = next_y;
                      cout << "Advancing to waypoint " << path_point
                           << ": (" << this->x_des << "," << this->y_des << ")\n";
                  }
              } else {
                  if (std::abs(err_ang) > 0.9) {
                      v = 0;
                      w = Kp_ang * err_ang;
                  } else {
                      v = Kp_lin * err_lin * 1000.0;
                      w = Kp_ang * err_ang;
                  }

                  if ((v - prev_v) > 5)         v = prev_v + 5;
                  if ((w - prev_w) > 0.05)      w = prev_w + 0.05;
                  else if ((w - prev_w) < -0.05) w = prev_w - 0.05;
              }
          }

          if (going_home_after_localization && isNear(0.0, 0.0, 0.08)) {
              going_home_after_localization = false;
              if (!isNear(goal_final_x, goal_final_y, 0.08)) {
                  cout << "[NAV] Home reached, switching to final goal ("
                       << goal_final_x << ", " << goal_final_y << ")\n";
                  planPathTo(goal_final_x, goal_final_y, false);
              }
          }

          v = std::clamp(v, -400.0, 400.0);
          w = std::clamp(w, -0.5, 0.5);
      }

      prev_v = v;
      prev_w = w;

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

      MCL::Pose pose{};
      double var = 0.0;
      int particle_count = 0;
      bool mcl_active = false;
      bool converged_now = false;
      bool pose_invalid = false;
      bool reached_convergence = false;

      {
          std::lock_guard<std::mutex> lock(mclMutex);

          if (mcl.isActive()) {
              mcl.weightUpdate(laserData);
              mcl.resample();

              // Task 2: use pose estimated from current-cycle post-resample state
              pose = mcl.estimatePose();

              // Pose from best particle — filter handles degeneracy internally.
              x = pose.x; y = pose.y; fi = pose.fi;

              converged_now = mcl.hasConverged();
              if (converged_now) {
                  mcl_converged_streak++;

                  if (mcl_converged_streak >= MCL_CONVERGE_REQUIRED) {
                      x = pose.x; y = pose.y; fi = pose.fi;
                      mcl.deactivate();
                      mcl_converged_streak = 0;
                      reached_convergence = true;
                  }
              } else {
                  mcl_converged_streak = 0;
              }
          } else {
              converged_now = false;
          }

          // Logging/GUI values must reflect the same post-update cycle state
          var = mcl.particleVariance();
          mcl_active = mcl.isActive();
          particle_count = mcl.particleCount();
      }

      if (pose_invalid) {
          cout << "[MCL] Best particle isolated — holding previous pose\n";
      }

      if (mcl_active && converged_now) {
          cout << "[MCL] Converge streak: " << mcl_converged_streak
               << "/" << MCL_CONVERGE_REQUIRED << "\n";
      }

      if (reached_convergence) {
          going_home_after_localization = true;
          cout << "[NAV] MCL converged, heading to home (0,0) before final goal\n";
          planPathTo(0.0, 0.0, false);
      }

      if (lidarcounter % 10 == 0 && mcl_active) {

          cout << "\n--- MCL ---";
          cout << "\nParticles:  " << particle_count << " | Variance: " << var;
          cout << "\nBest pose:  x=" << pose.x << " y=" << pose.y << " fi=" << pose.fi;
          cout << "\nConverged:  " << (converged_now ? "YES" : "NO")
               << " (streak " << mcl_converged_streak << "/" << MCL_CONVERGE_REQUIRED << ")"
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
