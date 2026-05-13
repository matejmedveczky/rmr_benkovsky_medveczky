#ifndef ROBOT_H
#define ROBOT_H
#include "librobot/librobot.h"
#include <QObject>
#include <QWidget>
#include <deque>
#include <cmath>
#include <fstream>
#include "map.h"
#include "mcl.h"

#ifndef DISABLE_OPENCV
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

Q_DECLARE_METATYPE(cv::Mat)
#endif
#ifndef DISABLE_SKELETON
Q_DECLARE_METATYPE(skeleton)
#endif
Q_DECLARE_METATYPE(std::vector<LaserData>)
Q_DECLARE_METATYPE(int[280][280])
class robot : public QObject {
  Q_OBJECT
 public:
  explicit robot(QObject *parent = nullptr);

  void initAndStartRobot(std::string ipaddress);

  // tato funkcia len nastavuje hodnoty.. posielaju sa v callbacku(dobre, kvoli
  // asynchronnosti a zabezpeceniu,ze sa poslu len raz pri viacero prepisoch
  // vramci callu)
  void setSpeedVal(double forw, double rots);
  // tato funkcia fyzicky posiela hodnoty do robota
  void setSpeed(double forw, double rots);


  void resetRobot();
  void returnHome();
  void saveMap(const std::string& filename);
  void loadMap(const std::string& filename);
  void setDesiredPosition(double xDes, double yDes);
 signals:
  void publishPosition(double x, double y, double z);
  void publishLidar(const std::vector<LaserData> &lidata);
  void publishMap(int grid[280][280]);
  void publishVariance(double var);
#ifndef DISABLE_OPENCV
  void publishCamera(const cv::Mat &camframe);
#endif
#ifndef DISABLE_SKELETON
  void publishSkeleton(const skeleton &skeledata);
#endif
 private:
  /// toto su vase premenne na vasu odometriu
  double x = 0;
  double y = 0;
  double fi = 0;
  vector<pair<double, double>> path;

  bool first_tick = true;
  double old_left_encoder = 0.0;
  double old_right_encoder = 0.0;
  double angle_old = 0.0;
  unsigned old_timestamp = 0.0;

  double prev_v = 0.0;
  double prev_w = 0.0;
  ///-----------------------------
  /// toto su rychlosti ktore sa nastavuju setSpeedVal a posielaju v
  /// processThisRobot
  double forwardspeed;  // mm/s
  double rotationspeed; // omega/s

  double x_des;
  double y_des;
  int goal_grid_x;
  int goal_grid_y;
  double err_lin_prev = 0, err_ang_prev = 0;
  int path_point;

  Map map;
  MCL mcl;

  double max_v_dt = 400/2;
  // double integral_lin = 0, integral_ang = 0;

  /// toto su callbacky co sa sa volaju s novymi datami
  int processThisLidar(const std::vector<LaserData> &laserData);
  int processThisRobot(const TKobukiData &robotdata);
#ifndef DISABLE_OPENCV
  int processThisCamera(cv::Mat cameraData);
#endif

  /// pomocne strukutry aby ste si trosku nerobili race conditions
  std::vector<LaserData> copyOfLaserData;
#ifndef DISABLE_OPENCV
  cv::Mat frame[3];
#endif
  /// classa ktora riesi komunikaciu s robotom
  libRobot robotCom;

  /// pomocne premenne... moc nerieste naco su
  int datacounter;
  int lidarcounter;
  int mcl_converged_streak = 0;
  static const int MCL_CONVERGE_REQUIRED = 10;  // must be converged for 10 consecutive LIDAR scans
#ifndef DISABLE_OPENCV
  bool useCamera1;
  int actIndex;
#endif

#ifndef DISABLE_SKELETON
  int processThisSkeleton(skeleton skeledata);
  int updateSkeletonPicture;
  skeleton skeleJoints;
#endif
  int useDirectCommands;

 public:
  enum CellState { UNKNOWN = 0, FREE = 1, OCCUPIED = 2, BUFFER = 3};

  // Pose history for laser interpolation
  struct PoseStamp { double x, y, fi; unsigned timestamp; };
  std::deque<PoseStamp> poseHistory;

  // Occupancy grid
  //static const int    GRID_SIZE = 280;
  //static constexpr double CELL_SIZE = 0.05;

  //static const int GRID_OFFSET_X = 140;
  //static const int GRID_OFFSET_Y = 140;

  //int grid[GRID_SIZE][GRID_SIZE];

  //static const int BUFFER_SIZE = 5;

  static constexpr float L_HIT =  0.85f;
  static constexpr float L_FREE  =  0.40f;
  static constexpr float L_MIN   = -10.0f;
  static constexpr float L_MAX   =  10.0f;
  static constexpr float THRESHOLD_OCC  =  0.5f;
  static constexpr float THRESHOLD_FREE = -0.5f;

  //double gridOriginX = -(GRID_SIZE * CELL_SIZE / 2.0);
  //double gridOriginY = -(GRID_SIZE * CELL_SIZE / 2.0);
};

#endif // ROBOT_H
