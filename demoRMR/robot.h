#ifndef ROBOT_H
#define ROBOT_H
#include "librobot/librobot.h"
#include <QObject>
#include <QWidget>

#include <vector>
#include <utility>

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
Q_DECLARE_METATYPE(std::vector<int>)
class robot : public QObject {
  Q_OBJECT
public:
  explicit robot(QObject *parent = nullptr);

  void initAndStartRobot(std::string ipaddress);

  // tato funkcia len nastavuje hodnoty.. posielaju sa v callbacku(dobre, kvolistd::vector<float>
  // asynchronnosti a zabezpeceniu,ze sa poslu len raz pri viacero prepisoch
  // vramci callu)
  void setSpeedVal(double forw, double rots);
  // tato funkcia fyzicky posiela hodnoty do robota
  void setSpeed(double forw, double rots);
  void resetRobot();
  void returnHome();
  void setGoal(double xg, double yg);
  void stopRobot();
  void resumeRobot();
signals:
  void publishPosition(double x, double y, double z);
  void publishLidar(const std::vector<LaserData> &lidata);
  void publishHistogram(const std::vector<int> &binaryHist,
                        const std::vector<int> &maskedHist);
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

  bool first_tick = true;
  double old_left_encoder = 0.0;
  double old_right_encoder = 0.0;
  double angle_old = 0.0;
  unsigned old_timestamp = 0.0;

  double prev_v = 0.0;
  double prev_w = 0.0;

  // Nastavi sa na true po stlaceni tlacidla STOP.
  // Kym je true, VFH ani regulator nesmu znovu rozbehnut robota.
  bool stoppedByButton = false;
  ///-----------------------------
  /// toto su rychlosti ktore sa nastavuju setSpeedVal a posielaju v
  /// processThisRobot
  double forwardspeed;  // mm/s
  double rotationspeed; // omega/s

  double x_des = 0.0;
  double y_des = 0.0;
  double err_lin_prev = 0, err_ang_prev = 0;

  double x_target = 0.0;
  double y_target = 0.0;

  double max_v_dt = 400/2;
  // double integral_lin = 0, integral_ang = 0;

  double robotRadius = 0.18;      // približne polovica šírky robota + rezerva [m]
  double safetyMargin = 0.05;     // bezpečnostná rezerva [m]

  /// uloha 2 ///

  static constexpr int sectorCount = 36;

  std::vector<std::pair<int,int>> freeGaps;
  std::vector<float> primaryHistogram;
  std::vector<int> binaryHistogram;
  std::vector<int> maskedHistogram;

  float a_hist = 1.0f;
  float b_hist = 0.6f;
  float robotSafetyRadius = 0.2f;
  float histogramThreshold = 5.0f;

  float finalApproachDistance = 0.80f;
  float wallBehindTargetMargin = 0.06f;

  int previousBestSector = -1;

  float normalizeLaserAngle(float rawAngleDeg);
  float normalizeAngle360(float angleDeg);
  float normalizeAngle180(float angleDeg);
  float laserDistanceToMeters(float rawDistance);
  float minDistanceInAngleRange(float fromDeg, float toDeg);
  bool canIgnoreVFHNearGoal(float signedGoalAngleDeg, double distToTarget);

  void updatePrimaryHistogram();
  void updateBinaryHistogram(float threshold);
  void updateMaskedHistogram();
  void findFreeGaps();

  int goalSector(float goalAngleDeg);
  int circularSectorDiff(int a, int b);
  int chooseBestSector(float goalAngleDeg);
  float candidateCost(int candidateSector, int goalSectorIdx);

  double sectorToAngle(int sectorIndex);
  void updateWaypointFromSector(int bestSector);
  void updateVFHNavigation(float goalAngleDeg, double distToTarget);

  /// uloha 2 ///

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
};

#endif // ROBOT_H
