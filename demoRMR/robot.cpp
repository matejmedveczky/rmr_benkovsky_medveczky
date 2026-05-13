 #include "robot.h"
#include <cmath>
#include <algorithm>

robot::robot(QObject *parent) : QObject(parent)
{
    qRegisterMetaType<LaserMeasurement>("LaserMeasurement");
    qRegisterMetaType<std::vector<int>>("std::vector<int>");
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

    x_des = 0.0;
    y_des = 0.0;
    x_target = 0.0;
    y_target = 0.0;

    prev_v = 0.0;
    prev_w = 0.0;
    stoppedByButton = false;


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
    stoppedByButton = false;

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

void robot::returnHome()
{
    stoppedByButton = false;

    x_target = 0.0;
    y_target = 0.0;

    x_des = 0.0;
    y_des = 0.0;
}

void robot::setGoal(double xg, double yg)
{
    stoppedByButton = false;

    x_target = xg;
    y_target = yg;

    x_des = xg;
    y_des = yg;

    std::cout << "Novy ciel nastaveny: x_target = "
              << x_target << ", y_target = " << y_target << std::endl;
}

int robot::processThisRobot(const TKobukiData &robotdata)
{
    long double tick = robotCom.getTickToMeter();

    if(first_tick) {
        old_left_encoder  = robotdata.EncoderLeft;
        old_right_encoder = robotdata.EncoderRight;
        angle_old         = robotdata.GyroAngle/100;
        old_timestamp     = robotdata.timestamp;
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
    old_timestamp     = robotdata.timestamp;

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

    if(stoppedByButton)
    {
        forwardspeed = 0;
        rotationspeed = 0;
        prev_v = 0;
        prev_w = 0;

        x_des = x;
        y_des = y;

        robotCom.setTranslationSpeed(0);
        robotCom.setRotationSpeed(0);

        if(datacounter % 5 == 0)
        {
            emit publishPosition(x, y, fi);
        }

        datacounter++;
        return 0;
    }

    double dx      = x_des - x;
    double dy      = y_des - y;
    double err_lin = sqrt(dx*dx + dy*dy);
    double err_ang = atan2(dy, dx) - fi;

    while(err_ang >  M_PI) err_ang -= 2*M_PI;
    while(err_ang < -M_PI) err_ang += 2*M_PI;

    double Kp_lin = 0.8;
    double Kp_ang = 1.0;

/*
    double Kp_lin = 1.0;
    double Kp_ang = 1.2;
*/
    double v = 0.0;
    double w = 0.0;

    if(err_lin < 0.02) {
        v = 0.0;
        w = 0.0;
    } else {
        if(std::abs(err_ang) > 0.6) {
            // Najprv sa natocime na docasny ciel.
            v = 0.0;
            w = Kp_ang * err_ang;
        }
        else {
            v = Kp_lin * err_lin * 1000.0; // m -> mm/s
            w = Kp_ang * err_ang;
        }

        // Jemne obmedzenie zmeny rychlosti, aby robot neskakal.
        if((v - prev_v) > 5.0)
            v = prev_v + 5.0;

        if((w - prev_w) > 0.3)
            w = prev_w + 0.3;
        else if((w - prev_w) < -0.3)
            w = prev_w - 0.3;
    }

    prev_v = v;
    prev_w = w;

    v = std::clamp(v, -400.0, 400.0); // mm/s
    w = std::clamp(w, -2.0, 2.0);     // rad/s

    setSpeedVal(v, w);

    if(datacounter % 5 == 0)
    {
        cout << "\nRobot pos x/y/angle: " << x << " " << y << " " << fi;
        cout << "\nErrors lin/ang:      " << err_lin << " " << err_ang;
        cout << "\nCommands v/w:        " << v << " " << w << std::endl;
        emit publishPosition(x, y, fi);
    }

    if(useDirectCommands == 0)
    {
        if(forwardspeed == 0 && rotationspeed != 0)
            robotCom.setRotationSpeed(rotationspeed);
        else if(forwardspeed != 0 && rotationspeed == 0)
            robotCom.setTranslationSpeed(forwardspeed);
        else if(forwardspeed != 0 && rotationspeed != 0)
            robotCom.setArcSpeed(forwardspeed, forwardspeed / rotationspeed);
        else
            robotCom.setTranslationSpeed(0);
    }

    datacounter++;
    return 0;
}


/// uloha2 ///

void robot::stopRobot()
{
    stoppedByButton = true;

    forwardspeed = 0;
    rotationspeed = 0;
    prev_v = 0;
    prev_w = 0;

    x_des = x;
    y_des = y;

    robotCom.setTranslationSpeed(0);
    robotCom.setRotationSpeed(0);
}

void robot::resumeRobot()
{
    stoppedByButton = false;

    prev_v = 0;
    prev_w = 0;
}

float robot::normalizeAngle360(float angleDeg)
{
    while(angleDeg < 0.0f) angleDeg += 360.0f;
    while(angleDeg >= 360.0f) angleDeg -= 360.0f;
    return angleDeg;
}

float robot::normalizeAngle180(float angleDeg)
{
    angleDeg = normalizeAngle360(angleDeg);
    if(angleDeg > 180.0f)
        angleDeg -= 360.0f;
    return angleDeg;
}

float robot::normalizeLaserAngle(float rawAngleDeg)
{
    // - lebo ľavotočvý
    return normalizeAngle360(-rawAngleDeg);
}

float robot::laserDistanceToMeters(float rawDistance)
{
    return rawDistance / 1000.0f;
}

float robot::minDistanceInAngleRange(float fromDeg, float toDeg)
{
    float best = 1e9f;

    for(const auto& point : copyOfLaserData)
    {
        float angle = normalizeAngle180(normalizeLaserAngle(point.scanAngle));
        float dist = laserDistanceToMeters(point.scanDistance);

        if(dist <= 0.0f)
            continue;

        if(fromDeg <= toDeg)
        {
            if(angle >= fromDeg && angle <= toDeg)
                best = std::min(best, dist);
        }
        else
        {
            if(angle >= fromDeg || angle <= toDeg)
                best = std::min(best, dist);
        }
    }

    return best;
}

bool robot::canIgnoreVFHNearGoal(float signedGoalAngleDeg, double distToTarget)
{
    if(distToTarget > finalApproachDistance)
        return false;

    if(std::abs(signedGoalAngleDeg) > 70.0f)
        return false;

    // Pozrieme sa laserom v smere ciela.
    float goalRayDist = minDistanceInAngleRange(
        signedGoalAngleDeg - 12.0f,
        signedGoalAngleDeg + 12.0f
        );

    if(goalRayDist == 1e9f)
        return false;

    if(goalRayDist <= static_cast<float>(distToTarget))
        return false;

    return goalRayDist > static_cast<float>(distToTarget) + wallBehindTargetMargin;
}

void robot::updatePrimaryHistogram()
{
    primaryHistogram.clear();
    primaryHistogram.resize(sectorCount, 0.0f);

    float sectorSize = 360.0f / sectorCount;

    for(const auto& point : copyOfLaserData)
    {
        float alpha = normalizeLaserAngle(point.scanAngle);
        float d = laserDistanceToMeters(point.scanDistance); // m

        if(d <= 0.0f)
            continue;

        if(d > 1.20f)
            continue;

        float gammaDeg = 0.0f;

        if(d <= robotSafetyRadius)
        {
            gammaDeg = 90.0f;
        }
        else
        {
            float ratio = robotSafetyRadius / d;
            ratio = std::clamp(ratio, -1.0f, 1.0f);
            gammaDeg = asinf(ratio) * 180.0f / M_PI;
        }

        float leftBound  = normalizeAngle360(alpha - gammaDeg);
        float rightBound = normalizeAngle360(alpha + gammaDeg);

        //váha prekážky
        //čím je prekážka bližšie, tým je mi väčšie
        //čím je prekážka ďalej, tým je mi menšie
        float mi = a_hist - b_hist * d;

        if(mi < 0.0f)
            mi = 0.0f;

        for(int k = 0; k < sectorCount; k++)
        {
            float sectorStart = k * sectorSize;
            float sectorEnd   = (k + 1) * sectorSize;

            bool belongs = false;

            if(leftBound <= rightBound)
            {
                if(std::max(sectorStart, leftBound) <= std::min(sectorEnd, rightBound))
                    belongs = true;
            }
            else
            {
                // Interval prechadza cez 0 stupnov.
                if(sectorStart <= rightBound || sectorEnd >= leftBound)
                    belongs = true;
            }

            if(belongs)
                primaryHistogram[k] += mi;
        }
    }
}

void robot::updateBinaryHistogram(float threshold)
{
    binaryHistogram.clear();
    binaryHistogram.resize(primaryHistogram.size(), 0);

    for(int i = 0; i < static_cast<int>(primaryHistogram.size()); i++)
    {
        binaryHistogram[i] = (primaryHistogram[i] >= threshold) ? 1 : 0;
    }
}

void robot::updateMaskedHistogram()
{
    maskedHistogram = binaryHistogram;
}

void robot::findFreeGaps()
{
    freeGaps.clear();

    int start = -1;

    for(int i = 0; i < static_cast<int>(maskedHistogram.size()); i++)
    {
        if(maskedHistogram[i] == 0)
        {
            if(start == -1)
                start = i;
        }
        else
        {
            if(start != -1)
            {
                freeGaps.push_back({start, i - 1});
                start = -1;
            }
        }
    }

    if(start != -1)
        freeGaps.push_back({start, static_cast<int>(maskedHistogram.size()) - 1});
}

int robot::goalSector(float goalAngleDeg)
{
    goalAngleDeg = normalizeAngle360(goalAngleDeg);
    float sectorSize = 360.0f / sectorCount;
    return static_cast<int>(goalAngleDeg / sectorSize);
}

int robot::circularSectorDiff(int a, int b)
{
    int diff = std::abs(a - b);
    return std::min(diff, sectorCount - diff);
}

float robot::candidateCost(int candidateSector, int goalSectorIdx)
{
    int diffGoal = circularSectorDiff(candidateSector, goalSectorIdx);
    int diffForward = circularSectorDiff(candidateSector, 0);
    int diffPrev = 0;

    if(previousBestSector != -1)
        diffPrev = circularSectorDiff(candidateSector, previousBestSector);

    float mu1 = 3.0f; // cielovy smer
    float mu2 = 2.0f; // aktualny smer robota
    float mu3 = 5.0f; // predosly vybraty smer

    return mu1 * diffGoal + mu2 * diffForward + mu3 * diffPrev;
}

int robot::chooseBestSector(float goalAngleDeg)
{
    if(freeGaps.empty())
        return -1;

    int gSector = goalSector(goalAngleDeg);
    std::vector<int> candidates;

    int edgeOffset = 3;

    for(const auto& gap : freeGaps)
    {
        int start = gap.first;
        int end   = gap.second;
        int width = end - start + 1;

        if(width <= 6)
        {
            candidates.push_back((start + end) / 2);
        }
        else
        {
            int leftCandidate  = start + edgeOffset;
            int rightCandidate = end - edgeOffset;

            if(leftCandidate > end) leftCandidate = start;
            if(rightCandidate < start) rightCandidate = end;

            candidates.push_back(leftCandidate);
            candidates.push_back(rightCandidate);
        }

        if(gSector >= start && gSector <= end)
            candidates.push_back(gSector);
    }

    if(candidates.empty())
        return -1;

    int bestSector = candidates.front();
    float bestCost = candidateCost(bestSector, gSector);

    for(int candidate : candidates)
    {
        float cost = candidateCost(candidate, gSector);

        if(cost < bestCost)
        {
            bestCost = cost;
            bestSector = candidate;
        }
    }

    previousBestSector = bestSector;
    return bestSector;
}

double robot::sectorToAngle(int sectorIndex)
{
    double sectorSize = 360.0 / sectorCount;
    return sectorIndex * sectorSize + sectorSize / 2.0;
}

void robot::updateWaypointFromSector(int bestSector)
{
    if(bestSector < 0)
    {
        x_des = x;
        y_des = y;
        return;
    }

    double targetAngleDeg = sectorToAngle(bestSector);
    if(targetAngleDeg > 180.0)
        targetAngleDeg -= 360.0;

    double targetAngleLocalRad = targetAngleDeg * M_PI / 180.0;
    double targetAngleGlobalRad = fi + targetAngleLocalRad;

    double waypointDist = 0.25; // m

    x_des = x + waypointDist * cos(targetAngleGlobalRad);
    y_des = y + waypointDist * sin(targetAngleGlobalRad);
}

void robot::updateVFHNavigation(float goalAngleDeg, double distToTarget)
{
    updatePrimaryHistogram();
    updateBinaryHistogram(histogramThreshold);
    updateMaskedHistogram();
    findFreeGaps();

    emit publishHistogram(binaryHistogram, maskedHistogram);

    int gSector = goalSector(goalAngleDeg);

    bool goalIsFree =
        gSector >= 0 &&
        gSector < static_cast<int>(maskedHistogram.size()) &&
        maskedHistogram[gSector] == 0;

    float signedGoalAngleDeg = normalizeAngle180(goalAngleDeg);

    float goalRayDist = minDistanceInAngleRange(
        signedGoalAngleDeg - 12.0f,
        signedGoalAngleDeg + 12.0f
        );

    bool pathToGoalClear =
        goalRayDist == 1e9f ||
        goalRayDist > static_cast<float>(distToTarget) + 0.15f;

    if(goalIsFree && pathToGoalClear)
    {
        x_des = x_target;
        y_des = y_target;
        previousBestSector = -1;
        return;
    }

    int bestSector = chooseBestSector(goalAngleDeg);
    updateWaypointFromSector(bestSector);
}

int robot::processThisLidar(const std::vector<LaserData>& laserData)
{
    copyOfLaserData = laserData;

    if(stoppedByButton)
    {
        x_des = x;
        y_des = y;
        previousBestSector = -1;

        emit publishLidar(copyOfLaserData);
        return 0;
    }

    double dx_goal = x_target - x;
    double dy_goal = y_target - y;

    //euklidovská vzdialnosť
    double distToTarget = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

    if(distToTarget < 0.05)
    {
        x_des = x_target;
        y_des = y_target;

        previousBestSector = -1;

        emit publishLidar(copyOfLaserData);
        return 0;
    }


    double goalAngleGlobalRad = atan2(dy_goal, dx_goal);
    double goalAngleLocalRad = goalAngleGlobalRad - fi;

    while(goalAngleLocalRad >  M_PI) goalAngleLocalRad -= 2.0 * M_PI;
    while(goalAngleLocalRad < -M_PI) goalAngleLocalRad += 2.0 * M_PI;

    float signedGoalAngleDeg = static_cast<float>(goalAngleLocalRad * 180.0 / M_PI);

    if(canIgnoreVFHNearGoal(signedGoalAngleDeg, distToTarget))
    {
        x_des = x_target;
        y_des = y_target;
        previousBestSector = -1;
        emit publishLidar(copyOfLaserData);
        return 0;
    }

    float goalAngleLocalDeg = normalizeAngle360(signedGoalAngleDeg);

    updateVFHNavigation(goalAngleLocalDeg, distToTarget);

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
