#pragma once

#include "librobot/librobot.h"
#include <string>
#include <vector>
#include <deque>
#include <utility>
#include <cmath>

class Map {
public:
    enum CellState { UNKNOWN = 0, FREE = 1, OCCUPIED = 2, BUFFER = 3 };

    struct PoseStamp { double x, y, fi; unsigned timestamp; };

    static const int         GRID_SIZE    = 280;
    static constexpr double  CELL_SIZE    = 0.05;
    static const int         GRID_OFFSET_X = 140;
    static const int         GRID_OFFSET_Y = 140;
    static const int         BUFFER_SIZE  = 5;

    static constexpr float   L_HIT            =  0.85f;
    static constexpr float   L_FREE           =  0.40f;
    static constexpr float   L_MIN            = -10.0f;
    static constexpr float   L_MAX            =  10.0f;
    static constexpr float   THRESHOLD_OCC   =  0.5f;
    static constexpr float   THRESHOLD_FREE  = -0.5f;

    int grid[GRID_SIZE][GRID_SIZE];

    double originX = -(GRID_SIZE * CELL_SIZE / 2.0);
    double originY = -(GRID_SIZE * CELL_SIZE / 2.0);

    Map();

    void saveMap(const std::string& filename);
    void loadMap(const std::string& filename);


    void addPose(const PoseStamp& ps);

    void processLidarScan(const std::vector<LaserData>& scan);

    void worldToGrid(double wx, double wy, int& col, int& row) const;

    std::vector<int> occDir(int r, int c) const;

    void floodMap(double x_des, double y_des, double robot_x, double robot_y);

    std::vector<std::pair<double,double>> calculatePath(double start_x, double start_y);

    void bufferMap();

private:
    std::deque<PoseStamp> poseHistory;

    PoseStamp interpolatePose(unsigned timestamp) const;
    void      bresenham(int c0, int r0, int c1, int r1);
    void      updateCell(const LaserData& point, const PoseStamp& pose);
};
