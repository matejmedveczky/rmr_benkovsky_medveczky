#pragma once

#include "librobot/librobot.h"
#include <string>
#include <vector>
#include <deque>
#include <utility>
#include <cmath>

class Map {
public:
    // ── Cell states ─────────────────────────────────────────────────
    enum CellState { UNKNOWN = 0, FREE = 1, OCCUPIED = 2, BUFFER = 3 };

    // ── Pose stamp (for LIDAR timestamp interpolation) ───────────────
    struct PoseStamp { double x, y, fi; unsigned timestamp; };

    // ── Grid constants ───────────────────────────────────────────────
    static const int         GRID_SIZE    = 280;
    static constexpr double  CELL_SIZE    = 0.05;
    static const int         GRID_OFFSET_X = 140;
    static const int         GRID_OFFSET_Y = 140;
    static const int         BUFFER_SIZE  = 5;

    // Log-odds thresholds (map building)
    static constexpr float   L_HIT            =  0.85f;
    static constexpr float   L_FREE           =  0.40f;
    static constexpr float   L_MIN            = -10.0f;
    static constexpr float   L_MAX            =  10.0f;
    static constexpr float   THRESHOLD_OCC   =  0.5f;
    static constexpr float   THRESHOLD_FREE  = -0.5f;

    // ── Public grid (robot reads it to emit publishMap and pass to MCL) ─
    int grid[GRID_SIZE][GRID_SIZE];

    double originX = -(GRID_SIZE * CELL_SIZE / 2.0);
    double originY = -(GRID_SIZE * CELL_SIZE / 2.0);

    // ── Lifecycle ───────────────────────────────────────────────────
    Map();

    // ── I/O ─────────────────────────────────────────────────────────
    void saveMap(const std::string& filename);
    void loadMap(const std::string& filename);

    // ── Map building ────────────────────────────────────────────────
    // Call from processThisRobot to keep pose history up to date
    void addPose(const PoseStamp& ps);

    // Call from processThisLidar — runs the full update loop internally
    void processLidarScan(const std::vector<LaserData>& scan);

    // ── Coordinate utilities ─────────────────────────────────────────
    void worldToGrid(double wx, double wy, int& col, int& row) const;

    // ── Map queries ──────────────────────────────────────────────────
    // Returns {north, west, south, east} occupancy flags around (r,c)
    std::vector<int> occDir(int r, int c) const;

    // ── Path planning ────────────────────────────────────────────────
    // Flood-fills grid with Dijkstra costs from (x_des, y_des).
    // robot_x/y used only for early exit optimisation.
    void floodMap(double x_des, double y_des, double robot_x, double robot_y);

    // Greedy descent on flood values starting from (start_x, start_y).
    // Returns world-space waypoints.
    std::vector<std::pair<double,double>> calculatePath(double start_x, double start_y);

    // ── Obstacle inflation ───────────────────────────────────────────
    void bufferMap();

private:
    std::deque<PoseStamp> poseHistory;

    PoseStamp interpolatePose(unsigned timestamp) const;
    void      bresenham(int c0, int r0, int c1, int r1);
    void      updateCell(const LaserData& point, const PoseStamp& pose);
};
