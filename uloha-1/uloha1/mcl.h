#pragma once

#include "librobot/librobot.h"
#include "map.h"
#include <vector>
#include <random>
#include <utility>
#include <cmath>

class MCL {
public:
    struct Particle { double x, y, fi, weight; };
    struct Pose     { double x, y, fi; };

    static const int    GRID_SIZE = 280;
    static constexpr double CELL_SIZE = 0.05;

    static const int    N_MIN         = 50;
    static const int    N_MAX         = 500;
    static const int    N_DEFAULT     = 300;
    static constexpr double VAR_HIGH      = 0.50;
    static constexpr double VAR_LOW       = 0.05;
    static constexpr double VAR_CONVERGED = 0.01;

    static constexpr double A1 = 0.10;
    static constexpr double A2 = 0.01;
    static constexpr double A3 = 0.05;
    static constexpr double A4 = 0.01;

    MCL();

    void init(const Map& map);

    bool isActive() const { return active; }
    void deactivate();


    void motionUpdate(double step_dist, double delta_fi);

    void weightUpdate(const std::vector<LaserData>& laser);
    void resample();

    void reinit();

    Pose estimatePose() const;

    double particleVariance() const;

    bool hasConverged() const;
    int particleCount() const { return (int)particles.size(); }
    int distFieldAt(int row, int col) const { return dist_field[row][col]; }

    bool isPoseValid() const { return last_pose_valid; }

private:
    bool   active = false;
    int    N      = N_DEFAULT;

    std::vector<Particle>          particles;
    std::vector<std::pair<int,int>> free_cells;
    int    dist_field[GRID_SIZE][GRID_SIZE];

    const Map* map_ptr = nullptr;

    mutable bool last_pose_valid = false;

    std::mt19937 rng;

    void computeDistanceField();
    void scatterParticles(int n);
    void worldToGrid(double wx, double wy, int& col, int& row) const;
};
