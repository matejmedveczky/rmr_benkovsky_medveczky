#pragma once

#include "librobot/librobot.h"
#include "map.h"
#include <vector>
#include <random>
#include <utility>
#include <cmath>

class MCL {
public:
    // ── Public types ────────────────────────────────────────────────
    struct Particle { double x, y, fi, weight; };
    struct Pose     { double x, y, fi; };

    // ── Grid constants (must match robot.h) ─────────────────────────
    static const int    GRID_SIZE = 280;
    static constexpr double CELL_SIZE = 0.05;

    // ── Tuning constants ────────────────────────────────────────────
    static const int    N_MIN         = 50;
    static const int    N_MAX         = 500;
    static const int    N_DEFAULT     = 300;
    static constexpr double VAR_HIGH      = 0.50;   // m² → grow N
    static constexpr double VAR_LOW       = 0.05;   // m² → shrink N
    static constexpr double VAR_CONVERGED = 0.01;   // m² → done

    // Motion noise (a1–a4 from probabilistic robotics)
    static constexpr double A1 = 0.10;   // rot  noise ← rotation
    static constexpr double A2 = 0.01;   // rot  noise ← translation
    static constexpr double A3 = 0.05;   // trans noise ← translation
    static constexpr double A4 = 0.01;   // trans noise ← rotation

    // ── Lifecycle ───────────────────────────────────────────────────
    MCL();

    // Call after map is ready (loadMap). Passes live Map reference —
    // MCL always reads current map state.
    void init(const Map& map);

    bool isActive() const { return active; }
    void deactivate();

    // ── Core MCL steps ──────────────────────────────────────────────

    // Called every robot tick with odometric deltas
    void motionUpdate(double step_dist, double delta_fi);

    // Called every LIDAR callback
    void weightUpdate(const std::vector<LaserData>& laser);
    void resample();

    // Returns best-particle pose estimate
    Pose estimatePose() const;

    // Current spatial variance of particle cloud (m²)
    double particleVariance() const;

    // True when variance < VAR_CONVERGED and robot is near (x_des, y_des)
    bool hasConverged() const;
    // Particle count (for logging)
    int particleCount() const { return (int)particles.size(); }

private:
    // ── State ───────────────────────────────────────────────────────
    bool   active = false;
    int    N      = N_DEFAULT;

    std::vector<Particle>          particles;
    std::vector<std::pair<int,int>> free_cells;  // (row, col) of FREE cells
    int    dist_field[GRID_SIZE][GRID_SIZE];

    // Live pointer to robot's Map — not owned
    const Map* map_ptr = nullptr;

    std::mt19937 rng;

    // ── Internal helpers ────────────────────────────────────────────
    void computeDistanceField();
    void scatterParticles(int n);
    void worldToGrid(double wx, double wy, int& col, int& row) const;
};
