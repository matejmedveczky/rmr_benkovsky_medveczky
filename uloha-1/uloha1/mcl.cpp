#include "mcl.h"
#include <algorithm>
#include <climits>
#include <iostream>
#include <queue>

using std::cout;

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

MCL::MCL() : rng(std::random_device{}())
{
    cout << "[MCL] Constructor start\n" << std::flush;
    cout << "[MCL] Constructor done\n" << std::flush;
}

void MCL::init(const Map& map)
{
    map_ptr = &map;

    // Cache free cells — used for initial scatter and random injection
    free_cells.clear();
    for (int r = 0; r < GRID_SIZE; r++)
        for (int c = 0; c < GRID_SIZE; c++)
            if (map_ptr->grid[r][c] == Map::FREE)
                free_cells.push_back({r, c});

    computeDistanceField();
    scatterParticles(N);
    active = true;

    cout << "[MCL] Initialized: " << N << " particles, "
         << free_cells.size() << " free cells\n";
}

void MCL::activate()
{
    active = true;
    cout << "[MCL] Activated";
}

void MCL::deactivate()
{
    active = false;
    first_deactivate = false;
    //particles.clear();
    //free_cells.clear();
    cout << "[MCL] Deactivated - odometry takes over\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal: BFS distance field
// dist_field[r][c] = distance in cells to nearest OCCUPIED cell
// ─────────────────────────────────────────────────────────────────────────────

void MCL::computeDistanceField()
{
    for (int r = 0; r < GRID_SIZE; r++)
        for (int c = 0; c < GRID_SIZE; c++)
            dist_field[r][c] = INT_MAX / 2;

    std::queue<std::pair<int,int>> q;

    for (int r = 0; r < GRID_SIZE; r++)
        for (int c = 0; c < GRID_SIZE; c++)
            if (map_ptr->grid[r][c] == Map::OCCUPIED) {
                dist_field[r][c] = 0;
                q.push({r, c});
            }

    const int dr[] = {-1, 1,  0, 0, -1, -1,  1,  1};
    const int dc[] = { 0, 0, -1, 1, -1,  1, -1,  1};

    while (!q.empty()) {
        auto [r, c] = q.front(); q.pop();
        for (int i = 0; i < 8; i++) {
            int nr = r + dr[i], nc = c + dc[i];
            if (nr >= 0 && nr < GRID_SIZE && nc >= 0 && nc < GRID_SIZE)
                if (dist_field[nr][nc] > dist_field[r][c] + 1) {
                    dist_field[nr][nc] = dist_field[r][c] + 1;
                    q.push({nr, nc});
                }
        }
    }
    cout << "[MCL] Distance field computed\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal: scatter N particles uniformly over free cells
// ─────────────────────────────────────────────────────────────────────────────

void MCL::scatterParticles(int n)
{
    particles.clear();
    if (free_cells.empty()) return;

    std::uniform_int_distribution<int>    cell_dist(0, (int)free_cells.size() - 1);
    std::uniform_real_distribution<double> angle_dist(-M_PI, M_PI);

    for (int i = 0; i < n; i++) {
        auto [r, c] = free_cells[cell_dist(rng)];
        Particle p;
        p.x      = c * CELL_SIZE + map_ptr->originX;
        p.y      = r * CELL_SIZE + map_ptr->originY;
        p.fi     = angle_dist(rng);
        p.weight = 1.0 / n;
        p.valid  = true;
        particles.push_back(p);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal: coordinate conversion
// ─────────────────────────────────────────────────────────────────────────────

void MCL::worldToGrid(double wx, double wy, int& col, int& row) const
{
    col = (int)std::floor((wx - map_ptr->originX) / CELL_SIZE);
    row = (int)std::floor((wy - map_ptr->originY) / CELL_SIZE);
}

// ─────────────────────────────────────────────────────────────────────────────
// motionUpdate — called every robot tick
// Applies odometric delta + noise to every particle
// ─────────────────────────────────────────────────────────────────────────────

void MCL::motionUpdate(double step_dist, double delta_fi)
{
    if (!active || particles.empty()) return;

    std::normal_distribution<double> gauss(0.0, 1.0);

    for (auto& p : particles) {
        double noisy_fi   = delta_fi
                          + (A1 * std::abs(delta_fi) + A2 * std::abs(step_dist))
                                * gauss(rng);
        double noisy_dist = step_dist
                            + (A3 * std::abs(step_dist) + A4 * std::abs(delta_fi))
                                  * gauss(rng);

        p.fi += noisy_fi;
        while (p.fi >  M_PI) p.fi -= 2*M_PI;
        while (p.fi < -M_PI) p.fi += 2*M_PI;

        p.x += noisy_dist * std::cos(p.fi);
        p.y += noisy_dist * std::sin(p.fi);

        // Penalize particles that walked into walls — they'll die at resample
        int col, row;
        worldToGrid(p.x, p.y, col, row);
        bool out_of_bounds = (col < 0 || col >= GRID_SIZE || row < 0 || row >= GRID_SIZE);
        bool in_wall       = !out_of_bounds && (map_ptr->grid[row][col] == Map::OCCUPIED ||
                                          map_ptr->grid[row][col] == Map::BUFFER);
        p.valid = !(out_of_bounds || in_wall);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// weightUpdate — called every LIDAR callback
// Scores each particle by how well its projected LIDAR matches the map.
// Uses every 10th ray. Weight = 1 / (1 + mean_distance_error_in_metres)
// ─────────────────────────────────────────────────────────────────────────────

void MCL::weightUpdate(const std::vector<LaserData>& laser)
{
    if (!active || laser.empty() || particles.empty()) return;

    constexpr double INVALID_WEIGHT = 1e-12;
    double valid_weight_sum = 0.0;

    for (auto& p : particles) {
        if (!p.valid) {
            p.weight = INVALID_WEIGHT;
            continue;
        }

        double error    = 0.0;
        int    rays     = 0;

        for (int i = 0; i < (int)laser.size(); i += 10) {
            double dist_m = laser[i].scanDistance / 1000.0;
            if (dist_m < 0.05 || dist_m > 2.5) continue;

            double angle = p.fi - (laser[i].scanAngle * M_PI / 180.0);
            double hit_x = p.x  + dist_m * std::cos(angle);
            double hit_y = p.y  + dist_m * std::sin(angle);

            int col, row;
            worldToGrid(hit_x, hit_y, col, row);

            if (col >= 0 && col < GRID_SIZE && row >= 0 && row < GRID_SIZE)
                error += dist_field[row][col] * CELL_SIZE;  // cells → metres
            else
                error += 2.0;  // out-of-bounds: max plausible penalty

            rays++;
        }

        if (rays > 0) error /= rays;
        p.weight   = 1.0 / (1.0 + error);
        valid_weight_sum += p.weight;
    }

    // Normalize valid particles only; invalid particles keep tiny weight.
    if (valid_weight_sum > 1e-18 && std::isfinite(valid_weight_sum))
        for (auto& p : particles)
            if (p.valid) p.weight /= valid_weight_sum;
}

// ─────────────────────────────────────────────────────────────────────────────
// resample — stochastic universal sampling + 5% random injection + AMCL resize
// ─────────────────────────────────────────────────────────────────────────────

void MCL::resample()
{
    if (!active || particles.empty()) return;

    // AMCL: adjust N based on variance
    double var = particleVariance();
    double inject_ratio = std::clamp(var / VAR_HIGH, 0.01, 0.05);  // 1-5%
    int n_random   = std::max(1, (int)(N * inject_ratio));
    int n_resample = N - n_random;

    // Build cumulative weight array
    std::vector<double> cum(particles.size());
    cum[0] = particles[0].weight;
    for (int i = 1; i < (int)particles.size(); i++)
        cum[i] = cum[i-1] + particles[i].weight;

    double cum_total = cum.back();
    if (!(cum_total > 0.0) || !std::isfinite(cum_total)) {
        double w = 1.0 / (double)particles.size();
        cum[0] = w;
        for (int i = 1; i < (int)particles.size(); i++)
            cum[i] = cum[i-1] + w;
        cum_total = cum.back();
    }

    std::uniform_real_distribution<double> start_dist(0.0, cum_total / n_resample);
    double start = start_dist(rng);
    int    idx   = 0;

    std::vector<Particle> next;
    next.reserve(N);

    for (int i = 0; i < n_resample; i++) {
        double target = start + (double)i * (cum_total / n_resample);
        while (idx < (int)cum.size() - 1 && cum[idx] < target) idx++;
        next.push_back(particles[idx]);
    }

    // Random injection for diversity / recovery
    if (!free_cells.empty()) {
        std::uniform_int_distribution<int>    cell_dist(0, (int)free_cells.size() - 1);
        std::uniform_real_distribution<double> angle_dist(-M_PI, M_PI);
        for (int i = 0; i < n_random; i++) {
            auto [r, c] = free_cells[cell_dist(rng)];
            Particle p;
            p.x = c * CELL_SIZE + map_ptr->originX;
            p.y = r * CELL_SIZE + map_ptr->originY;
            p.fi = angle_dist(rng);
            p.weight = 1.0 / N;
            p.valid = true;
            next.push_back(p);
        }
    }

    // Reset weights
    double w = 1.0 / (double)next.size();
    for (auto& p : next) p.weight = w;

    particles = std::move(next);
}

// ─────────────────────────────────────────────────────────────────────────────
// estimatePose — returns highest-weight particle
// Best particle is safer than weighted mean for multimodal clouds
// ─────────────────────────────────────────────────────────────────────────────

MCL::Pose MCL::estimatePose() const
{
    if (particles.empty()) return {0, 0, 0};
    const auto& best = *std::max_element(particles.begin(), particles.end(),
                                         [](const Particle& a, const Particle& b){ return a.weight < b.weight; });
    return {best.x, best.y, best.fi};
}

// ─────────────────────────────────────────────────────────────────────────────
// Diagnostics
// ─────────────────────────────────────────────────────────────────────────────

double MCL::particleVariance() const
{
    if (particles.empty()) return 1e9;

    // Only consider top 50% by weight
    std::vector<const Particle*> sorted;
    for (const auto& p : particles) sorted.push_back(&p);
    std::sort(sorted.begin(), sorted.end(),
              [](const Particle* a, const Particle* b){ return a->weight > b->weight; });

    int n = sorted.size() / 2;
    double mx = 0, my = 0;
    for (int i = 0; i < n; i++) { mx += sorted[i]->x; my += sorted[i]->y; }
    mx /= n; my /= n;

    double var = 0;
    for (int i = 0; i < n; i++) {
        double dx = sorted[i]->x - mx, dy = sorted[i]->y - my;
        var += dx*dx + dy*dy;
    }
    return var / n;
}

bool MCL::hasConverged() const
{
    if (!active) return false;
    return particleVariance() < VAR_CONVERGED;
}
