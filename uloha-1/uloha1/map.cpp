#include "map.h"
#include <fstream>
#include <iostream>
#include <cstring>
#include <queue>
#include <algorithm>
#include <tuple>

using std::cout;

Map::Map()
{
    cout << "[Map] Constructor start\n" << std::flush;
    memset(grid, 0, sizeof(grid));
    cout << "[Map] Constructor done\n" << std::flush;
}

void Map::saveMap(const std::string& filename)
{
    cout << "[Map] Saving...\n";
    std::ofstream file(filename);
    if (!file.is_open()) { cout << "[Map] Save failed: cannot open file\n"; return; }

    for (int r = 0; r < GRID_SIZE; r++) {
        for (int c = 0; c < GRID_SIZE; c++) {
            file << grid[r][c];
            if (c < GRID_SIZE - 1) file << " ";
        }
        file << "\n";
    }
    file.close();
    cout << "[Map] Saved\n";
}

void Map::loadMap(const std::string& filename)
{
    cout << "[Map] Loading...\n";
    std::ifstream file(filename);
    if (!file.is_open()) { cout << "[Map] Load failed: cannot open file\n"; return; }

    for (int r = 0; r < GRID_SIZE; r++)
        for (int c = 0; c < GRID_SIZE; c++) {
            file >> grid[r][c];
            if (grid[r][c] > BUFFER) grid[r][c] = FREE;
        }

    file.close();
    cout << "[Map] Loaded\n";
}


void Map::addPose(const PoseStamp& ps)
{
    poseHistory.push_back(ps);
    if (poseHistory.size() > 200)
        poseHistory.pop_front();
}

void Map::processLidarScan(const std::vector<LaserData>& scan)
{
    if (poseHistory.empty()) return;

    for (const auto& point : scan) {
        double dist_m = point.scanDistance / 1000.0;
        if (dist_m < 0.05 || dist_m > 2.5 || (dist_m > 0.5 && dist_m < 0.7)) continue;
        PoseStamp pose = interpolatePose(point.timestamp);
        updateCell(point, pose);
    }
}

void Map::worldToGrid(double wx, double wy, int& col, int& row) const
{
    col = (int)std::floor((wx - originX) / CELL_SIZE);
    row = (int)std::floor((wy - originY) / CELL_SIZE);
}

std::vector<int> Map::occDir(int r, int c) const
{
    return {
        grid[r-1][c] == OCCUPIED ? 1 : 0,
        grid[r][c-1] == OCCUPIED ? 1 : 0,
        grid[r+1][c] == OCCUPIED ? 1 : 0,
        grid[r][c+1] == OCCUPIED ? 1 : 0
    };
}

void Map::bufferMap()
{
    std::vector<std::pair<int,int>> obstacles;
    for (int r = 0; r < GRID_SIZE; r++)
        for (int c = 0; c < GRID_SIZE; c++)
            if (grid[r][c] == OCCUPIED)
                obstacles.push_back({r, c});

    const int dr[] = {-1, 1, 0, 0, -1, -1,  1,  1};
    const int dc[] = { 0, 0, 1,-1,  1, -1,  1, -1};

    for (auto [r, c] : obstacles)
        for (int dir = 0; dir < 8; dir++)
            for (int p = 1; p <= BUFFER_SIZE; p++) {
                int nr = r + dr[dir] * p;
                int nc = c + dc[dir] * p;
                if (nr < 0 || nr >= GRID_SIZE || nc < 0 || nc >= GRID_SIZE) break;
                if      (grid[nr][nc] == FREE)     grid[nr][nc] = BUFFER;
                else if (grid[nr][nc] == OCCUPIED) break;
            }
}

void Map::floodMap(double x_des, double y_des, double robot_x, double robot_y)
{
    double x_offset = GRID_OFFSET_X * CELL_SIZE;
    double y_offset = GRID_OFFSET_Y * CELL_SIZE;

    int x_des_grid = (int)((x_des + x_offset) / CELL_SIZE);
    int y_des_grid = (int)((y_des + y_offset) / CELL_SIZE);
    int x_grid     = (int)((robot_x + x_offset) / CELL_SIZE);
    int y_grid     = (int)((robot_y + y_offset) / CELL_SIZE);

    for (int i = 0; i < GRID_SIZE; i++)
        for (int j = 0; j < GRID_SIZE; j++)
            if (grid[i][j] > BUFFER) grid[i][j] = FREE;

    bufferMap();

    std::priority_queue<
        std::tuple<int,int,int>,
        std::vector<std::tuple<int,int,int>>,
        std::greater<>
        > pq;

    grid[y_des_grid][x_des_grid] = 4;
    pq.push({4, x_des_grid, y_des_grid});

    const int dx[]        = {-1, 1, 0, 0, -1, -1,  1,  1};
    const int dy[]        = { 0, 0,-1, 1, -1,  1, -1,  1};
    const int step_cost[] = {10,10,10,10, 14, 14, 14, 14};

    while (!pq.empty()) {
        auto [cost, x, y] = pq.top(); pq.pop();
        if (grid[y][x] != cost) continue;

        for (int i = 0; i < 8; i++) {
            int nx = x + dx[i], ny = y + dy[i];
            int new_cost = cost + step_cost[i];
            if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE)
                if (grid[ny][nx] == FREE) {
                    grid[ny][nx] = new_cost;
                    pq.push({new_cost, nx, ny});
                    if (nx == x_grid && ny == y_grid) return;
                }
        }
    }

    if (grid[y_grid][x_grid] < 4)
        cout << "[Map] ERROR: start position not reachable\n";
}

std::vector<std::pair<double,double>> Map::calculatePath(double start_x_world, double start_y_world)
{
    double x_offset = GRID_OFFSET_X * CELL_SIZE;
    double y_offset = GRID_OFFSET_Y * CELL_SIZE;

    int start_x = (int)((start_x_world + x_offset) / CELL_SIZE);
    int start_y = (int)((start_y_world + y_offset) / CELL_SIZE);

    std::vector<std::pair<int,int>> grid_path;
    grid_path.push_back({start_x, start_y});

    while (grid[start_y][start_x] != 4) {
        int min_val = grid[start_y][start_x];
        int best_x  = start_x, best_y = start_y;

        int cur_dx = 0, cur_dy = 0;
        if (grid_path.size() >= 2) {
            cur_dx = start_x - grid_path[grid_path.size()-2].first;
            cur_dy = start_y - grid_path[grid_path.size()-2].second;
        }

        const int dx[] = {-1,-1, 1, 1, 0, 0,-1, 1};
        const int dy[] = {-1, 1,-1, 1,-1, 1, 0, 0};

        for (int i = 0; i < 8; i++) {
            int nx = start_x + dx[i];
            int ny = start_y + dy[i];
            bool is_turn     = (dx[i] != cur_dx || dy[i] != cur_dy);
            int  turn_penalty = is_turn ? 3 : 0;

            if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE)
                if (grid[ny][nx] >= 4) {
                    int effective_cost = grid[ny][nx] + turn_penalty;
                    if (effective_cost < min_val) {
                        min_val = effective_cost;
                        best_x  = nx;
                        best_y  = ny;
                    }
                }
        }

        start_x = best_x;
        start_y = best_y;
        grid_path.push_back({start_x, start_y});

        if (grid_path.size() > (size_t)(GRID_SIZE * GRID_SIZE)) {
            cout << "[Map] No path found\n";
            break;
        }
    }

    std::vector<std::pair<double,double>> world_path;
    auto toWorld = [&](int gx, int gy) -> std::pair<double,double> {
        return { (gx - GRID_OFFSET_X) * CELL_SIZE,
                (gy - GRID_OFFSET_Y) * CELL_SIZE };
    };

    world_path.push_back(toWorld(grid_path[0].first, grid_path[0].second));

    for (int i = 1; i < (int)grid_path.size() - 1; i++) {
        int dx_prev = grid_path[i].first   - grid_path[i-1].first;
        int dy_prev = grid_path[i].second  - grid_path[i-1].second;
        int dx_next = grid_path[i+1].first - grid_path[i].first;
        int dy_next = grid_path[i+1].second- grid_path[i].second;
        if (dx_prev != dx_next || dy_prev != dy_next)
            world_path.push_back(toWorld(grid_path[i].first, grid_path[i].second));
    }

    world_path.push_back(toWorld(grid_path.back().first, grid_path.back().second));

    cout << "[Map] Path: " << world_path.size() << " waypoints\n";
    return world_path;
}

Map::PoseStamp Map::interpolatePose(unsigned ts) const
{
    if (poseHistory.empty()) return {0, 0, 0, ts};
    if (ts <= poseHistory.front().timestamp) return poseHistory.front();
    if (ts >= poseHistory.back().timestamp)  return poseHistory.back();

    for (size_t i = 1; i < poseHistory.size(); i++) {
        if (poseHistory[i].timestamp >= ts) {
            const PoseStamp& a = poseHistory[i-1];
            const PoseStamp& b = poseHistory[i];
            double ratio = (double)(ts - a.timestamp) / (double)(b.timestamp - a.timestamp);
            double dfi   = b.fi - a.fi;
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

void Map::bresenham(int c0, int r0, int c1, int r1)
{
    int dc = std::abs(c1-c0), dr = std::abs(r1-r0);
    int sc = (c0 < c1) ? 1 : -1;
    int sr = (r0 < r1) ? 1 : -1;
    int err = dc - dr;

    while (true) {
        if (c0 == c1 && r0 == r1) break;
        if (c0 >= 0 && c0 < GRID_SIZE && r0 >= 0 && r0 < GRID_SIZE)
            if (grid[r0][c0] == UNKNOWN)
                grid[r0][c0] = FREE;
        int e2 = 2 * err;
        if (e2 > -dr) { err -= dr; c0 += sc; }
        if (e2 <  dc) { err += dc; r0 += sr; }
    }
}

void Map::updateCell(const LaserData& point, const PoseStamp& pose)
{
    double dist_m       = point.scanDistance / 1000.0;
    double global_angle = pose.fi - (point.scanAngle * M_PI / 180.0);
    double x_hit        = pose.x + dist_m * std::cos(global_angle);
    double y_hit        = pose.y + dist_m * std::sin(global_angle);

    int col_r, row_r, col_h, row_h;
    worldToGrid(pose.x, pose.y, col_r, row_r);
    worldToGrid(x_hit,  y_hit,  col_h, row_h);

    bresenham(col_r, row_r, col_h, row_h);

    if (col_h >= 0 && col_h < GRID_SIZE && row_h >= 0 && row_h < GRID_SIZE)
        grid[row_h][col_h] = OCCUPIED;
}
