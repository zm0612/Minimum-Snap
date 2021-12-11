//
// Created by Zhang Zhimeng on 2021/11/25.
//

#include "path_searcher/a_star/a_star_2d.h"

#include <ros/ros.h>

AStar2D::AStar2D(double x_lower, double x_upper, double y_lower,
                 double y_upper, double resolution, bool allow_diagonal) {
    Init(x_lower, x_upper, y_lower, y_upper, resolution, allow_diagonal);
}

AStar2D::~AStar2D() {
    ReleaseMemory();
}

void AStar2D::Init(double x_lower, double x_upper, double y_lower,
                   double y_upper, double resolution, bool allow_diagonal) {
    ros::Time start_time = ros::Time::now();
    x_lower_ = x_lower;
    x_upper_ = x_upper;
    y_lower_ = y_lower;
    y_upper_ = y_upper;
    resolution_ = resolution;
    allow_diagonal_ = allow_diagonal;
    tie_breaker_ = 1.0 + 1.0 / 10000.0;

    GRID_X_SIZE_ = std::floor((x_upper_ - x_lower_) / resolution_);
    GRID_Y_SIZE_ = std::floor((y_upper_ - y_lower_) / resolution_);

    if (map_data_) {
        delete[] map_data_;
        map_data_ = nullptr;
    }
    map_data_ = new uint8_t[GRID_Y_SIZE_ * GRID_X_SIZE_];
    memset(map_data_, 0, GRID_X_SIZE_ * GRID_Y_SIZE_ * sizeof(uint8_t));

    if (grid_node_map_) {
        for (int i = 0; i < GRID_X_SIZE_; ++i) {
            for (int j = 0; j < GRID_Y_SIZE_; ++j) {
                delete grid_node_map_[i][j];
                grid_node_map_[i][j] = nullptr;
            }
            delete[] grid_node_map_[i];
            grid_node_map_[i] = nullptr;
        }

        delete[] grid_node_map_;
        grid_node_map_ = nullptr;
    }

    grid_node_map_ = new GridNodePtr<2> *[GRID_X_SIZE_];
    for (int i = 0; i < GRID_X_SIZE_; i++) {
        grid_node_map_[i] = new GridNodePtr<2>[GRID_Y_SIZE_];
        for (int j = 0; j < GRID_Y_SIZE_; ++j) {
            Eigen::Vector2i temp_index(i, j);
            Eigen::Vector2d pt = GridIndex2Coordinate(temp_index);
            grid_node_map_[i][j] = new GridNode<2>(temp_index, pt);
        }
    }
    ros::Time end_time = ros::Time::now();
    ROS_INFO_STREAM("Init grid map use time(ms): " << (end_time - start_time).toSec() * 1000);
}

void AStar2D::Reset() {
    for (int i = 0; i < GRID_X_SIZE_; ++i) {
        for (int j = 0; j < GRID_Y_SIZE_; ++j) {
            grid_node_map_[i][j]->Reset();
        }
    }
}

Eigen::Vector2d AStar2D::GridIndex2Coordinate(const Eigen::Vector2i &grid_index) const {
    Eigen::Vector2d pt;
    pt.x() = ((double) grid_index[0] + 0.5) * resolution_ + x_lower_;
    pt.y() = ((double) grid_index[1] + 0.5) * resolution_ + y_lower_;

    return pt;
}

Eigen::Vector2i AStar2D::Coordinate2GridIndex(const Eigen::Vector2d &pt) const {
    Eigen::Vector2i grid_index;

    grid_index[0] = std::min(std::max(int((pt[0] - x_lower_) / resolution_), 0), GRID_X_SIZE_ - 1);
    grid_index[1] = std::min(std::max(int((pt[1] - y_lower_) / resolution_), 0), GRID_Y_SIZE_ - 1);

    return grid_index;
}

Eigen::Vector2d AStar2D::CoordinateRounding(const Eigen::Vector2d &pt) {
    return GridIndex2Coordinate(Coordinate2GridIndex(pt));
}

bool AStar2D::HasObstacle(const Eigen::Vector2i &grid_index) const {
    int grid_index_x = grid_index[0];
    int grid_index_y = grid_index[1];

    return (grid_index_x >= 0 && grid_index_x < GRID_X_SIZE_ && grid_index_y >= 0 && grid_index_y < GRID_Y_SIZE_
            && (map_data_[grid_index_x * GRID_X_SIZE_ + grid_index_y] == 1));
}

__attribute__((unused)) bool AStar2D::IsFree(const Eigen::Vector2i &grid_index) const {
    int grid_index_x = grid_index[0];
    int grid_index_y = grid_index[1];

    return (grid_index_x >= 0 && grid_index_x < GRID_X_SIZE_ && grid_index_y >= 0 && grid_index_y < GRID_Y_SIZE_
            && (map_data_[grid_index_x * GRID_X_SIZE_ + GRID_Y_SIZE_] == 0));
}

bool AStar2D::Search(const Eigen::Vector2d &start_pt, const Eigen::Vector2d &goal_pt) {
    ros::Time start_time = ros::Time::now();

    const Eigen::Vector2i start_grid_index = Coordinate2GridIndex(start_pt);
    const Eigen::Vector2i goal_grid_index = Coordinate2GridIndex(goal_pt);

    GridNodePtr<2> start_node_ptr = grid_node_map_[start_grid_index[0]][start_grid_index[1]];
    const GridNodePtr<2> goal_node_ptr = grid_node_map_[goal_grid_index[0]][goal_grid_index[1]];

    start_node_ptr->g_score_ = 0.0;
    start_node_ptr->f_score_ = GetHeuristicValue(start_node_ptr, goal_node_ptr);
    start_node_ptr->status_ = NODE_STATUS::IN_OPENSET;

    openset_.clear();
    openset_.insert(std::make_pair(start_node_ptr->f_score_, start_node_ptr));

    std::vector<GridNodePtr<2>> neighbor_nodes;
    std::vector<double> neighbor_edge_cost;

    GridNodePtr<2> current_node_ptr;
    GridNodePtr<2> neighbor_node_ptr = nullptr;
    while (!openset_.empty()) {
        current_node_ptr = openset_.begin()->second;
        current_node_ptr->status_ = NODE_STATUS::IN_CLOSESET;
        openset_.erase(openset_.begin());

        if (current_node_ptr->grid_index_ == goal_grid_index) {
            ros::Time end_time = ros::Time::now();
            terminal_node_ptr_ = current_node_ptr;
            ROS_INFO("\033[1;32m --> Time in A star is %f ms, path cost %f m \033[0m",
                     (end_time - start_time).toSec() * 1000.0, current_node_ptr->g_score_ * resolution_);

            return true;
        }

        GetNeighborNodes(current_node_ptr, neighbor_nodes, neighbor_edge_cost);

        for (unsigned int i = 0; i < neighbor_nodes.size(); ++i) {
            neighbor_node_ptr = neighbor_nodes[i];

            if (neighbor_node_ptr->status_ == NODE_STATUS::NOT_VISITED) {
                neighbor_node_ptr->g_score_ = current_node_ptr->g_score_ + neighbor_edge_cost[i];
                neighbor_node_ptr->f_score_ =
                        GetHeuristicValue(neighbor_node_ptr, goal_node_ptr) + neighbor_node_ptr->g_score_;
                neighbor_node_ptr->parent_node_ = current_node_ptr;
                neighbor_node_ptr->status_ = NODE_STATUS::IN_OPENSET;
                openset_.insert(std::make_pair(neighbor_node_ptr->f_score_, neighbor_node_ptr));
                continue;
            } else if (neighbor_node_ptr->status_ == NODE_STATUS::IN_OPENSET) {
                double g_score_temp = current_node_ptr->g_score_ + neighbor_edge_cost[i];
                if (neighbor_node_ptr->g_score_ > g_score_temp) {
                    neighbor_node_ptr->g_score_ = g_score_temp;
                    neighbor_node_ptr->f_score_ = g_score_temp + GetHeuristicValue(neighbor_node_ptr, goal_node_ptr);
                    neighbor_node_ptr->parent_node_ = current_node_ptr;

                    auto map_iter = openset_.begin();
                    for (; map_iter != openset_.end(); map_iter++) {
                        if (map_iter->second->grid_index_ == neighbor_node_ptr->grid_index_) {
                            openset_.erase(map_iter);
                            openset_.insert(std::make_pair(neighbor_node_ptr->f_score_, neighbor_node_ptr));
                            break;
                        }
                    }
                }
                continue;
            } else { // In closeset! In fact, this judgment will never take effect,
                // but for code consistency, it is retained!
                continue;
            }
        }

        ros::Time end_time = ros::Time::now();

        //this search timed out!
        if ((end_time - start_time).toSec() * 1000.0 > 2000) {
            ROS_WARN_STREAM("Time out! more than 2000ms");
            return false;
        }
    }

    return false;
}

double AStar2D::GetHeuristicValue(GridNodePtr<2> node_1_ptr, GridNodePtr<2> node_2_ptr) const {
    return ManhattanHeu(node_1_ptr->grid_index_.cast<double>(), node_2_ptr->grid_index_.cast<double>());
}

double AStar2D::ManhattanHeu(const Vec2d &coord_1, const Vec2d &coord_2) const {
    return (coord_1 - coord_2).lpNorm<1>() * tie_breaker_;
}

__attribute__((unused)) double AStar2D::EuclideanHeu(const Vec2d &coord_1, const Vec2d &coord_2) const {
    return (coord_1 - coord_2).norm() * tie_breaker_;
}

__attribute__((unused)) double AStar2D::DiagonalHeu(const Vec2d &coord_1, const Vec2d &coord_2) const {
    double dx = std::fabs(coord_2[0] - coord_1[0]);
    double dy = std::fabs(coord_2[1] - coord_1[1]);

    double h;
    h = -0.5857864 * std::min(dx, dy) + (dx + dy);

    return tie_breaker_ * h;
}

TypeVectorVecd<2> AStar2D::GetPath() const {
    TypeVectorVecd<2> path;
    std::vector<GridNodePtr<2>> temp_nodes;

    GridNodePtr<2> grid_node_ptr = terminal_node_ptr_;
    while (grid_node_ptr != nullptr) {
        temp_nodes.emplace_back(grid_node_ptr);
        grid_node_ptr = grid_node_ptr->parent_node_;
    }

    for (const auto &node: temp_nodes) {
        path.emplace_back(node->coordinate_);
    }

    std::reverse(path.begin(), path.end());

    return path;
}

void AStar2D::SetObstacle(const double pt_x, const double pt_y) {
    if (pt_x < x_lower_ || pt_x > x_upper_ ||
        pt_y < y_lower_ || pt_y > y_upper_) {
        return;
    }

    int grid_index_x = static_cast<int>((pt_x - x_lower_) / resolution_);
    int grid_index_y = static_cast<int>((pt_y - y_lower_) / resolution_);

    map_data_[grid_index_y + grid_index_x * GRID_X_SIZE_] = 1;
}

void AStar2D::GetNeighborNodes(const GridNodePtr<2> &curr_node_ptr, std::vector<GridNodePtr<2>> &neighbor_nodes,
                               std::vector<double> &neighbor_edge_costs) {
    neighbor_nodes.clear();
    neighbor_edge_costs.clear();

    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            Eigen::Vector2i neighbor_node_index = curr_node_ptr->grid_index_ + Eigen::Vector2i(i, j);

            if (!allow_diagonal_ && std::abs(i) == std::abs(j)) {
                continue;
            }

            if (neighbor_node_index == curr_node_ptr->grid_index_) {
                continue;
            }

            if (neighbor_node_index[0] < 0 || neighbor_node_index[0] >= GRID_X_SIZE_ ||
                neighbor_node_index[1] < 0 || neighbor_node_index[1] >= GRID_Y_SIZE_) {
                continue;
            }

            if (HasObstacle(neighbor_node_index)) {
                continue;
            }

            GridNodePtr<2> neighbor_node_ptr = grid_node_map_[neighbor_node_index[0]][neighbor_node_index[1]];
            if (neighbor_node_ptr->status_ == NODE_STATUS::IN_CLOSESET) {
                continue;
            }

            neighbor_nodes.emplace_back(neighbor_node_ptr);

            double neighbor_edge_cost = (neighbor_node_ptr->grid_index_.cast<double>() -
                                         curr_node_ptr->grid_index_.cast<double>()).norm();
            neighbor_edge_costs.emplace_back(neighbor_edge_cost);
        }
    }
}

void AStar2D::ReleaseMemory() {
    delete[] map_data_;
    map_data_ = nullptr;

    for (int i = 0; i < GRID_X_SIZE_; ++i) {
        for (int j = 0; j < GRID_Y_SIZE_; ++j) {
            delete grid_node_map_[i][j];
            grid_node_map_[i][j] = nullptr;
        }
        delete[] grid_node_map_[i];
        grid_node_map_[i] = nullptr;
    }

    delete[] grid_node_map_;
    grid_node_map_ = nullptr;

    terminal_node_ptr_ = nullptr;
}

TypeVectorVecd<2> AStar2D::GetVisitedNodeCoord() const {
    TypeVectorVecd<2> visited_nodes_coord;

    for (int i = 0; i < GRID_X_SIZE_; i++) {
        for (int j = 0; j < GRID_Y_SIZE_; ++j) {
            if (grid_node_map_[i][j]->status_ == NODE_STATUS::IN_CLOSESET) {
                visited_nodes_coord.emplace_back(grid_node_map_[i][j]->coordinate_);
            }
        }
    }

    return visited_nodes_coord;
}

double AStar2D::GetResolution() const {
    return resolution_;
}