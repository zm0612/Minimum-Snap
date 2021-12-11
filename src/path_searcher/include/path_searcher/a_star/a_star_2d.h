//
// Created by Zhang Zhimeng on 2021/11/25.
//

#ifndef PATH_SEARCHER_A_STAR_2D_H
#define PATH_SEARCHER_A_STAR_2D_H

#include "path_searcher/utility/type.h"
#include "path_searcher/utility/grid_node.h"

class AStar2D;

typedef typename std::shared_ptr<AStar2D> AStar2DPTR;

class AStar2D {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AStar2D() = default;

    AStar2D(double x_lower, double x_upper, double y_lower,
            double y_upper, double resolution, bool allow_diagonal = true);

    ~AStar2D();

    void Init(double x_lower, double x_upper, double y_lower,
              double y_upper, double resolution, bool allow_diagonal = true);

    void Reset();

    bool Search(const Eigen::Vector2d &start_pt, const Eigen::Vector2d &goal_pt);

    void SetObstacle(double pt_x, double pt_y);

    TypeVectorVecd<2> GetPath() const;

    Eigen::Vector2d CoordinateRounding(const Eigen::Vector2d &pt);

    TypeVectorVecd<2> GetVisitedNodeCoord() const;

    double GetResolution() const;

protected:
    inline Eigen::Vector2d GridIndex2Coordinate(const Eigen::Vector2i &grid_index) const;

    inline Eigen::Vector2i Coordinate2GridIndex(const Eigen::Vector2d &pt) const;

    inline bool HasObstacle(const Eigen::Vector2i &grid_index) const;

    void GetNeighborNodes(const GridNodePtr<2> &curr_node_ptr,
                          std::vector<GridNodePtr<2>> &neighbor_nodes,
                          std::vector<double> &neighbor_edge_costs);

    inline double GetHeuristicValue(GridNodePtr<2> node_1_ptr, GridNodePtr<2> node_2_ptr) const;

    inline double ManhattanHeu(const Vec2d &coord_1, const Vec2d &coord_2) const;

    __attribute__((unused)) inline bool IsFree(const Eigen::Vector2i &grid_index) const;

    __attribute__((unused)) inline double DiagonalHeu(const Vec2d &coord_1, const Vec2d &coord_2) const;

    __attribute__((unused)) inline double EuclideanHeu(const Vec2d &coord_1, const Vec2d &coord_2) const;

    void ReleaseMemory();

protected:
    uint8_t *map_data_ = nullptr; // 1: occupied 0: free
    double resolution_{};
    int GRID_X_SIZE_{}, GRID_Y_SIZE_{};

    double x_lower_{}, x_upper_{};
    double y_lower_{}, y_upper_{};

    double tie_breaker_{};
    bool allow_diagonal_ = true;

    GridNodePtr<2> terminal_node_ptr_ = nullptr;
    GridNodePtr<2> **grid_node_map_ = nullptr;
    std::multimap<double, GridNodePtr<2>> openset_;
};

#endif //PATH_SEARCHER_A_STAR_2D_H
