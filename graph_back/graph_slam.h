#ifndef GRAPH_SLAM_H
#define GRAPH_SLAM_H
#include"edge_se3_plane.h"
#include"g2o/core/sparse_optimizer.h"
#include<g2o/core/optimization_algorithm_factory.h>
#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
namespace g2o {
    class EdgeSe3Plane;
    class VertexSE3;
    class VertexPlane;
    class VertexPointXYZ;
    class EdgeSE3;
}
namespace loam {
class GraphSlam
{
public:
    using Ptr=std::shared_ptr<GraphSlam>;
    GraphSlam();
    ~GraphSlam();
    g2o::VertexSE3* addVertexSE3(const Eigen::Isometry3d& pose);
    g2o::VertexPlane* addVertexPlane(const Eigen::Vector4d& plane);
    void addEdgeSE3Plane(g2o::VertexSE3 *se3_node, const Eigen::Vector4d &floor_coffes,const Eigen::MatrixXd information);
    g2o::EdgeSE3* addEdgeSE3(g2o::VertexSE3* node1,g2o::VertexSE3* node2,const g2o::Isometry3D& relative_pose,const Eigen::MatrixXd information);
    void optimization();
public:
    g2o::VertexPlane* plane_node;
    std::unique_ptr<g2o::SparseOptimizer> graph;
};

}
#endif // GRAPH_SLAM_H
