#include"loam_velodyne/graph_slam.h"
#include<time.h>
G2O_USE_OPTIMIZATION_LIBRARY(csparse)
namespace g2o {
G2O_REGISTER_TYPE(EDGE_SE3_PLANE, EdgeSe3Plane)
}
namespace loam
{
GraphSlam::GraphSlam()
{
  graph.reset(new g2o::SparseOptimizer());
  std::string optim_method="lm_var";
  g2o::OptimizationAlgorithmFactory* optim_algorith_factory=g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty optim_algorith_property;
  g2o::OptimizationAlgorithm *solver=optim_algorith_factory->construct(optim_method,optim_algorith_property);
  graph->setAlgorithm(solver);
  if(!graph->solver())
  {
    std::cout<<"construct solver failed"<<std::endl;
    return;
  }
  std::cout<<"construct solver done"<<std::endl;
  Eigen::Vector4d ground_plane(0,1,0,0);
  //plane_node=addVertexPlane(ground_plane);
  //plane_node->setFixed(true);
}
GraphSlam::~GraphSlam() {
  graph.reset();
}
void GraphSlam::addEdgeSE3Plane(g2o::VertexSE3 *se3_node, const Eigen::Vector4d &floor_coffes
                                ,const Eigen::MatrixXd information)
{
   g2o::EdgeSe3Plane *edge_plane(new g2o::EdgeSe3Plane());
   g2o::Plane3D plane_measurement(floor_coffes);
   edge_plane->setMeasurement(plane_measurement);
   edge_plane->setInformation(information);
   edge_plane->vertices()[0]=se3_node;
   edge_plane->vertices()[1]=plane_node;
   graph->addEdge(edge_plane);
}
  g2o::VertexSE3* GraphSlam::addVertexSE3(const Eigen::Isometry3d &pose)
  {
    g2o::VertexSE3* node(new g2o::VertexSE3());
    node->setId(graph->vertices().size());
    node->setEstimate(pose);
    graph->addVertex(node);
    return node;
  }
  g2o::VertexPlane* GraphSlam::addVertexPlane(const Eigen::Vector4d &plane)
  {
    g2o::VertexPlane* node(new g2o::VertexPlane());
    g2o::Plane3D ground_plane(plane);
    node->setId(graph->vertices().size());
    node->setEstimate(ground_plane);
    graph->addVertex(node);
    return node;
  }
  g2o::EdgeSE3* GraphSlam::addEdgeSE3(g2o::VertexSE3* node1,g2o::VertexSE3* node2
                  ,const g2o::Isometry3D& relative_pose,const Eigen::MatrixXd information)
  {
    g2o::EdgeSE3* se3_edge(new g2o::EdgeSE3());
    se3_edge->vertices()[0]=node1;
    se3_edge->vertices()[1]=node2;
    se3_edge->setMeasurement(relative_pose);
    se3_edge->setInformation(information);
    graph->addEdge(se3_edge);
    return se3_edge;
  }
  void GraphSlam::optimization()
  {
    std::cout<<"nodes:"<<graph->vertices().size()<<"edges:"<<graph->edges().size()<<std::endl;
    if(graph->edges().size()<5) return;
    std::cout<<"pose graph optimization"<<std::endl;
    std::cout<<"nodes:"<<graph->vertices().size()<<"edges:"<<graph->edges().size()<<std::endl;
    std::cout<<"graph init start"<<std::endl;
    graph->initializeOptimization();
    graph->setVerbose(false);

    double chi2 = graph->chi2();
    std::cout << "chi2: (before)" << chi2<<std::endl;
    std::cout<<"graph optim start"<<std::endl;
    std::clock_t start=std::clock();
    std::cout<<"graph optim start 1"<<std::endl;
    graph->optimize(1024);
    std::cout<<"optimization time is:"<<(double)(std::clock()-start)/CLOCKS_PER_SEC*1000<<"ms"<<std::endl;
  }
}
