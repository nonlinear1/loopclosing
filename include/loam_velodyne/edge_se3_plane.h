#ifndef EDGE_SE3_PLANE_H
#define EDGE_SE3_PLANE_H
#include<g2o/types/slam3d/vertex_se3.h>
#include<g2o/types/slam3d_addons/vertex_plane.h>
#include<g2o/types/slam3d/edge_se3.h>
namespace g2o {
class EdgeSe3Plane:public g2o::BaseBinaryEdge<3,g2o::Plane3D,g2o::VertexSE3,g2o::VertexPlane>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSe3Plane():
    BaseBinaryEdge<3, g2o::Plane3D, g2o::VertexSE3, g2o::VertexPlane>()
  {}
  virtual void computeError () override
  {
    const g2o::VertexSE3 *se3=static_cast<g2o::VertexSE3*>(_vertices[0]);
    const g2o::VertexPlane *vetexplane=static_cast<g2o::VertexPlane*>(_vertices[1]);
    Eigen::Isometry3d pose=se3->estimate().inverse();
    g2o::Plane3D wPlane=pose*vetexplane->estimate();
    _error=wPlane.ominus(_measurement);
  }
  virtual void setMeasurement(const g2o::Plane3D& p) override
  {
    _measurement=p;
  }
  virtual bool read(std::istream& is) override{}
  //! write the vertex to a stream
  virtual bool write(std::ostream& os) const override{}
};
}
#endif // EDGE_SE3_PLANE_H
