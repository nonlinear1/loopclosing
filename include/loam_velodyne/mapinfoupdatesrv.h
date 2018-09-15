#ifndef MAPINFOUPDATESRV_H
#define MAPINFOUPDATESRV_H
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include<QCheckBox>
#include<QPushButton>
namespace loam {
class MapInfoUpdateSrv:public rviz::Panel
{
Q_OBJECT
public:
  MapInfoUpdateSrv(QWidget* parent);

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;
public:
  void update_checkbox(int stage);
  void update_distance_thresh();
private:
  ros::NodeHandle _nh;
  ros::ServiceClient _distance_client;
  ros::ServiceClient _checkbox_client;
  ros::ServiceClient _save_map;

  QLineEdit* _line_edit;
  QCheckBox* _check_box;
  QPushButton* _send_distance;
  QPushButton* _save_map_button;
};
}
#endif // MAPINFOUPDATESRV_H
