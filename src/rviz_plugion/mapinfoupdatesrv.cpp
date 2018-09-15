#include"rviz_plugion/mapinfoupdatesrv.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件
#include"loam_velodyne/GlobalMap.h"
#include"loam_velodyne/SaveMap.h"
#include<QDoubleValidator>
#include<QDebug>
namespace mapinfo {
MapInfoUpdateSrv::MapInfoUpdateSrv(QWidget* parent):
  rviz::Panel( parent )
{
  QVBoxLayout* vertical_layout=new QVBoxLayout();

  _check_box=new QCheckBox("all map cloud");
  _check_box->setChecked(false);
  vertical_layout->addWidget(_check_box);

  _line_edit=new QLineEdit("distance thresh");
  QDoubleValidator *pDfValidator = new QDoubleValidator(0.0, 100.0 , 2, _line_edit);
  pDfValidator->setNotation(QDoubleValidator::StandardNotation);
  _line_edit->setValidator(pDfValidator);
  _line_edit->setText(QString("%1").arg(100.00));
  vertical_layout->addWidget(new QLabel("distance thresh"));
  vertical_layout->addWidget(_line_edit);

  _send_distance=new QPushButton("sendDis");
  vertical_layout->addWidget(_send_distance);

  _save_map_button=new QPushButton("saveMap");

  _line_resolution=new QLineEdit("saveMapRes");
  QDoubleValidator *pDfValidatorRes = new QDoubleValidator(0.0, 1.0 , 2, _line_resolution);
  pDfValidatorRes->setNotation(QDoubleValidator::StandardNotation);
  _line_resolution->setValidator(pDfValidatorRes);
  _line_resolution->setText(QString("%1").arg(0.4));

  _line_destination=new QLineEdit("saveMapDes");
  _line_destination->setText(QString::fromStdString("/home/mameng/catkin_loam/map/map.pcd"));

  vertical_layout->addWidget(new QLabel("map resolution"));
  vertical_layout->addWidget(_line_resolution);
  vertical_layout->addWidget(new QLabel("save destination"));
  vertical_layout->addWidget(_line_destination);
  vertical_layout->addWidget(_save_map_button);

  QHBoxLayout* layout = new QHBoxLayout();
  layout->addLayout( vertical_layout );
  setLayout( layout );

  connect( _send_distance, SIGNAL(clicked()), this, SLOT( update_distance_thresh() ));
  connect( _save_map_button, SIGNAL(clicked()), this, SLOT( save_map() ));

  _distance_client = _nh.serviceClient<loam_velodyne::GlobalMap>("/setup_map");
  _save_map = _nh.serviceClient<loam_velodyne::SaveMap>("/save_map");
}
void MapInfoUpdateSrv::update_distance_thresh()
{
  loam_velodyne::GlobalMap global_map;
  float distance=100;
  if(!_line_edit->text().isEmpty())
       distance=_line_edit->text().toFloat();
  global_map.request.distance=distance;
  if(_check_box->isChecked())
     global_map.request.isDisGlobalMap=true;
  else
     global_map.request.isDisGlobalMap=false;
  if(_distance_client.call(global_map))
  {
    if(global_map.response.success)
      std::cout<<"set up success"<<std::endl;
    else
      std::cout<<"set up fail"<<std::endl;
  }
}
void MapInfoUpdateSrv::save_map()
{
  std::cout<<"save_map"<<std::endl;
  loam_velodyne::SaveMap srv;
  srv.request.destination="./tmp";
  srv.request.resolution=0.2;
  if(!_line_destination->text().isEmpty())
    srv.request.destination = _line_destination->text().toStdString();
  if(!_line_resolution->text().isEmpty())
    srv.request.resolution = _line_resolution->text().toFloat();
  if (_save_map.call(srv))
  {
    if(srv.response.success)
    {
        std::cout<<"success to save pointcloud"<<std::endl;
    }
    else
    {
        std::cout<<"fail to save pointcloud"<<std::endl;
    }
  }
  else
  {
    std::cout<<"fail to save pointcloud"<<std::endl;
  }
}
void MapInfoUpdateSrv::save( rviz::Config config ) const{}
void MapInfoUpdateSrv::load( const rviz::Config& config ){}
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapinfo::MapInfoUpdateSrv,rviz::Panel )
