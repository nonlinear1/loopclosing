#include"loam_velodyne/mapinfoupdatesrv.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件
namespace loam {
MapInfoUpdateSrv::MapInfoUpdateSrv(QWidget* parent):
  rviz::Panel( parent )
{
  QVBoxLayout* vertical_layout=new QVBoxLayout();
  _line_edit=new QLineEdit("distance_thresh");
  vertical_layout->addWidget(_line_edit);

  _check_box=new QCheckBox("all map cloud");
  _check_box->setChecked(true);
  vertical_layout->addWidget(_check_box);

  _send_distance=new QPushButton("sendDis");
  _save_map_button=new QPushButton("saveMap");
  vertical_layout->addWidget(_send_distance);
  vertical_layout->addWidget(_save_map_button);

  QHBoxLayout* layout = new QHBoxLayout();
  layout->addLayout( vertical_layout );
  setLayout( layout );

  connect( _check_box, SIGNAL(stateChanged(int)), this, SLOT( update_checkbox(int) ));
  connect( _send_distance, SIGNAL(clicked()), this, SLOT( update_distance_thresh() ));

  _checkbox_client = _nh.serviceClient<loam_velodyne::SaveMap>("/save_map");
}
MapInfoUpdateSrv::update_checkbox(int stage)
{
  if(stage==Qt::Checked)
  {

  }
  else
  {

  }
}
}

