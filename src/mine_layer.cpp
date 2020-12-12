#include<deminer/mine_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mine_area::MineLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace mine_area
{

MineLayer::MineLayer() {}

void MineLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &MineLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void MineLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void MineLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void MineLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
  unsigned int mx;
  unsigned int my;
  if(worldToMap(mark_x, mark_y, mx, my)){
    setCost(mx, my, LETHAL_OBSTACLE);
  }
  
  *min_x = std::min(*min_x, mark_x);
  *min_y = std::min(*min_y, mark_y);
  *max_x = std::max(*max_x, mark_x);
  *max_y = std::max(*max_y, mark_y);
}

void MineLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]); 
    }
  }
}

// MineLayer::MineLayer(){mbx=0;mby=0;mkx=0;mky=0;}
// MineLayer::~MineLayer(){}

// void MineLayer::onInitialize()
// {
//   ros::NodeHandle nh("~/" + name_);
//   default_value_ = NO_INFORMATION;
//   matchSize();

//   sub_ = nh.subscribe("/mine_location", 1, &MineLayer::Callback, this);
//   sub2_ = nh.subscribe("/mine_settings",1,&MineLayer::setValue, this);

//   dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
//   dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
//       &MineLayer::reconfigureCB, this, _1, _2);
//   dsrv_->setCallback(cb);
// }
// void MineLayer::setValue(const geometry_msgs::Point::ConstPtr& data)
// {
//   point_.x = data->x; //Size
//   point_.y = data->y; //Clear
//   point_.z = data->z; 
// }
// void MineLayer::Callback(const geometry_msgs::Point::ConstPtr& world)
// {
//   createCircle(world->x, world->y, point_.x, world->z);

// }

// void MineLayer::matchSize()
// {
//   Costmap2D* master = layered_costmap_->getCostmap();
//   resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
//             master->getOriginX(), master->getOriginY());
// }


// void MineLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
// {
//   enabled_ = config.enabled;
// }


// void MineLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
//                                            double* min_y, double* max_x, double* max_y)
// {
//   Costmap2D* master = layered_costmap_->getCostmap();

//   *min_x = -(master->getSizeInCellsX()/(master->getResolution()));
//   *min_y = -(master->getSizeInCellsY()/(master->getResolution()));
//   *max_x = (master->getSizeInCellsX()/(master->getResolution()));
//   *max_y = (master->getSizeInCellsY()/(master->getResolution()));
// }

// void MineLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
//                                           int max_j)
// {
//   if (!enabled_)
//     return;

//   for (int j = min_j; j < max_j; j++)
//   {
//     for (int i = min_i; i < max_i; i++)
//     {
//       int index = getIndex(i, j);
//       if (costmap_[index] == NO_INFORMATION)
//         continue;
//       master_grid.setCost(i, j, costmap_[index]);
//     }
//   }
// }

// void MineLayer::createCircle(double x_center,double y_center, int r, bool operation)
// {
//     Costmap2D* master = layered_costmap_->getCostmap();
//     if (!point_.y)
//     {
//         for (int i=0; i<r; i++)
//         {
//           for (int j=0; j<r; j++)
//             {
//               if(worldToMap((x_center+(i*master->getResolution())),(y_center+(master->getResolution()*j)),mx, my))
//                  {
//                   if (mbx<mx) mbx=mx;
//                   if (mkx>mx) mkx=mx;
//                   if (mby<my) mby=my;
//                   if (mky>my) mky=my;

//                   if(!operation)
//                       setCost(mx,my,LETHAL_OBSTACLE);
//                   else
//                       setCost(mx,my,NO_INFORMATION);
//                  }
//             }
//         }
//    }
//    else
//    {
//       for (int i=mkx; i<=mbx; i++)
//       {
//          for (int j=mky; j<=mby; j++)
//            setCost(i,j,NO_INFORMATION);
//       }
//    }
// }
} // end namespace