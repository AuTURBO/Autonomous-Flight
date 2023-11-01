#ifndef JPS_MAP_UTIL_H
#define JPS_MAP_UTIL_H

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/path.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap_server/OctomapServer.h>
#include <map>
#include <jps_basis/data_type.h>

#include <iostream>
#include <Eigen/Eigen>

using namespace Eigen;

namespace JPS {
    //Type ailas -> 1D array map data as Tmap name 
    using Tmap = std::vector<signed char>;

    //MapUtil template class for collision checking 
    template <int Dim> class MapUtil{

        public:

            MapUtil() {} //simple constructor 
            Tmap getMap() { return map_; } // get map data
            bool has_map_() { return has_map;} // check map exist
            decimal_t getRes() { return res_; } // get resolution 
            Veci<Dim> getDim() { return dim_; } // get dimensions 
            Vecf<Dim> getOrigin() { return origin_d_; } // get origin 


    };

    typedef MapUtil<2> OccMapUtil;
    typedef MapUtil<3> VoxelMapUtil;
}

#endif