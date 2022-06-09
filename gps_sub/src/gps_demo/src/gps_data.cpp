#include <../include/gps_data.hpp>
#include "glog/logging.h"

//静态成员变量必须要类外初始化
double GPSData::origin_longitude = 0.0;
double GPSData::origin_latitude = 0.0;
double GPSData::origin_altitude = 0.0;
bool GPSData::origin_position_inited = false;
GeographicLib::LocalCartesian GPSData::geo_converter;

void GPSData::InitOriginPosition(){
    geo_converter.Reset(latitude, longitude, altitude);

    origin_longitude = longitude;
    origin_latitude = latitude;
    origin_altitude = altitude;

    origin_position_inited = true;
}

void GPSData::UpdateXYZ(){
    if(!origin_position_inited){
        LOG(WARNING) << "GeoConverter has not set origin position";
    }
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}
