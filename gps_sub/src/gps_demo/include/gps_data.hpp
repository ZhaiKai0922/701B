#include <deque>
#include <GeographicLib/LocalCartesian.hpp>

class GPSData{
    public:
    double time = 0.0;
    double longitude = 0.0;       //经度
    double latitude = 0.0;          //纬度
    double altitude = 0.0;          //高度

    double local_E = 0.0;           //坐标系转换，经纬高转换为ENU（东北天）
    double local_N = 0.0;
    double local_U = 0.0;

    static double origin_longitude;  //初始化
    static double origin_latitude;
    static double origin_altitude;
    
    private:
    static GeographicLib::LocalCartesian geo_converter;
    static bool origin_position_inited;

    public:
    void InitOriginPosition();          //初始化
    void UpdateXYZ();                        //转换为ENU
    
};