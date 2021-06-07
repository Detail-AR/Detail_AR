#ifndef Geo_Proc_h
#define Geo_Proc_h

#include "basic.h"

class Geo_Proc
{
private:

    Mat world_table_and_ball;
    vector<Point3f> world_table_outside_corners;
    vector<Point2f> img_corners; // == corners
    vector<Point3i> world_ball_loc;

    Mat INTRINSIC;
    Mat distCoeffs;
    Mat rvec, tvec;

    bool device_dir; // true: 가로 false: 세로

    // 실제 당구 시스템의 규격을 저장하는 변수
    int ball_rad;
    int table_depth;
    int edge_thickness;
    int B_W;
    int B_H;


public:

    Geo_Proc();
    void Set_Device_Dir(bool dir);
    void Cam_and_Balls_3D_Loc(vector<Point2i>& corners, vector<Point2i>& balls_center,  vector<int>& ball_color_ref);

    // for test
    void Draw_Virtual_3D_Obj(Mat& img);

};

#endif