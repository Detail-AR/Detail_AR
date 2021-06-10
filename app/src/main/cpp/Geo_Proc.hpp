#ifndef Geo_Proc_h
#define Geo_Proc_h

#include "basic.h"

class Geo_Proc
{
    private:
    
    Mat Ball_and_Sol_templete;
    vector<Point3d> world_table_outside_corners;
    vector<Point2d> img_corners; // == corners
    vector<Point3i> wor_ball_loc;
    vector<int> world_ball_color_ref;

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
    
    Geo_Proc(int f_len);
    void Set_Device_Dir(bool dir);
    bool Cam_and_Balls_3D_Loc(vector<Point2i>& corners, vector<Point2i>& balls_center, vector<int>& ball_color_ref,
    vector<Point2i>& wor_ball_cen);

    // for test
    void Draw_Obj_on_Templete();
    void Draw_3D_Templete_on_Img(Mat& img);

    //clear
    void Clear_prev_frame_info();

};

#endif