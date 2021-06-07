#include "basic.h"
#include "Detection.hpp"
#include "Geo_Proc.hpp"

void Detail_AR_Main(Mat& input, Mat& output){


    Mat img = input; // for android
    Detection detect;
    Geo_Proc geo_proc;


    if(img.cols>img.rows){
        resize(img, img, Size(1000,562));
        geo_proc.Set_Device_Dir(true); // 가로방향
    }
    else{
        resize(img, img, Size(562,1000));
        geo_proc.Set_Device_Dir(false); // 세로방향
    }



    vector<Point2i> corners;
    vector<Point2i> balls_center;
    vector<int> ball_color_ref;

    detect.Set_Image(img, false);    //  android 에서는 false로 설정한다.


    int Corners_failed = detect.Detect_Billiard_Corners(corners);
    int Balls_failed = detect.Detect_Billirad_Balls(balls_center, ball_color_ref);

        
        if((int)corners.size() == 4){
            geo_proc.Cam_and_Balls_3D_Loc(corners, balls_center, ball_color_ref);
            geo_proc.Draw_Virtual_3D_Obj(img);
        }

        if(Balls_failed == 1)
            detect.Draw_Balls(img, balls_center, ball_color_ref);

        if(Corners_failed != 0)
            detect.Draw_Corners(img, corners);


    output = img; // for android
    detect.Clear_prev_frame_info();

}