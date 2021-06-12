#include "basic.h"
#include "Detection.hpp"
#include "Geo_Proc.hpp"

void Detail_AR_Main(Mat& input, Mat& output){


    Mat img = input; // for android
    Detection detect;
    Geo_Proc geo_proc(700);


    if(img.cols>img.rows){
        resize(img, img, Size(1000,562));
        geo_proc.Set_Device_Dir(true); // 가로방향
    }
    else{
        resize(img, img, Size(562,1000));
        geo_proc.Set_Device_Dir(false); // 세로방향
    }

    detect.Set_Image(img, false);    //  android 에서는 false로 설정한다.


    vector<Point2i> corners;
    vector<Point2i> balls_center;
    vector<int> ball_color_ref;
    vector<Point2i> wor_ball_cen;



    int Corners_failed = detect.Detect_Billiard_Corners(corners);
    int Balls_failed = detect.Detect_Billirad_Balls(balls_center, ball_color_ref);

    bool can_find_pose;
        if((int)corners.size() == 4){  
            can_find_pose = geo_proc.Cam_and_Balls_3D_Loc(corners, balls_center, ball_color_ref, wor_ball_cen);
            
            // here, we need solution class    input: wor_ball_cen,  output: solution arrow.

            geo_proc.Draw_Obj_on_Templete();   // draw circles under the balls and draw solution arrows on the table

            if(can_find_pose)
                geo_proc.Draw_3D_Templete_on_Img(img);
        }

        if(Balls_failed == 1)
            detect.Draw_Balls(img, balls_center, ball_color_ref);

        if(Corners_failed != 0){
            detect.Draw_Corners(img, corners);
        }



    output = img; // for android
    detect.Clear_prev_frame_info();
    geo_proc.Clear_prev_frame_info();
    corners.clear();

}