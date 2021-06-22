#include "basic.h"
#include "Detection.hpp"
#include "Geo_Proc.hpp"
#include <android/log.h>

void Detail_AR_Main(Mat& input, Mat& output){


    Mat img = input; // for android
    static Detection detect;
    static Geo_Proc geo_proc(700);


    if(img.cols>img.rows){
        resize(img, img, Size(1000,562));
        geo_proc.Set_Device_Dir(true, img); // 가로방향
    }
    else{
        resize(img, img, Size(562,1000));
        geo_proc.Set_Device_Dir(false, img); // 세로방향
    }


    detect.Set_Image(img, false);    //  android 에서는 false로 설정한다.

        // 루프 시작 지점 

        vector<Point2i> corners;    // need clear
        vector<Point2i> balls_center;  
        vector<int> ball_color_ref;
        vector<Point2i> wor_ball_cen;
        int Corners_num;
        int Balls_num;
        int can_find_pose=-1;
        bool find_ball_loc = false; 
        static int situation = 1;

        
        // 상황1
        if(situation == 1){

            Corners_num = detect.Detect_Billiard_Corners(corners);
        
            Balls_num = detect.Detect_Billirad_Balls(balls_center, ball_color_ref);
       
            // 제일 처음 모든 공의 위치를 파악하기 위함.
            if(Corners_num == 4 && Balls_num >= 4){  // 4개의 코너, 4개의 공 모두가 감지 되어야함.
                can_find_pose = geo_proc.Find_Balls_3D_Loc(corners, balls_center, ball_color_ref, wor_ball_cen, true);
                           
                //if can_fidnd_pose is true;  false -> continue;

                // here, we need solution class --->>   input: wor_ball_cen, ball_color_ref  output: solution arrow.
                if(can_find_pose != -1){
                    geo_proc.Draw_Obj_on_Templete();   // draw circles under the balls and draw solution arrows on the table
                    geo_proc.Draw_3D_Templete_on_Img(img);
                    find_ball_loc = true;
                }
            }

        }

                      
        // 상황2
        //if find_ball_loc is true.

        if(situation == 2){
            Corners_num = detect.Detect_Billiard_Corners(corners);
            Balls_num = detect.Detect_Billirad_Balls(balls_center, ball_color_ref);

            if(Corners_num != -1 || Balls_num != -1 ){   // 공, 코너 둘중 하나라도 감지해야함.
                can_find_pose = geo_proc.Find_Cam_Pos(corners, balls_center, ball_color_ref);

                if(can_find_pose != -1){   // 포즈를 추정할 수 있다면
                    geo_proc.Draw_3D_Templete_on_Img(img);

                }
            }

            //geo_proc.Remove_Drawing_Info();
        }
        // for debugging

        if(can_find_pose == 0)
            geo_proc.Draw_Object(img);

        if(Balls_num != 0)
            detect.Draw_Balls(img, balls_center, ball_color_ref);

        if(Corners_num != 0){
            detect.Draw_Corners(img, corners);
        }

        // output = img; // for android
        detect.Clear_prev_frame_info();
        //geo_proc.Clear_prev_frame_info();

        if(find_ball_loc) {
            situation = 2;
        }


    output = img; // for android
    detect.Clear_prev_frame_info();

}