#include "basic.h"
#include "Detection.hpp"
#include "Geo_Proc.hpp"

void Detail_AR_Main(Mat& input, Mat& output){
<<<<<<< HEAD

=======
    
>>>>>>> f8462fda59353253e572f8ba133eb66e390ce58b

    Mat img = input; // for android
    Detection detect;
    Geo_Proc geo_proc;

<<<<<<< HEAD

=======
        
>>>>>>> f8462fda59353253e572f8ba133eb66e390ce58b
        if(img.cols>img.rows){
            resize(img, img, Size(1000,562));
            geo_proc.Set_Device_Dir(true); // 가로방향
        }
<<<<<<< HEAD
        else{
=======
        else{  
>>>>>>> f8462fda59353253e572f8ba133eb66e390ce58b
            resize(img, img, Size(562,1000));
            geo_proc.Set_Device_Dir(false); // 세로방향
        }



        vector<Point2i> corners;
<<<<<<< HEAD
        vector<Point2i> balls_center;
        vector<int> ball_color_ref;

        detect.Set_Image(img);
=======
        vector<Point2i> balls_center;  
        vector<int> ball_color_ref;

        detect.Set_Image(img, false);    //  android 에서는 false로 설정한다.
>>>>>>> f8462fda59353253e572f8ba133eb66e390ce58b


        int Corners_failed = detect.Detect_Billiard_Corners(corners);
        int Balls_failed = detect.Detect_Billirad_Balls(balls_center, ball_color_ref);

<<<<<<< HEAD

=======
        
>>>>>>> f8462fda59353253e572f8ba133eb66e390ce58b
        geo_proc.Cam_and_Balls_3D_Loc(corners, balls_center, ball_color_ref);
        geo_proc.Draw_Virtual_3D_Obj(img);


        //detect.Draw_Balls(img, balls_center, ball_color_ref);
        //detect.Draw_Corners(img, corners);

        output = img; // for android
        detect.Clear_prev_frame_info();
<<<<<<< HEAD

=======
    
>>>>>>> f8462fda59353253e572f8ba133eb66e390ce58b
}