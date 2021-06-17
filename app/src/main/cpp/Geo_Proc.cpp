#include "Geo_Proc.hpp"
#include "add_ons.h"
#include "basic.h"
#include <android/log.h>

Geo_Proc::Geo_Proc(int f_len) : ball_rad(33), table_depth(40), edge_thickness(52), B_W(1500), B_H(2730) {   // 입력 영상의 픽셀을 고려해야함.
    double * intrinsic_para;

    double f = f_len;
    double cx = 500;
    double cy = 300;

    if(device_dir) // 가로방향 // {fx, 0, cx, 0, fy, cy, 0, 0, 1};
        intrinsic_para = new double[9]{f, 0, cx, 0, f, cy, 0, 0, 1};
    else
        intrinsic_para = new double[9]{f, 0, cy, 0, f, cx, 0, 0, 1};

    INTRINSIC = Mat(3,3, CV_64FC1, intrinsic_para);

    distCoeffs = Mat();
    //Mat_<double>(4, 1) << 0.040982,  -0.026665,  -0.007009,  0.000758 ;


    world_table_outside_corners.resize(4);

    // 중심을 기준으로 왼쪽 위부터 저장
    
    world_table_outside_corners[1] = Point3d(B_W+edge_thickness, -edge_thickness, table_depth);
    world_table_outside_corners[2] = Point3d(B_W+edge_thickness, B_H+edge_thickness, table_depth);
    world_table_outside_corners[3] = Point3d(-edge_thickness, B_H+edge_thickness, table_depth);
    world_table_outside_corners[0] = Point3d(-edge_thickness, -edge_thickness, table_depth);

}

void Geo_Proc::Set_Device_Dir(bool dir){
    device_dir = dir;
}


int Geo_Proc::Find_Balls_3D_Loc(vector<Point2i>& input_corners, vector<Point2i>& balls_center, vector<int>& ball_color_ref,
vector<Point2i>& wor_ball_cen, bool update)
{
    if(input_corners.size() != 4)   // 코너가 4개일 경우에만 처리.
        return -1;
    
    Mat in_rvec, in_tvec;
    vector<Point2d> output_corners(4);
    
    bool pose_flag = Cam_Pos_with_Four_Corners(input_corners, output_corners, in_rvec, in_tvec, false);

    if(pose_flag){   // 카메라 pose를 구할수 있다면  포즈 정보 업데이트.
        img_corners = output_corners;
        rvec = in_rvec;
        tvec = in_tvec;
    }
    else
        return -1; // 카메라 pose를 알 수 없으므로 공 위치파악 불가.


    // 구해진 카메라 pose를 이용하여 각 공의 world 위치 계산
    Mat R;
    Rodrigues(rvec, R);
    Mat R_inv = R.inv();
    int b_n = balls_center.size();
    vector<Vec3d> b_c_vec(b_n);

    for(int i=0; i<b_n ; i++){  // 이미지 위의 점을 homogenous vector 로 변환
        b_c_vec[i] = Vec3d((double)balls_center[i].x, (double)balls_center[i].y, 1.0);
    }

    Mat IN_PTS((int)b_c_vec.size(), 3, CV_64F, b_c_vec.data());
    
    Mat A =  R_inv*INTRINSIC.inv()*IN_PTS.t();
    Mat TVEC;
    repeat(tvec, 1, b_n, TVEC);
    Mat B = R_inv*TVEC; 


    int hei = ball_rad;
    vector<Point2i> wor_ball(b_n);
    
    for(int i=0; i<b_n ; i++){
        double s =  (hei + B.ptr<double>(2)[i]) /( A.ptr<double>(2)[i] + 1e-8);

        int x = (int)( s * A.ptr<double>(0)[i] - B.ptr<double>(0)[i]);
        int y = (int)( s * A.ptr<double>(1)[i] - B.ptr<double>(1)[i]);
        wor_ball[i] = Point2i(x,y);
    }


    // put the output
    wor_ball_cen = wor_ball; 

    if(update){   // world 공 좌표 업데이트
        wor_ball_loc.resize(b_n);
        for(int i=0; i<b_n ; i++){
            wor_ball_loc[i] = Point3i(wor_ball[i].x, wor_ball[i].y, 0);
        }
    
        world_ball_color_ref = ball_color_ref;
    }
    
    return 0;
}


void Geo_Proc::Draw_Obj_on_Templete(){
    Ball_and_Sol_templete = Mat(B_H,B_W, CV_8UC3, Scalar(0,0,0));   // for drawing

    int b_n = wor_ball_loc.size();


    
    for(int i=0; i<b_n ; i++){
        Scalar color;
        if(world_ball_color_ref[i] == 0)
            color = Scalar(0,0,255);
        else if(world_ball_color_ref[i] == 1 )
            color = Scalar(0, 255, 255);
        else
            color = Scalar(255,255,255);

        circle(Ball_and_Sol_templete, Point(wor_ball_loc[i].x, wor_ball_loc[i].y), ball_rad, color, 10, 8, 0);
    }

    //resize(Ball_and_Sol_templete, Ball_and_Sol_templete,Size(500, 1000));
    //imshow("result", Ball_and_Sol_templete);

}


void Geo_Proc::Draw_3D_Templete_on_Img(Mat& img){

    // *****당구공아래에 원 표시*****

    Mat output;
    Mat H = getPerspectiveTransform(H_wor_pts, H_img_pts);

    warpPerspective(Ball_and_Sol_templete, output, H, img.size(), INTER_NEAREST);

    Mat mask = (output != Scalar(0,0,0));
    output.copyTo(img, mask);


    // 공 바닥 중점 표시 + 이거랑 코너도 안쪽 코너로 해야하는것 잊지 말기!!
    /*
    int size = wor_ball_loc.size();
    for(int i=0; i<size ; i++){
        object_wor_pt[i] = Point3d(wor_ball_loc[i].x, wor_ball_loc[i].y, 0);
    }

    projectPoints(object_wor_pt, rvec, tvec, INTRINSIC, distCoeffs, object_img_pt);
    for(int i=0; i<size; i++){
        circle(img, Point(object_img_pt[i].x, object_img_pt[i].y), 2, Scalar(255,255,255), 2, 8, 0);
    }
    */
}


void Geo_Proc::Draw_Object(Mat& img){

    vector<Point3d> object_wor_pt;
    vector<Point2d> object_img_pt;

    // 3축 그리기
    object_wor_pt.resize(4);
    object_wor_pt[0] = Point3d(200,0,0);
    object_wor_pt[1] = Point3d(0,200,0);
    object_wor_pt[2] = Point3d(0,0,200);
    object_wor_pt[3] = Point3d(0,0,0); // center

    projectPoints(object_wor_pt, rvec, tvec, INTRINSIC, distCoeffs, object_img_pt);
    
    line(img, object_img_pt[3], object_img_pt[0], Scalar(255,0,0), 5);
    line(img, object_img_pt[3], object_img_pt[1], Scalar(0,255,0), 5);
    line(img, object_img_pt[3], object_img_pt[2], Scalar(0,0,255), 5);



    // 코너 그대로 사영시키기
    projectPoints(world_table_outside_corners, rvec, tvec, INTRINSIC, distCoeffs, object_img_pt);

    for(int i=0; i<4; i++){
        circle(img, Point(img_corners[i].x, img_corners[i].y), 5, Scalar(0, 0, i*80), 2, 8, 0);
    }
    for(int i=0; i<4; i++){
        circle(img, Point(object_img_pt[i].x, object_img_pt[i].y), 10, Scalar(0, i*80, i*80), 2, 8, 0);
    }




    // 안쪽 테두리 표시
    object_wor_pt.resize(4);

    object_wor_pt[0] = Point3d(0,0,0);
    object_wor_pt[1] = Point3d(B_W,0,0);
    object_wor_pt[2] = Point3d(B_W,B_H,0);
    object_wor_pt[3] = Point3d(0,B_H,0);

    projectPoints(object_wor_pt, rvec, tvec, INTRINSIC, distCoeffs, object_img_pt);

    for(int i=0; i<4; i++){
        line(img, object_img_pt[i%4], object_img_pt[(i+1)%4], Scalar(0,255,255), 2);
    }

}



void Geo_Proc::Clear_prev_frame_info(){
    img_corners.clear();
}