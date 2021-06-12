#include "Geo_Proc.hpp"
#include "add_ons.h"
#include "basic.h"

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

bool Geo_Proc::Cam_and_Balls_3D_Loc(vector<Point2i>& input_corners, vector<Point2i>& balls_center, vector<int>& ball_color_ref,
vector<Point2i>& wor_ball_cen)
{
    if(input_corners.size() != 4)   // 코너가 4개일 경우에만 처리.
        return false;
    
    img_corners.resize(4);
    vector<Point2d> reproject_point(4);

    Sort_Corners_Clockwise(input_corners);
    for(int i=0; i<4 ;i++)
        img_corners[i] =input_corners[3-i];   // ** 반시계 방향으로 넣어야 z축이 천장을 향한다


   // 코너가 시계방향으로 돌면서 reprojection 오차가 이 가장 작은 값을 취한다.
   vector<pair<double, int>> dist_with_index(4);
   vector<Point2d> temp[4];

    for(int i=0; i<4 ; i++){
    Clockwise_Permutation(img_corners);
    solvePnP(world_table_outside_corners, img_corners, INTRINSIC, distCoeffs, rvec, tvec);
    projectPoints(world_table_outside_corners, rvec, tvec, INTRINSIC, distCoeffs, reproject_point);

    double dist = double_vector_dist_sum(img_corners, reproject_point);

    dist_with_index[i].first = dist;
    dist_with_index[i].second = i;

    temp[i] = img_corners;
   }


    sort(dist_with_index.begin(), dist_with_index.end());

    for(int i=0; i<4 ;i++){
       // cout<<"<"<<dist_with_index[i].first<<","<<dist_with_index[i].second<<">"<<endl;
    }

    if(dist_with_index[0].first>400)   // 4개의 코너가 잘못되었을 가능성이 높다.
        return false;


    img_corners = temp[dist_with_index[0].second]; // temp-> reprojection  오차가 가장 작은 코너순서
    solvePnP(world_table_outside_corners, img_corners, INTRINSIC, distCoeffs, rvec, tvec);

    
    // 각 공의 3D world 좌표계 기준 위치 계산.
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
        double s =  (hei + B.at<double>(2,i)) /( A.at<double>(2,i) + 1e-8);

        int x = (int)( s * A.at<double>(0,i) - B.at<double>(0,i));
        int y = (int)( s * A.at<double>(1,i) - B.at<double>(1,i));
        wor_ball[i] = Point2i(x,y);
    }



    wor_ball_cen = wor_ball;  // put the output

    wor_ball_loc.resize(b_n);
    for(int i=0; i<b_n ; i++){
        wor_ball_loc[i] = Vec3i(wor_ball[i].x, wor_ball[i].y, hei);
    }
    
    world_ball_color_ref = ball_color_ref;
    
    return true;
}

void Geo_Proc::Draw_Obj_on_Templete(){
    Ball_and_Sol_templete = Mat(B_H,B_W, CV_8UC3);   // for drawing

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

    if(img_corners.size() != 4)
        return;

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


    object_wor_pt.resize(4);

    int depth = 20;
    
    object_wor_pt[0] = Point3d(0,0,0);
    object_wor_pt[1] = Point3d(B_W,0,0);
    object_wor_pt[2] = Point3d(B_W,B_H,0);
    object_wor_pt[3] = Point3d(0,B_H,0);

    projectPoints(object_wor_pt, rvec, tvec, INTRINSIC, distCoeffs, object_img_pt);



    // 안쪽 테두리 표시
    for(int i=0; i<4; i++){
        line(img, object_img_pt[i%4], object_img_pt[(i+1)%4], Scalar(0,255,255), 2);
        //circle(img, Point(object_img_pt[i].x, object_img_pt[i].y), 10, Scalar(0, 255, 255), 2, 8, 0);
    }


    // *****당구공아래에 원 표시*****
    vector<Point2f> table_corner(4);
    table_corner[0] = Point2f(0,0);
    table_corner[1] = Point2f(B_W,0);
    table_corner[2] = Point2f(B_W,B_H);
    table_corner[3] = Point2f(0,B_H);


    vector<Point2f> img_corners_f(4);
    for(int i=0; i<4 ; i++)
        img_corners_f[i] = object_img_pt[i];
    

    Mat output;
    Mat H = getPerspectiveTransform(table_corner, img_corners_f);

    warpPerspective(Ball_and_Sol_templete, output, H, img.size(), INTER_NEAREST);

    Mat mask = (output != Scalar(0,0,0));
    output.copyTo(img, mask);


    // 임의의 직선 그리기

}

void Geo_Proc::Clear_prev_frame_info(){
    img_corners.clear();
}