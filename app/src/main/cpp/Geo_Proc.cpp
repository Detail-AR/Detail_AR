#include "Geo_Proc.hpp"
#include "add_ons.h"
#include "basic.h"

Geo_Proc::Geo_Proc() : ball_rad(33), table_depth(0), edge_thickness(52), B_W(1224), B_H(2448) {   // 입력 영상의 픽셀을 고려해야함.
    double * intrinsic_para;

    //double f = 1543;
    double f = 700;
    double cx = 500;
    double cy = 250;

    if(device_dir) // 가로방향 // {fx, 0, cx, 0, fy, cy, 0, 0, 1};
        intrinsic_para = new double[9]{f, 0, cx, 0, f, cy, 0, 0, 1};
    else
        intrinsic_para = new double[9]{f, 0, cy, 0, f, cx, 0, 0, 1};

    INTRINSIC = Mat(3,3, CV_64FC1, intrinsic_para);

    distCoeffs = Mat_<double>(4, 1) << 0.040982,  -0.026665,  -0.007009,  0.000758 ;


    world_table_and_ball.create(B_H,B_W, CV_8UC3);
    world_table_outside_corners.resize(4);

    // 중심을 기준으로 왼쪽 위부터 저장

    world_table_outside_corners[1] = Point3f(B_W+edge_thickness, -edge_thickness, table_depth);
    world_table_outside_corners[2] = Point3f(B_W+edge_thickness, B_H+edge_thickness, table_depth);
    world_table_outside_corners[3] = Point3f(-edge_thickness, B_H+edge_thickness, table_depth);
    world_table_outside_corners[0] = Point3f(-edge_thickness, -edge_thickness, table_depth);

    /*
    world_table_outside_corners[0] = Point3f(0, 0, 0);
    world_table_outside_corners[1] = Point3f(1000, 0, 0);
    world_table_outside_corners[2] = Point3f(1000, 1000, 0);
    world_table_outside_corners[3] = Point3f(0, 1000, 0);*/


}

void Geo_Proc::Set_Device_Dir(bool dir){
    device_dir = dir;
}

void Geo_Proc::Cam_and_Balls_3D_Loc(vector<Point2i>& corners, vector<Point2i>& balls_center,  vector<int>& ball_color_ref){
    if(corners.size() != 4)
        return;

    img_corners.resize(4);
    vector<Point2f> reproject_point(4);

    Sort_Corners_Clockwise(corners);
    for(int i=0; i<4 ;i++)
        img_corners[i] = corners[i];

    /*
    img_corners[0]=Point2f(200,0);
    img_corners[1]=Point2f(400,100);
    img_corners[2]=Point2f(200,200);
    img_corners[3]=Point2f(0,100);
    */

    //cout<<world_table_outside_corners;

    // 코너가 시계방향으로 돌면서 reprojection 오차가 이 가장 작은 값을 취한다.
    vector<pair<float, int>> dist_with_index(4);
    vector<Point2f> temp[4];

    for(int i=0; i<4 ; i++){
        Clockwise_Permutation(img_corners);
        solvePnP(world_table_outside_corners, img_corners, INTRINSIC, distCoeffs, rvec, tvec);
        projectPoints(world_table_outside_corners, rvec, tvec, INTRINSIC, distCoeffs, reproject_point);

        float dist = float_vector_dist_sum(img_corners, reproject_point);

        dist_with_index[i].first = dist;
        dist_with_index[i].second = i;

        temp[i] = img_corners;
    }


    sort(dist_with_index.begin(), dist_with_index.end());


    img_corners = temp[dist_with_index[0].second]; // temp-> reprojection  오차가 가장 작은 코너순서

    solvePnP(world_table_outside_corners, img_corners, INTRINSIC, distCoeffs, rvec, tvec);
}

void Geo_Proc::Draw_Virtual_3D_Obj(Mat& img){

    if(img_corners.size() != 4)
        return;


    vector<Point3f> object_wor_pt;
    vector<Point2f> object_img_pt;

    // 3축 그리기
    object_wor_pt.resize(4);
    object_wor_pt[0] = Point3f(200,0,0);
    object_wor_pt[1] = Point3f(0,200,0);
    object_wor_pt[2] = Point3f(0,0,200);
    object_wor_pt[3] = Point3f(0,0,0); // center

    projectPoints(object_wor_pt, rvec, tvec, INTRINSIC, distCoeffs, object_img_pt);

    line(img, object_img_pt[3], object_img_pt[0], Scalar(255,0,0), 5);
    line(img, object_img_pt[3], object_img_pt[1], Scalar(0,255,0), 5);
    line(img, object_img_pt[3], object_img_pt[2], Scalar(0,0,255), 5);




    // 코너 그대로 사영시키기
    projectPoints(world_table_outside_corners, rvec, tvec, INTRINSIC, distCoeffs, object_img_pt);
    Mat R;
    Rodrigues(rvec, R);


    for(int i=0; i<4; i++){
        circle(img, Point(img_corners[i].x, img_corners[i].y), 5, Scalar(0, 255, 0), 2, 8, 0);
    }
    for(int i=0; i<4; i++){
        circle(img, Point(object_img_pt[i].x, object_img_pt[i].y), 10, Scalar(0, 0, 255), 2, 8, 0);
    }


    // 안쪽 테두리 표시
    object_wor_pt.resize(4);

    object_wor_pt[0] = Point3f(0,0,30);
    object_wor_pt[1] = Point3f(1224,0,30);
    object_wor_pt[2] = Point3f(1224,2448,30);
    object_wor_pt[3] = Point3f(0,2448,30);



    projectPoints(object_wor_pt, rvec, tvec, INTRINSIC, distCoeffs, object_img_pt);
    for(int i=0; i<4; i++){
        line(img, object_img_pt[i%4], object_img_pt[(i+1)%4], Scalar(0,255,255), 2);
        //circle(img, Point(object_img_pt[i].x, object_img_pt[i].y), 10, Scalar(0, 255, 255), 2, 8, 0);
    }


    object_wor_pt[0] = Point3f(300,300,30);
    object_wor_pt[1] = Point3f(924,300,30);
    object_wor_pt[2] = Point3f(924,2148,30);
    object_wor_pt[3] = Point3f(300,2148,30);

    projectPoints(object_wor_pt, rvec, tvec, INTRINSIC, distCoeffs, object_img_pt);
    for(int i=0; i<4; i++){
        line(img, object_img_pt[i%4], object_img_pt[(i+1)%4], Scalar(255,0, 255), 2);
        //circle(img, Point(object_img_pt[i].x, object_img_pt[i].y), 10, Scalar(0, 255, 255), 2, 8, 0);
    }


}