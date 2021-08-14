#include "Geo_Proc.hpp"
#include "add_ons.h"
#include "basic.h"


Geo_Proc::Geo_Proc(int f_len) : ball_rad(33), table_depth(-40), edge_thickness(52), B_W(1224), B_H(2448), e_num(0), cum(0)
{   // 입력 영상의 픽셀을 고려해야함.
    double * intrinsic_para;

    double f = f_len;
    double cx = 240;
    double cy = 360;


    if(device_dir) // 가로방향 // {fx, 0, cx, 0, fy, cy, 0, 0, 1};
        intrinsic_para = new double[9]{f, 0, cx, 0, f, cy, 0, 0, 1};
    else
        intrinsic_para = new double[9]{f, 0, cy, 0, f, cx, 0, 0, 1};

    INTRINSIC = Mat(3,3, CV_64FC1, intrinsic_para);
    distCoeffs = Mat();

    world_table_outside_corners.resize(4);

    // 중심을 기준으로 왼쪽 위부터 저장
    
    world_table_outside_corners[1] = Point3d(B_W+edge_thickness, -edge_thickness, table_depth);
    world_table_outside_corners[2] = Point3d(B_W+edge_thickness, B_H+edge_thickness, table_depth);
    world_table_outside_corners[3] = Point3d(-edge_thickness, B_H+edge_thickness, table_depth);
    world_table_outside_corners[0] = Point3d(-edge_thickness, -edge_thickness, table_depth);

    world_table_inside_corners_d.resize(4);
    world_table_inside_corners_f.resize(4);

    world_table_inside_corners_d[0] = Point3d(0,0,0);
    world_table_inside_corners_d[1] = Point3d(B_W,0,0);
    world_table_inside_corners_d[2] = Point3d(B_W,B_H,0);
    world_table_inside_corners_d[3] = Point3d(0,B_H,0);

    world_table_inside_corners_f[0] = Point3f(0,0,0);
    world_table_inside_corners_f[1] = Point3f(B_W,0,0);
    world_table_inside_corners_f[2] = Point3f(B_W,B_H,0);
    world_table_inside_corners_f[3] = Point3f(0,B_H,0);

}

void Geo_Proc::Set_Device_Dir(bool dir, Mat& img){
    device_dir = dir;
    temp_img = img;
}


int Geo_Proc::Find_Balls_3D_Loc(vector<Point2i>& input_corners, vector<Point2i>& balls_center, vector<int>& ball_color_ref,
vector<Point2i>& wor_ball_cen, bool update)
{
    if(input_corners.size() != 4)   // 코너가 4개일 경우에만 처리.
        return -1;
    
    Mat in_rvec, in_tvec;
    
    bool pose_flag = Cam_Pos_with_Four_Corners(input_corners,  in_rvec, in_tvec, false);

    if(pose_flag){   // 카메라 pose를 구할수 있다면  포즈 정보 업데이트.

        // for drawing.
        projectPoints(world_table_inside_corners_f, in_rvec, in_tvec, INTRINSIC, distCoeffs, H_img_pts);

        H_wor_pts.resize(4);
        H_wor_pts[0] = Point2f(0,0);
        H_wor_pts[1] = Point2f(B_W,0);
        H_wor_pts[2] = Point2f(B_W,B_H);
        H_wor_pts[3] = Point2f(0,B_H);
        
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


    int hei = -ball_rad;
    vector<Point2i> wor_ball(b_n);
    
    for(int i=0; i<b_n ; i++){
        double s =  (hei + B.ptr<double>(2)[i]) /( A.ptr<double>(2)[i] + 1e-8);

        int x = (int)( s * A.ptr<double>(0)[i] - B.ptr<double>(0)[i]);
        int y = (int)( s * A.ptr<double>(1)[i] - B.ptr<double>(1)[i]);
        wor_ball[i] = Point2i(x,y);   // 공의 위치가 이상하면 기각하는 코드 추가
    }


    // put the output
    wor_ball_cen = wor_ball; 

    bool first_red = false;
    int ball_r = -ball_rad;
    /// *** world ball 좌표 R R Y W 순으로 업데이트 ***
    if(update){     
        wor_ball_loc.resize(4);
        world_ball_color_ref.assign(4, -1);

        for(int i=0; i<b_n ; i++){
            if(ball_color_ref[i] == 0 && !first_red){
                wor_ball_loc[0] = Point3d(wor_ball[i].x, wor_ball[i].y, ball_r);
                world_ball_color_ref[0] = 0;
                first_red = true;
            }
            else if(ball_color_ref[i] == 0 && first_red){
                wor_ball_loc[1] = Point3d(wor_ball[i].x, wor_ball[i].y, ball_r);
                world_ball_color_ref[1] = 0;
            }
            else if(ball_color_ref[i] == 1){
                wor_ball_loc[2] = Point3d(wor_ball[i].x, wor_ball[i].y, ball_r);
                world_ball_color_ref[2] = 1;
            }
            else if(ball_color_ref[i] == 2){
                wor_ball_loc[3] = Point3d(wor_ball[i].x, wor_ball[i].y, ball_r);
                world_ball_color_ref[3] = 2;
            }
        }
    }
    
    wor_ball_n = b_n;

    return 0;
}


void Geo_Proc::Draw_Obj_on_Template(){
    Ball_templete = Mat(B_H,B_W, CV_8UC3, Scalar(0,0,0));   // for drawing
                    
    for(int i=0; i<4 ; i++){
        Scalar color;

        if(world_ball_color_ref[i] == 0)
            color = Scalar(255,20,20);
        else if(world_ball_color_ref[i] == 1 )
            color = Scalar(255, 255, 20);
        else if(world_ball_color_ref[i] == 2 )
            color = Scalar(255,255,255);
        else
            continue;
                        
        circle(Ball_templete, Point(wor_ball_loc[i].x, wor_ball_loc[i].y), ball_rad, color, 8, 8, 0);
    }
                   
}


void Geo_Proc::Reprojection_Error(vector<Point3d>& match1, vector<Point2d>& match2,  Mat& rvec_, Mat& tvec_, 
double& distance){

    bool PnP_flag;
    vector<Point2d> reproject_point;
    try{
    PnP_flag = solvePnP(match1, match2, INTRINSIC, distCoeffs, rvec_, tvec_);
    }
    catch(const cv::Exception& e){
        PnP_flag = false;
    }


    if(PnP_flag){
        projectPoints(match1, rvec_, tvec_, INTRINSIC, distCoeffs, reproject_point);
        distance = double_vector_dist_sum(match2, reproject_point);
    }
    else{
        distance = 10000.0;
    }
}

void Geo_Proc::Distance_from_Prev_Frame(Mat& rvec_, Mat& tvec_, double& distance){
    double dist1 = Dist_of_Rotation(rvec_, rvec);
    double dist2 = Dist_of_Translation(tvec_, tvec);
    distance = dist1 + dist2;
}


int Geo_Proc::Distance_from_Error_Frame(Mat& rvec_, Mat& tvec_){
    double dist1 = Dist_of_Rotation(rvec_, e_rvec);
    double dist2 = Dist_of_Translation(tvec_, e_tvec);
    int dist = (int)(dist1 + dist2);
    if(dist>= 10 && dist<=80)
        return (int)(dist1 + dist2);
    else if(dist < 10)
        return 20;
    else
        return 80;
}

bool Geo_Proc::Error_Comparison_with_Prev_Frame(Mat& in_rvec, Mat& in_tvec, double& distance){
    int error_thres = 60;   // 파라미터
    int average_dist = 35;
    //return true;
    if( error_thres + cum <= (int)(distance)){
        if(e_num < 3){
            if(e_num == 0)
                cum += average_dist;
            else
                cum += Distance_from_Error_Frame(in_rvec, in_tvec);
            
            e_rvec = in_rvec;
            e_tvec = in_tvec;
            in_rvec = rvec;    // 오류가 일어난 포즈 대신 이전레임 포즈 저장.
            in_tvec = tvec;
            e_num++;
            return true;
        }
        else{
            cum += Distance_from_Error_Frame(in_rvec, in_tvec);
            e_rvec = in_rvec;
            e_tvec = in_tvec;
            e_num++;
            return false;
        }

    }
    else if( 10 > (int)(distance)){
        e_num = 0;
        cum = 0;
        in_rvec = rvec;    
        in_tvec = tvec;
        return true;
    }
    else{
        e_num = 0;
        cum = 0;
        return true;
    }

    

}

Mat Geo_Proc::GetTemplate(void){
    return Ball_templete.clone();
}

void Geo_Proc::SaveTemplate(Mat& templete){
    Ball_and_Sol_templete = templete;
}

void Geo_Proc::Draw_3D_Template_on_Img(Mat& img){

    // *****당구공아래에 원 표시*****
    

    Mat output;
    Mat H = getPerspectiveTransform(H_wor_pts, H_img_pts);
 

    warpPerspective(Ball_and_Sol_templete, output, H, img.size(), INTER_NEAREST);
    

    Mat mask;
    cvtColor(output, mask, COLOR_BGR2GRAY);
    threshold(mask, mask, 0, 255, THRESH_BINARY_INV);
    Mat fg;
    bitwise_and(img, img, fg, mask);
    img = fg+output;
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
        circle(img, Point(object_img_pt[i].x, object_img_pt[i].y), 10, Scalar(0, i*80, i*80), 2, 8, 0);
    }
    // 안쪽 테두리 표시
    projectPoints(world_table_inside_corners_d, rvec, tvec, INTRINSIC, distCoeffs, object_img_pt);

    for(int i=0; i<4; i++){
        line(img, object_img_pt[i%4], object_img_pt[(i+1)%4], Scalar(0,255,255), 2);
    }

}
