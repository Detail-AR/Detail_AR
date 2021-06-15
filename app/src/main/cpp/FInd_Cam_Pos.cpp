#include "Geo_Proc.hpp"
#include "add_ons.h"
#include "basic.h"


int Geo_Proc::Find_Cam_Pos(vector<Point2i>& input_corners, vector<Point2i>& balls_center, vector<int>& ball_color_ref){
    
    // 코너의 위치가 정확할 확률이 높으므로 코너를 중심으로 분류해보자.
    Mat in_rvec, in_tvec;
    vector<Point2d> output_corners;

    int ball_num = balls_center.size();
    int corner_num = input_corners.size();
    int can_find_pose = -1;
    
    if(corner_num == 4){  // can_find_pose == 0
        // 공의 갯수와 상관 없이 4개의 코너를 이용해서 계산.
        bool pose_flag = Cam_Pos_with_Four_Corners(input_corners, output_corners, in_rvec, in_tvec, true);
        if(pose_flag){   // 카메라 pose를 구할수 있다면  포즈 정보 업데이트.
            img_corners = output_corners;
            can_find_pose = 0;
        }

        // 당구공의 위치 재계산 가능. 

    }
    else if(corner_num == 2){ // can_find_pose == 1
        if(ball_num  < 2)   return -1;
            // 계산 불가
    }
    else if(corner_num == 1){ // vcan_find_pose == 2
        if(ball_num  < 3)   return -1;
            // 계산 불가
    }
    else{ // ** 코너를 못 찾았더라도 공이 4개 있으면 괜찮다. can_find_pose == 3
        if(ball_num != 4)   return -1;
            // 계산 불가
        
    }


    if(can_find_pose != -1){
        rvec = in_rvec;
        tvec = in_tvec;
        return can_find_pose;
    }

    return -1;
   
    // 코너와 공의 조합이 주어지면 어떻게 시계방향으로 정렬을 할 수 있을까
    // 시계방향 정렬에는 의미가 없을수도 있다.
    
    // 그리고 코너랑 다르게 공들의 위치가 매우 다양하기 때문에 오목한 모양이 나올수 있는데,
    // 이것으로도 추정이 가능한지...??
    
    
    
    // 코너랑 공이랑 각각 따로 정렬 한다...??
    
    // 코너가 2개이면 뒤집어 가면서 해야한다....  매칭은 12 23 34 41 하면 되니께..!
    // 코너쪽에서는 2개의 후보를 가지고 8번의 매칭을 해야 한다.
    // reprojection 에러나 이전프레임과의 카메라 포즈 비교로 걸러내자!!
    
    // 공은 그대로 매칭하면 되긴하는데.. 빨간공이 문제이다.
    // 빨간공이 2개라면 이 역시 뒤집어 가면서 해주어야 한다. 따로 만들어 주어야 하나?
    
    
    
    
    // 코너가 1개면 정렬 필요없음..!! 매칭은 1 2 3 4 로 하면 되니께!!
    // 역시 빨간공이 있는 경우가 문제가 될듯!!

    // 공으로 카메라 pose를 추정할 때 공이 일직선에 있으면 안되는데 ...
    

}


bool Geo_Proc::Cam_Pos_with_Four_Corners(vector<Point2i>& input_corners, vector<Point2d>& output_corners,
Mat& in_rvec, Mat& in_tvec, bool comp_prev){
    
    vector<Point2d> temp_corners(4);
    vector<Point2d> reproject_point(4);

    Sort_Corners_Clockwise(input_corners);
    for(int i=0; i<4 ;i++)
        temp_corners[i] = static_cast<Point2d>(input_corners[3-i]);   // ** 반시계 방향으로 넣어야 z축이 천장을 향한다


   // 코너가 시계방향으로 돌면서 reprojection 오차가 이 가장 작은 값을 취한다.
   vector<pair<double, int>> dist_with_index(4);
   vector<Point2d> temp[4];
   Mat temp_rvec[4], temp_tvec[4];

    for(int i=0; i<4 ; i++){
    Clockwise_Permutation(temp_corners);
    solvePnP(world_table_outside_corners, temp_corners, INTRINSIC, distCoeffs, temp_rvec[i], temp_tvec[i]);
    projectPoints(world_table_outside_corners, temp_rvec[i], temp_tvec[i], INTRINSIC, distCoeffs, reproject_point);

    double dist = double_vector_dist_sum(temp_corners, reproject_point);

    dist_with_index[i].first = dist;
    dist_with_index[i].second = i;

    temp[i] = temp_corners;
   }
    
    sort(dist_with_index.begin(), dist_with_index.end());

    if(dist_with_index[0].first>200)   // 4개의 코너가 잘못되었을 가능성이 높다.
        return false;

    int id1 = dist_with_index[0].second;
    int id2 = dist_with_index[1].second;

    /*
    for(int i=0; i<4 ;i++){
       
       cout<<"<"<<dist_with_index[i].first<<","<<dist_with_index[i].second<<">"<<endl;
    }*/


    if(comp_prev){   // 이전프레임 정보를 이용하여 id1과 id2 의 포즈중 하나를 선택한다.
        // 이전프레임과 rvec 와 tvec 을 비교하여 적절한 코너와 rvec, tvec 을 반환한다.

        Mat in_rvec1 = temp_rvec[id1];
        Mat in_tvec1 = temp_tvec[id1];

        Mat in_rvec2 = temp_rvec[id2];
        Mat in_tvec2 = temp_tvec[id2];
        
        double dist_r1 = Dist_of_Rotation(rvec, in_rvec1);
        double dist_r2 = Dist_of_Rotation(rvec, in_rvec2);
        double dist_t1 = Dist_of_Translation(tvec, in_tvec1);
        double dist_t2 = Dist_of_Translation(tvec, in_tvec2);

        //cout<<dist_r1<<","<<dist_t1<<"  "<<dist_r2<<","<<dist_t2<<endl;

        if(dist_r1+ dist_t1 > dist_r2 + dist_t2){
            in_tvec = in_tvec2;
            in_rvec = in_rvec2;
            output_corners = temp[id2];
        }
        else{
            in_tvec = in_tvec1;
            in_rvec = in_rvec1;
            output_corners = temp[id1];
        }


        //if() 이전프레임과 rvec과 tvec의 차이가 너무 크다면 구할 수 없다.
    }
    else  // index1 과 index2의 포즈 둘중 아무거나 반환.
    {
        output_corners = temp[id1];
        in_rvec = temp_rvec[id1];
        in_tvec = temp_tvec[id1];
    }
    

    // for drawing
    vector<Point3f> object_wor_pt(4);

    object_wor_pt[0] = Point3f(0,0,0);
    object_wor_pt[1] = Point3f(B_W,0,0);
    object_wor_pt[2] = Point3f(B_W,B_H,0);
    object_wor_pt[3] = Point3f(0,B_H,0);

    projectPoints(object_wor_pt, in_rvec, in_tvec, INTRINSIC, distCoeffs, H_img_pts);
    
    H_wor_pts.resize(4);
    H_wor_pts[0] = Point2f(0,0);
    H_wor_pts[1] = Point2f(B_W,0);
    H_wor_pts[2] = Point2f(B_W,B_H);
    H_wor_pts[3] = Point2f(0,B_H);

    return true;
}