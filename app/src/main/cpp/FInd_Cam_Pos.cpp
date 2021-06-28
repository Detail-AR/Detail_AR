#include "Geo_Proc.hpp"
#include "add_ons.h"
#include "basic.h"


int Geo_Proc::Find_Cam_Pos(vector<Point2i>& input_corners, vector<Point2i>& balls_center, vector<int>& ball_color_ref){

    // 코너의 위치가 정확할 확률이 높으므로 코너를 중심으로 분류해보자.
    Mat in_rvec, in_tvec;

    int ball_num = balls_center.size();
    int corner_num = input_corners.size();
    bool pose_flag;

    vector<pair<int, int>> dist_with_index(6);
    vector<Point2i> com_balls_c;
    vector<int> com_balls_r;
    Mat temp_rvec[6], temp_tvec[6];
    int dist, best_i;

    int com_table1[12]={0,1,0,2,1,2,1,3,2,3,0,3};
    int com_table2[12]={0,1,2,0,1,3,1,2,3,0,2,3}; 
    
    if(corner_num == 4){  // can_find_pose == 0
        // 공의 갯수와 상관 없이 4개의 코너를 이용해서 계산.
        pose_flag = Cam_Pos_with_Four_Corners(input_corners, in_rvec, in_tvec, true);
    }
    else if(corner_num == 2){ // can_find_pose == 1
        if(ball_num  < 2)   return -1;
            // 계산 불가

        if(ball_num == 2){
            dist = Cam_Pos_with_Two_Corners_Two_balls(input_corners, balls_center, ball_color_ref, in_rvec, in_tvec);
        }
        else{

            
            com_balls_c.resize(2);
            com_balls_r.resize(2);
            for(int i=0; i<3*(ball_num-2) ; i++){
                int i1 = com_table1[i*2];
                int i2 = com_table1[i*2+1];

                com_balls_c[0]=balls_center[i1];
                com_balls_r[0]=ball_color_ref[i1];
                com_balls_c[1]=balls_center[i2];
                com_balls_r[1]=ball_color_ref[i2];

                int temp_dist = Cam_Pos_with_Two_Corners_Two_balls(input_corners,  com_balls_c, com_balls_r, temp_rvec[i],  temp_tvec[i]);

                if(temp_dist == -1)
                    temp_dist = numeric_limits<int>::max();
                dist_with_index[i] = make_pair(temp_dist, i);

    
                
            }
            
            if(ball_num==3)
                sort(dist_with_index.begin(), dist_with_index.begin()+3);
            else
                sort(dist_with_index.begin(), dist_with_index.begin()+6);
          

            dist = dist_with_index[0].first;
            best_i = dist_with_index[0].second;
            in_rvec = temp_rvec[best_i];
            in_tvec = temp_tvec[best_i];
                       
        }

        if(dist == -1 || dist == numeric_limits<int>::max()){
            pose_flag = false;
        }
        else
            pose_flag = true;

    }
    else if(corner_num == 1){ // vcan_find_pose == 2
        if(ball_num  < 3)   return -1;

  
        if(ball_num == 3){
            dist = Cam_Pos_with_One_Corners_Three_balls(input_corners, balls_center, ball_color_ref, in_rvec, in_tvec);
        }
        else{
            com_balls_c.resize(3);
            com_balls_r.resize(3);
            for(int i=0; i<4 ; i++){
                int i1 = com_table2[i*3];
                int i2 = com_table2[i*3+1];
                int i3 = com_table2[i*3+2];

                com_balls_c[0]=balls_center[i1];
                com_balls_r[0]=ball_color_ref[i1];
                com_balls_c[1]=balls_center[i2];
                com_balls_r[1]=ball_color_ref[i2];
                com_balls_c[2]=balls_center[i3];
                com_balls_r[2]=ball_color_ref[i3];

                
                int temp_dist = Cam_Pos_with_One_Corners_Three_balls(input_corners, com_balls_c, com_balls_r, temp_rvec[i],  temp_tvec[i]);
                if(temp_dist == -1)
                    temp_dist = numeric_limits<int>::max();
                dist_with_index[i] = make_pair(temp_dist, i);

            }

            sort(dist_with_index.begin(), dist_with_index.begin()+4);
            dist = dist_with_index[0].first;
            best_i = dist_with_index[0].second;
            in_rvec = temp_rvec[best_i];
            in_tvec = temp_tvec[best_i];
            
        }


        if(dist == -1 || dist == numeric_limits<int>::max()){
            pose_flag = false;
        }
        else
            pose_flag = true;
    }
    else{ // ** 코너를 못 찾았더라도 공이 4개 있으면 괜찮다. can_find_pose == 3
        if(ball_num != 4)   return -1;
            // 계산 불가
        pose_flag = Cam_Pos_with_Four_balls(balls_center, ball_color_ref, in_rvec, in_tvec);

    }

    // 최종 결과 반영
    if(pose_flag){
                 
        double temp;
        Distance_from_Prev_Frame(in_rvec, in_tvec, temp);


         //cout<<"Reprojection Error: "<<best_dist<<" Dist_Prev_Frame: "<<temp<<endl<<endl;

        bool e_flag = Error_Comparison_with_Prev_Frame(in_rvec, in_tvec, temp);
        if(!e_flag)
            return -1;

        // for drawing.
        projectPoints(world_table_inside_corners_f, in_rvec, in_tvec, INTRINSIC, distCoeffs, H_img_pts);

        H_wor_pts.resize(4);
        H_wor_pts[0] = Point2f(0,0);
        H_wor_pts[1] = Point2f(B_W,0);
        H_wor_pts[2] = Point2f(B_W,B_H);
        H_wor_pts[3] = Point2f(0,B_H);

        rvec = in_rvec;
        tvec = in_tvec;
        return 0;
    }
    else
        return -1;
    
    // 공으로 카메라 pose를 추정할 때 공과 코너가 일직선에  있으면 안될것 같다.
}


bool Geo_Proc::Cam_Pos_with_Four_Corners(vector<Point2i>& input_corners, Mat& in_rvec, Mat& in_tvec, bool comp_prev){
    
    vector<Point2d> temp_corners(4);
    bool e_flag;
 
    Sort_Corners_Clockwise(input_corners);
    for(int i=0; i<4 ;i++)
        temp_corners[i] = static_cast<Point2d>(input_corners[i]); // 시계방향으로 해야 왼쪽 상단이 원점이 된다.

   // 코너가 시계방향으로 돌면서 reprojection 오차가 이 가장 작은 값을 취한다.
   vector<pair<double, int>> dist_with_index(4);
   vector<Point2d> temp[4];
   vector<Mat> temp_rvec(4), temp_tvec(4);

    for(int i=0; i<4 ; i++){
    Clockwise_Permutation(temp_corners);
    double dist;
    Reprojection_Error(world_table_outside_corners, temp_corners, temp_rvec[i], temp_tvec[i], dist);


    dist_with_index[i].first = dist;
    dist_with_index[i].second = i;

    temp[i] = temp_corners;
   }
    
    sort(dist_with_index.begin(), dist_with_index.end());

    if(dist_with_index[0].first>200)   // 4개의 코너가 잘못되었을 가능성이 높다.
        return false;

    int id1 = dist_with_index[0].second;
    int id2 = dist_with_index[1].second;


    if(comp_prev){   // 이전프레임 정보를 이용하여 id1과 id2 의 포즈중 하나를 선택한다.
        // 이전프레임과 rvec 와 tvec 을 비교하여 적절한 코너와 rvec, tvec 을 반환한다.

        Mat in_rvec1 = temp_rvec[id1];
        Mat in_tvec1 = temp_tvec[id1];
        Mat in_rvec2 = temp_rvec[id2];
        Mat in_tvec2 = temp_tvec[id2];
        
        double dist1, dist2;

        Distance_from_Prev_Frame(in_rvec1, in_tvec1, dist1);
        Distance_from_Prev_Frame(in_rvec2, in_tvec2, dist2);

        if(dist1 > dist2){
            in_tvec = in_tvec2;
            in_rvec = in_rvec2;
        }
        else{
            in_tvec = in_tvec1;
            in_rvec = in_rvec1;
        }

    }
    else  // index1 과 index2의 포즈 둘중 아무거나 반환.
    {
        in_rvec = temp_rvec[id1];
        in_tvec = temp_tvec[id1];
    }
    
    return true;
}


int Geo_Proc::Cam_Pos_with_Two_Corners_Two_balls(vector<Point2i>& input_corners, vector<Point2i>& balls_center, 
vector<int>& ball_color_ref, Mat& in_rvec, Mat& in_tvec){
    if(input_corners.size() != 2 || balls_center.size() !=2)
        return -1;
    

    Mat temp_rvec[16], temp_tvec[16];
    vector<pair<double, int>> dist(16);
    vector<Point3d> match1(4);
    vector<Point2d> match2(4);
    


    vector<Point3d> world_ball;  // W R / Y R / R R / W Y  ... possible combination.

    int red_n = 0;
    bool first_red = false;


    for(int i=0; i<2 ;i++){
        int color = ball_color_ref[i];
        if(color == 0){
            if(!first_red){
                world_ball.push_back(wor_ball_loc[0]);
                red_n++;
                first_red = true;
            }
            else{
                red_n++;
                world_ball.push_back(wor_ball_loc[1]);
            }
        }
        else if(color == 1){
            world_ball.push_back(wor_ball_loc[2]);
        }
        else
            world_ball.push_back(wor_ball_loc[3]);
    }
    
    
    

    int id= 0;
    double distance;

    match2[0] = input_corners[0];
    match2[1] = input_corners[1];
    match2[2] = static_cast<Point2d>(balls_center[0]);
    match2[3] = static_cast<Point2d>(balls_center[1]); 
    
    match1[2] = world_ball[0];   
    match1[3] = world_ball[1];  

    vector<vector<Point3d>> match_list1(16);
    vector<vector<Point2d>> match_list2(16);

    
    
    for(int j=0; j<4 ; j++){
        // world table 코너정보는 많이 이용할거같으니 맴버변수로 하나 만들자! 그리고 4개코너도 수정 ㄱㄱ
        match1[0] = world_table_outside_corners[j];
        match1[1] = world_table_outside_corners[(j+1)%4];


        for(int c=0; c<2 ;c++){

            if(red_n==2){  
                for(int r=0; r<2 ; r++){
                    // 매칭

                    // 매칭 거리구하는 함수...!! 
                    // vector<Point2d> match1 match2 전달해서 거리를 반환받자
                    
                    Reprojection_Error(match1, match2, temp_rvec[id], temp_tvec[id], distance);

                    dist[id]= make_pair(distance, id);

                    // for debug
                    match_list1[id] = match1;
                    match_list2[id] = match2;


                    swap(match1[2], match1[3]);  // red red 끼리 교환.
                    id++;   
                }
            }
            else if(red_n==1){
                // 매칭
                for(int r=0; r<2 ; r++){       
                    Reprojection_Error(match1, match2, temp_rvec[id], temp_tvec[id], distance);
                                
                    dist[id]= make_pair(distance, id);

                    // for debug
                    match_list1[id] = match1;
                    match_list2[id] = match2;

                    if(ball_color_ref[0] == 0)   // 첫 번쨰 공이 red라면
                        swap(match1[2],wor_ball_loc[1]);  // world 좌표에있는 다른 red 공과 바꾼다.
                    else 
                        swap(match1[3],wor_ball_loc[1]);
                    id++;
                }
            }
            else{  // Red 공이 없는 경우
                
                Reprojection_Error(match1, match2, temp_rvec[id], temp_tvec[id], distance);
                dist[id]= make_pair(distance, id);
                
                // for debug
                match_list1[id] = match1;
                match_list2[id] = match2;
                id++;

            }
            swap(match1[0], match1[1]);

        }
    }

    
    if(red_n>0){
        sort(dist.begin(), dist.end());
    }
    else{
        sort(dist.begin(), dist.begin()+8);
    }


    double best_dist = dist[0].first;
    int best_i = dist[0].second;

    
    if(best_dist > 100)
        return -1;

    in_rvec = temp_rvec[best_i];
    in_tvec = temp_tvec[best_i];

    return best_dist;
}


int Geo_Proc::Cam_Pos_with_One_Corners_Three_balls(vector<Point2i>& input_corners, vector<Point2i>& balls_center, 
vector<int>& ball_color_ref, Mat& in_rvec, Mat& in_tvec){
    if(input_corners.size() != 1 || balls_center.size() !=3)
        return -1;

    Mat temp_rvec[8], temp_tvec[8];
    vector<pair<double, int>> dist(8);
    vector<Point3d> match1(4);
    vector<Point2d> match2(4);

    vector<Point3d> world_ball;  // 코너 +  R R W  와  R W Y  2가지 조합. 각각 무조건 8개의 조합이 필요하다.   

    int red_n = 0;
    bool first_red = false;
    int red1_i, red2_i;


    for(int i=0; i<3 ;i++){
        int color = ball_color_ref[i];

        if(color == 0){
            if(!first_red){
                world_ball.push_back(wor_ball_loc[0]);
                red_n++;
                first_red = true;
                red1_i=i;

            }
            else{
                red_n++;
                red2_i=i;
                world_ball.push_back(wor_ball_loc[1]);
            }
        }
        else if(color == 1){
            world_ball.push_back(wor_ball_loc[2]);
        }
        else
            world_ball.push_back(wor_ball_loc[3]);
    }


    int id= 0;
    double distance;

    match2[0] = input_corners[0];
    match2[1] = static_cast<Point2d>(balls_center[0]);
    match2[2] = static_cast<Point2d>(balls_center[1]);
    match2[3] = static_cast<Point2d>(balls_center[2]); 
    
    match1[1] = world_ball[0];   
    match1[2] = world_ball[1];
    match1[3] = world_ball[2];    

    vector<vector<Point3d>> match_list1(8);
    vector<vector<Point2d>> match_list2(8);

       
    for(int j=0; j<4 ; j++){
        // world table 코너정보는 많이 이용할거같으니 맴버변수로 하나 만들자! 그리고 4개코너도 수정 ㄱㄱ
        match1[0] = world_table_outside_corners[j];

        if(red_n==2){  
            for(int r=0; r<2 ; r++){
                    
                Reprojection_Error(match1, match2, temp_rvec[id], temp_tvec[id], distance);
                dist[id]= make_pair(distance, id);

                // for debug
                match_list1[id] = match1;
                match_list2[id] = match2;

                swap(match1[red1_i+1], match1[red2_i+1]);  // red red 끼리 교환.
                id++;   
            }
        }
        else if(red_n==1){
            for(int r=0; r<2 ; r++){
                Reprojection_Error(match1, match2, temp_rvec[id], temp_tvec[id], distance);
                dist[id]= make_pair(distance, id);

                // for debug
                match_list1[id] = match1;
                match_list2[id] = match2;

                swap(match1[red1_i+1],wor_ball_loc[1]);
                id++;
            }
        }
        else
            return -1;

    }


    sort(dist.begin(), dist.end());

    double best_dist = dist[0].first;
    int best_i = dist[0].second;

    
    if(best_dist > 100)
        return -1;

    in_rvec = temp_rvec[best_i];
    in_tvec = temp_tvec[best_i];

    return best_dist;
}

bool Geo_Proc::Cam_Pos_with_Four_balls(vector<Point2i>& balls_center, 
vector<int>& ball_color_ref, Mat& in_rvec, Mat& in_tvec){
    if(balls_center.size() !=4)
        return false;

    Mat temp_rvec[2], temp_tvec[2];
    vector<pair<double, int>> dist(2);
    vector<Point3d> match1(4);
    vector<Point2d> match2(4);

    vector<Point3d> world_ball;  // R R W Y 만 가능하다.


    bool first_red = false;
    int red1_i, red2_i;


    for(int i=0; i<4 ;i++){
        int color = ball_color_ref[i];

        if(color == 0){
            if(!first_red){
                world_ball.push_back(wor_ball_loc[0]);
                first_red = true;
                red1_i=i;

            }
            else{
                red2_i=i;
                world_ball.push_back(wor_ball_loc[1]);
            }
        }
        else if(color == 1){
            world_ball.push_back(wor_ball_loc[2]);
        }
        else
            world_ball.push_back(wor_ball_loc[3]);
    }


    double distance;
    bool RE_flag;
    for(int i=0; i<4; i++){
        match1[i] = world_ball[i];
        match2[i] = static_cast<Point2d>(balls_center[i]);
    }
    
    vector<vector<Point3d>> match_list1(2);
    vector<vector<Point2d>> match_list2(2);


    for(int r=0; r<2 ; r++){               
        Reprojection_Error(match1, match2, temp_rvec[r], temp_tvec[r], distance);
        dist[r]= make_pair(distance, r);

        // for debug
        match_list1[r] = match1;
        match_list2[r] = match2;

        swap(match1[red1_i], match1[red2_i]);  // red red 끼리 교환.
    }

    
    sort(dist.begin(), dist.end());

    double best_dist = dist[0].first;
    int best_i = dist[0].second;

    if(best_dist > 100)
        return false;

    in_rvec = temp_rvec[best_i];
    in_tvec = temp_tvec[best_i];

    return true;
}



