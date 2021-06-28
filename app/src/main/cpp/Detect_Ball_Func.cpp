#include "basic.h"
#include "add_ons.h"

void Detect_ball_color(Mat& img_hsv, Mat* output_array){


	Mat Red_m_1, Red_m_2, Red_mask;
	Mat Yellow_mask;
    Mat White_mask;


	Scalar lower_red1 = Scalar(160, 130, 50);   // 오히려 분홍쪽에 색이 가까웠음
	Scalar upper_red1 = Scalar(179, 255, 255);

    Scalar lower_red2 = Scalar(0, 130, 50);
	Scalar upper_red2 = Scalar(3, 255, 255);

    Scalar lower_yellow = Scalar(3, 130, 50);   // 그림자 부분은 거의 붉은색이다
	Scalar upper_yellow = Scalar(35, 255, 255);


    Scalar lower_white = Scalar(0, 0, 100);     // 흰색은 채도가 없는 것? but.. 그렇게하면 안됨..
	Scalar upper_white = Scalar(179, 50, 255);


	inRange(img_hsv, lower_red1, upper_red1, Red_m_1);
    inRange(img_hsv, lower_red2, upper_red2, Red_m_2);

    bitwise_or(Red_m_1, Red_m_2, Red_mask);

	inRange(img_hsv, lower_yellow, upper_yellow, Yellow_mask);
    
	inRange(img_hsv, lower_white, upper_white, White_mask);


    Mat balls, All_ball;
    bitwise_or(Yellow_mask, Red_mask, balls);
    bitwise_or(balls, White_mask, All_ball);
    

    output_array[0] = Red_mask;
    output_array[1] = Yellow_mask;
    output_array[2] = White_mask; 
}




void Match_ball_and_color(Mat* ball_colors, Mat& hole, vector<Mat>& ball_candidate, vector<vector<Vec3i>>& label_with_color){
    label_with_color.resize(4);  // 공은 반드시 4개 이하이다.
    ball_candidate.resize(4, Mat());


    vector<pair<int,int>> find_best_color_area;
    int total;
    Mat eroded;

    //erode(hole, hole, Mat::ones(Size(3,3),CV_8UC1),Point(-1,-1),1); // 노이즈 제거용 *** 속도 개선이 필요하면 고려 ***

    Mat img_label, stats, centroids;
    int numOfLables = connectedComponentsWithStats(hole, img_label, stats, centroids, 4, CV_32S);

    if(numOfLables == 0 || numOfLables == 1){ return; }//cout<<"No balls detected!";  
    


    vector<pair<int,int>> FBFL; // Find_Best_Four_Label // 공은 반드시 4개이다.
    for (int j = 1; j < numOfLables; j++) { 
        int* label = stats.ptr<int>(j);
        int wid = label[2];
        int hei = label[3];
        int area = label[4];
        float ratio1, ratio2 = (float)area/(wid*hei);
        if(wid>hei)
            ratio1 = (float)hei/wid;
        else
            ratio1 = (float)wid/hei;
        

        if( ratio1< 0.4)           //******** 파라미터 : 공의 모양을 보고 판단해보자. *************
            continue;              //**** 필요하면 크기도 추가*****
        if(ratio2 < 0.3)
            continue;
        
        // ********* 특정 픽셀 비율 이하면 공이라고 판단 안하고 건너 띈다!! **************
        if(area < 60)
            continue;

        FBFL.push_back(make_pair(area, j));
    }



    int FBFL_size = FBFL.size();
    if((int)(FBFL_size) >= 4){
        //nth_element(FBFL.begin(), FBFL.begin()+ 4, FBFL.end(), cmp2); 
        // nth는 4번째까지 빠르게 고르지만 4번째까지는 대신 정렬이 안되어있음.. 굳이 필요 없으면 위에걸로 바꾸자
        sort(FBFL.begin(), FBFL.end(), cmp2);
        total = 4;
    }
    else{
        sort(FBFL.begin(), FBFL.end(), cmp2);
        total = FBFL_size;
    }

    
    // ball 후보들을 따로 분리해준다.
    // i 번쨰 라벨(hole) 은 몇 번째 j 컬러 일까



    for(int i=0; i<total ; i++){
        int label = FBFL[i].second;
        Mat ball_candi = label == img_label;
        int ball_candi_area = sum(ball_candi)[0]/255;
        bool is_the_ball=false;

        Mat what_color[3];  // 공 후보와 색깔(3종류)이 겹친 영상
        for(int j=0; j<3 ; j++){
            bitwise_and(ball_candi, ball_colors[j], what_color[j] );

            int area = sum(255 == what_color[j])[0]/255;
            find_best_color_area.push_back(make_pair(area, j));

        }

        sort(find_best_color_area.begin(), find_best_color_area.end(), cmp2);

        int best_color_index = find_best_color_area[0].second;
        int best_color_area = find_best_color_area[0].first;
        float second_color_ratio; // 두 번째로 우세한 색의 첫 번쨰로 우세한 색의 비율
        float third_color_ratio;
        int color_index;
        Point2i one_blob_center;


        float color_ratio = (float)best_color_area/ball_candi_area;

        if(color_ratio > 0.01){ // 파라미터 ******
            second_color_ratio = (float)(find_best_color_area[1].first) / best_color_area;
            third_color_ratio = (float)(find_best_color_area[2].first) / best_color_area;


            bool flag = false; // 겹쳤는가?

            if(second_color_ratio > 0.5){  // 다음으로 우세한 컬러의 인덱스와 중심을 계산한다
                color_index = find_best_color_area[1].second;

                Find_one_blob_center( what_color[color_index], one_blob_center);
                label_with_color[i].push_back(Vec3i(color_index, one_blob_center.x, one_blob_center.y));
                flag = true;
            }
            if(third_color_ratio > 0.5 ){
                color_index = find_best_color_area[2].second;

                Find_one_blob_center( what_color[color_index], one_blob_center);
                label_with_color[i].push_back(Vec3i(color_index, one_blob_center.x, one_blob_center.y));
                flag=true;
            }  

            if(flag){
                Find_one_blob_center( what_color[best_color_index], one_blob_center);
                label_with_color[i].push_back(Vec3i(best_color_index, one_blob_center.x, one_blob_center.y));
            }
            else{   // 겹치지 않은 공이면 공이 하나이므로 색의 상대적 위치를 알 필요 없음.
                label_with_color[i].push_back(Vec3i(best_color_index, -1, -1));
            }

            is_the_ball = true;
        }
        else{
            //cout<<"This label is not a ball!";  // 겹치는 색이 없는 라벨이면.. 그냥 공이 아닌거다.
            //ball_candidate[i] = Mat();  // 빈 영상을 준다.
        }

        if(is_the_ball)
            ball_candidate[i] = ball_candi;

        find_best_color_area.clear();
    }

}




void Find_ball_center(vector<Mat>& ball_candidate, vector<vector<Vec3i>>& label_with_color, Mat* ball_colors, 
vector<Point2i>& balls_center, vector<int>& ball_color_ref){


    // 4개의 공의 후보
    for(int i=0; i<4 ; i++){
        Mat temp = ball_candidate[i];
        if(temp.dims != 2)   // 겹치는 색이 없으면 공이 아니었음.
            continue;

        int overlapped = label_with_color[i].size();

        if(overlapped < 2){   // 공이 하나인 경우 ( 대부분의 경우 )

            vector<vector<Point>> contours;
            findContours( temp, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );

            vector<Point> contours_poly;
            Point2f centers;
            float radius;
 

            approxPolyDP( contours[0], contours_poly, 3, true );
            minEnclosingCircle( contours_poly, centers, radius );
 

            balls_center.push_back(Point2i(centers));
            ball_color_ref.push_back(label_with_color[i][0][0]);
    

        }
        else  // 공이 겹친 경우 (2개 3개 3개 모두 가능. 너무 겹치면 정확도가 낮아진다.)
        {
            vector<Point2i> centers;
            Mat points;
            Mat labels;
            findNonZero(temp, points);
            points.convertTo(points, CV_32F);


            kmeans(points, overlapped, labels, TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 3,
            KMEANS_PP_CENTERS, centers );

            //각각의 중심이 어느 색 color인지 판단해야 한다.
        

            for(int j=0; j<overlapped ; j++){
                int center_x = centers[j].x;
                int center_y = centers[j].y;
                float min_color_index;
                float min=1000000;

                for(int c=0; c<overlapped ; c++){
                    int color_index =label_with_color[i][c][0];
                    int color_x =label_with_color[i][c][1];
                    int color_y =label_with_color[i][c][2];
                    float dist = sqrt((center_x-color_x)*(center_x-color_x)+(center_y-color_y)*(center_y-color_y));

                    if(dist < min){
                        min = dist;
                        min_color_index = color_index;
                    }

                }
                balls_center.push_back(Point2i(centers[j]));
                ball_color_ref.push_back(min_color_index);
            }
    
        }

    }


}