#include "basic.h"
#include "function.h"



void Detection_Main(void){
    
    Mat img;
    Mat blue_and_morph;
    Mat ball_colors[3]; 
    Mat hole;
    Mat Big_blob, Big_blob_without_hole;

    Point2i Big_blob_center;
    vector<Point2i> balls_center;
    vector<Point2i> corners;
    vector<Vec4f> Candidate_lines;

    vector<Mat> ball_candidate;
    vector<vector<int>> label_with_color;

    for(int i=0; i<=16 ; i++){  
        
        stringstream str;
        //str << "images/image" << std::setw(2) << setfill('0') << i << ".jpg";
        str << "KaKao/KakaoTalk_20210507_232928365_" << std::setw(2) << setfill('0') << i << ".jpg";
        img = imread(str.str());
        
        /*
        if(img.cols>img.rows)
            resize(img, img, Size(1000,562));
        else  
            resize(img, img, Size(562,1000)); */

    
        
    Detect_Billiard(img, blue_and_morph);
    Detect_ball_color(img, ball_colors);  // hsv 영상을 미리 만들어서 매개변수로 보내자. 애초에 hsv 영상으로 imread 해도 될듯.
    

    Extract_Biggest_Blob_with_Center(blue_and_morph, Big_blob, Big_blob_center);

    Detect_Billiard_Hole(Big_blob, hole, Big_blob_without_hole);

    Detect_Billiard_Edge(Big_blob_without_hole, Big_blob_center, Candidate_lines);
    
    Calculation_Billiard_Corner(Candidate_lines, Big_blob_center, corners, img);

    Match_ball_and_color(ball_colors, hole, ball_candidate, label_with_color);

    //Find_ball_center(ball_candidate, label_with_color, balls_center);


    int size = corners.size();
    for(int j=0; j< size ;j++)
        circle(img, Point(corners[j].x, corners[j].y), 50, Scalar(0, 255, 0), 2, 8, 0);
    
    imshow("img",img);
    waitKey();








    // 따로 함수 만들자
    corners.clear();
    ball_candidate.clear();
    label_with_color.clear();
 

    /*
    imshow("img",img);
    imshow("blob", Big_blob);
    imshow("hole", hole);
     imshow("red", ball_colors[0]);
     imshow("yellow", ball_colors[1]);
    imshow("white", ball_colors[2]);
   
    waitKey(0);
    */


    }
}

void Detect_Billiard(Mat& img, Mat& output){


    Mat img_hsv;
	cvtColor(img, img_hsv, COLOR_BGR2HSV);

	Mat Blue, Blue_mask;
    Mat result;

	Scalar lower_blue = Scalar(97, 130, 50);
	Scalar upper_blue = Scalar(123, 255, 255);

	inRange(img_hsv, lower_blue, upper_blue, Blue_mask);

    // 모폴로지 부분
    Mat morph;

    Mat element(5,5, CV_8U, Scalar(1));    // 꼭 필요하지는 않다.
    morphologyEx(Blue_mask, morph, MORPH_OPEN, element);

    output = Blue_mask;
}

void Detect_ball_color(Mat& img, Mat* output_array){
    Mat img_hsv;
	cvtColor(img, img_hsv, COLOR_BGR2HSV);


	Mat Red_m_1, Red_m_2, Red_mask;
	Mat Yellow_mask;
    Mat White_mask;


	Scalar lower_red1 = Scalar(160, 130, 50);   // 오히려 분홍쪽에 색이 가까웠음
	Scalar upper_red1 = Scalar(179, 255, 255);

    Scalar lower_red2 = Scalar(0, 130, 50);
	Scalar upper_red2 = Scalar(7, 255, 255);

    Scalar lower_yellow = Scalar(3, 130, 50);   // 그림자 부분은 거의 붉은색이다
	Scalar upper_yellow = Scalar(35, 255, 255);


    Scalar lower_white = Scalar(0, 0, 50);     // 흰색은 채도가 없는 것? but.. 그렇게하면 안됨..
	Scalar upper_white = Scalar(179, 100, 255);


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

void Detect_Billiard_Hole(Mat& input, Mat& output1, Mat& output2) // ** 영상모서리와 당구대 모서리가 겹치는 부분도 구멍으로 간주함 **
{
    Mat border, morph;
    
    const int H_Size = 55;

    copyMakeBorder(input, border, H_Size+1,H_Size+1,H_Size+1,H_Size+1, BORDER_CONSTANT, Scalar(0));

    Mat element(H_Size,H_Size, CV_8U, Scalar(1));
    morphologyEx(border, morph, MORPH_CLOSE, element);  //


    Mat subtract = morph - border;


    Mat eroded;
    erode(subtract, eroded, Mat::ones(Size(3,3),CV_8UC1),Point(-1,-1),1);
    Mat edge = subtract- eroded;




    Mat subtract_roi = subtract(Rect(H_Size+1,H_Size+1, input.cols, input.rows));
    output1 = subtract_roi;
    output2 = output1+ input;
}


void Extract_Biggest_Blob_with_Center(Mat& input, Mat& output, Point2i& center)
{
    Mat img_label, stats, centroids;
    int numOfLables = connectedComponentsWithStats(input, img_label, stats, centroids, 4, CV_32S);
    
    int max_area = 0;
    int max_area_index;
    for (int j = 1; j < numOfLables; j++) {  // 0 is backgraound label..
        int* label = stats.ptr<int>(j);
        int area = label[4];

        if(max_area < area ){
            max_area = area;
            max_area_index = j;
        }
    }
    
    double * centroid = centroids.ptr<double>(max_area_index);
    center.x = centroid[0];
    center.y = centroid[1];
    
    output = img_label == max_area_index;
}

void Detect_Billiard_Edge(Mat& Big_blob, Point2i& Big_blob_center, vector<Vec4f>& Candidate_lines){
    Mat eroded, edge;
    vector<Vec4f> candidate_lines_info; 

    erode(Big_blob, eroded, Mat::ones(Size(3,3),CV_8UC1),Point(-1,-1),1);
    edge = Big_blob - eroded;
    
    //Canny(Big_blob, edge, 10, 200);

    vector<Vec2f> lines;
	HoughLines(edge, lines, 1, CV_PI / 180, 50);
	

    std::vector<cv::Vec2f>::const_iterator it= lines.begin();
	while (it!=lines.end()) {

		float rho= (*it)[0];   // first element is distance rho
		float theta= (*it)[1]; // second element is angle theta

        
        float a = -cos(theta)/sin(theta);
        float b = -1.0;
        float c = rho/sin(theta);
        
        float p = Big_blob_center.x; // p q 는 big_blob 의 중심이다.
        float q = Big_blob_center.y;
        
        float foot = -(a*p +b*q + c)/((a*a) + (b*b)); 
        float foot_x = foot*a + p;
        float foot_y = foot*b + q;

        int c_l_size = candidate_lines_info.size();
        bool flag = false;

            
            for(int i =0; i<c_l_size; i++)
            {   
                float c_theta = candidate_lines_info[i][1];
                float c_x =  candidate_lines_info[i][2] - p;
                float c_y =  candidate_lines_info[i][3] - q;
                float x =  foot_x - p;
                float y =  foot_y - q;

                
                double Rad = Vector_Degree(x, c_x, y, c_y);

                double diff_theta = abs(c_theta - theta) < CV_PI ? abs(c_theta - theta) : CV_PI- abs(c_theta - theta);
                

                if(diff_theta < 0.17 && Rad < 0.5) // 비슷한 종류의 직선이다.
                    flag=true;
                
            }
        
        if(!flag)
            candidate_lines_info.push_back(Vec4f(rho,theta,foot_x, foot_y));

        ++it;
    }

    Candidate_lines = candidate_lines_info;

}

void Calculation_Billiard_Corner(vector<Vec4f>& Candidate_lines, Point2i& Big_blob_center,vector<Point2i>& corners, Mat& temp){
    
    Mat border;
    float B_x = Big_blob_center.x;
    float B_y = Big_blob_center.y;

    float fx_1, fy_1,fx_2, fy_2; // 수선의 발 변수


    const int H_Size = 150;  // 교점 확인용이다. 나중에 지워야 한다.
    copyMakeBorder(temp, border, H_Size+1,H_Size+1,H_Size+1,H_Size+1, BORDER_CONSTANT, Scalar(0));
    circle(border, Point(B_x+H_Size, B_y+H_Size), 5, Scalar(0, 0, 255), 1, 8, 0);

    int line_num = Candidate_lines.size();
    vector<cv::Vec4f>::const_iterator it = Candidate_lines.begin();
    vector<Vec3f> candidate_corners;


    if(line_num == 2){

        Point2i sol = Get_Intersect_Point(Candidate_lines[0][0], 
        Candidate_lines[0][1],
        Candidate_lines[1][0],
        Candidate_lines[1][1]);
        /*
        fx_1 = Candidate_lines[0][2];
        fy_1 = Candidate_lines[0][3];
        fx_2 = Candidate_lines[1][2];
        fy_2 = Candidate_lines[1][3];

        circle(border, Point(fx_1+H_Size, fy_1+H_Size), 5, Scalar(0, 0, 255), 1, 8, 0);
        circle(border, Point(fx_2+H_Size, fy_2+H_Size), 5, Scalar(0, 0, 255), 1, 8, 0);



        double Rad = Vector_Degree(fx_1-B_x, fx_2-B_x, fy_1-B_y, fy_2-B_y); */

        if(!(sol.x==-1 && sol.y==-1) )  // 2.6은 150도
            corners.push_back(sol);
    }
    else if(line_num == 3){    // 불안한 방법이다.  예각일때 실패.**
        set<pair<int,int>> s;
    
        while (it != Candidate_lines.end()) {

		float rho1= (*it)[0];  
		float theta1= (*it)[1]; 
        vector<cv::Vec4f>::const_iterator it2 = Candidate_lines.begin();

        while (it2 != Candidate_lines.end()) {

		    float rho2= (*it2)[0];  
		    float theta2= (*it2)[1]; 
            if(it == it2){
                it2++;
                continue;
            }

            Point2i sol = Get_Intersect_Point(rho1,theta1,rho2,theta2);


            if( (sol.x == -1 && sol.y == -1 ) ){  // 각도차이가 너무작거나 동일한 직선이면 교점을 구하지 않는다.
                it2++;
                continue;
            }

            fx_1 = (*it)[2];
            fy_1 = (*it)[3];
            fx_2 = (*it2)[2];
            fy_2 = (*it2)[3];

            circle(border, Point(fx_1+H_Size, fy_1+H_Size), 5, Scalar(0, 0, 255), 1, 8, 0);
            circle(border, Point(fx_2+H_Size, fy_2+H_Size), 5, Scalar(0, 0, 255), 1, 8, 0);

            if(theta1 > CV_PI){ theta1 -= CV_PI;}
            if(theta2 > CV_PI){ theta2 -= CV_PI;}
            double diff_theta = abs(theta1 - theta2) < CV_PI ? abs(theta1 - theta2) : CV_PI- abs(theta1 - theta2);


            double Rad = Vector_Degree(fx_1-B_x, fx_2-B_x, fy_1-B_y, fy_2-B_y);
            candidate_corners.push_back(Vec3f(sol.x, sol.y, Rad));


            it2++;
        }

        sort(candidate_corners.begin(), candidate_corners.end(), cmp);
        
        s.insert(make_pair(candidate_corners[0][0], candidate_corners[0][1]));

        candidate_corners.clear();

		 ++it;
	    }

        set<pair<int,int>>::iterator it = s.begin();
        corners.push_back(Point2i((*it).first,(*it).second));
        it++;
        corners.push_back(Point2i((*it).first,(*it).second));

    }
    else if(line_num == 4){   // 직선이 4개일떄

    vector<vector<Vec3i>> lines_with_point(4);
    set<pair<int,int>> intersect_point;
    vector<pair<int,int>> m_pm_e;

    int line_num=0;
    while (it != Candidate_lines.end()) {

		float rho1= (*it)[0];  
		float theta1= (*it)[1]; 
        vector<cv::Vec4f>::const_iterator it2 = Candidate_lines.begin();


            while (it2 != Candidate_lines.end()) {
            
            if(it == it2){
                it2++;
                continue;
            }

		    float rho2= (*it2)[0];  
		    float theta2= (*it2)[1]; 
            Point2i sol = Get_Intersect_Point(rho1,theta1,rho2,theta2);
            
                        
            if( (sol.x == -1 && sol.y == -1 ) ){  // 각도차이가 너무작거나 동일한 직선이면 교점을 구하지 않는다.
                it2++;
                continue;
            }

            intersect_point.insert(make_pair(sol.x, sol.y));
            lines_with_point[line_num].push_back(Vec3i(sol.x, sol.y,0));

            it2++;
            }
		++it;
        line_num++;
	}

    // 교점이 4개일때, 5개 일떄는 line_with_point 의 갯수가 2개인 점들이 코너점이다.

    int i_p_size = intersect_point.size();
    if(i_p_size==4){

        set<pair<int,int>>::iterator iter = intersect_point.begin();
        while(iter != intersect_point.end()){
            corners.push_back(Point2i((*iter).first, (*iter).second));
            iter++;
        }

        for(int i=0; i<4 ;i++){
        circle(border, Point(corners[i].x+H_Size, corners[i].y+H_Size), 150, Scalar(0, 255, 0), 2, 8, 0);
        }
    }
    else if(i_p_size == 5){    // 2개의 교점을 가지는 직선을 주목해야한다.
        for(int i=0; i<4 ; i++){
            if(lines_with_point[i].size() == 2 ){
                for(int j=0; j<2 ; j++)
                    corners.push_back(Point2i(lines_with_point[i][j][0], lines_with_point[i][j][1]));
            }
        }
    }
    else if(i_p_size==6){

        for(int i=0; i<4 ;i++){
            for(int j=0; j<3 ; j++){  // 직선당 3개의 교점
                int X = lines_with_point[i][j][0]; 
                int Y = lines_with_point[i][j][1];

                int x1 = lines_with_point[i][(j+1)%3][0];
                int y1 = lines_with_point[i][(j+1)%3][1];
                int x2 = lines_with_point[i][(j+2)%3][0];
                int y2 = lines_with_point[i][(j+2)%3][1];
                
                int dot = (x1-X)*(x2-X) + (y1-Y)*(y2-Y);

                if( dot < 0 ){  // 중심점임
                    lines_with_point[i][j][2]++;
                    break;
                }

            }
        }

        


        for(int i=0; i<4 ;i++){
            for(int j=0; j<3 ; j++){
                int n = lines_with_point[i][j][2];
                if(n == 1)
                    m_pm_e.push_back(make_pair(lines_with_point[i][j][0],lines_with_point[i][j][1]));
            }
        }

        // m_pm_e[1]  : (m m) (pm) (pm) 임. // 무조건 코너 보장. m m 이 중복된 것의 반대편에있는 점이 마지막 코너
        //
        sort(m_pm_e.begin(), m_pm_e.end());
        pair<int, int> tem = m_pm_e[0];
        pair<int,int> m;
        corners.push_back(Point2i(m_pm_e[0].first,m_pm_e[0].second));

        for(int i=1; i<4 ;i++){   // 중복 제거 및 중복원소 검출
            if(tem == m_pm_e[i]){
                m = tem;                 // m은 중점이 두번 겹친 코너
                continue;
            }

            corners.push_back(Point2i(m_pm_e[i].first,m_pm_e[i].second));
            tem = m_pm_e[i];
        }

        
        int line_table[4]={0,}; // 중복원소(m)가 속한 line 찾기
        for(int i=0; i<4 ; i++){
            for(int j=0 ; j<3 ; j++){
                if(lines_with_point[i][j][0] == m.first && lines_with_point[i][j][1] == m.second)
                    line_table[i]=1;
            }
        }
    
        // 중복원소가 속하지 않는 line 의 index 담기
        int edge_line_num[2];
        int next=0;


        for(int i=0; i<4 ;i++){
            if(line_table[i] == 0)
                edge_line_num[next++]=i;
        }

        int l_1 =edge_line_num[0];
        int l_2 = edge_line_num[1];

        
        for(int i=0; i<3; i++){
            for(int j=0; j<3 ;j++){
                if(lines_with_point[l_1][i][0] == lines_with_point[l_2][j][0] &&
                    lines_with_point[l_1][i][1] == lines_with_point[l_2][j][1] ){
                        corners.push_back(Point2i(lines_with_point[l_1][i][0],lines_with_point[l_1][i][1]));
                    }

            }
        }
        
    } 


    }
    else
        cout<<"We cannot find Billiard"<<endl;

    /*
    int size = corners.size();
    cout<<size<<endl;
    for(int i=0; i<size ;i++){
        circle(border, Point(corners[i].x+H_Size, corners[i].y+H_Size), 150, Scalar(0, 255, 0), 2, 8, 0);
    }
    imshow("img", border);
    
    waitKey(0);
    */
}


double Vector_Degree(double x, double c_x, double y, double c_y){
    double dot_pro = ( x * c_x ) + ( y * c_y );
    double dDen = ( sqrt( pow( x, 2 ) + pow( y, 2 ) ) * sqrt( pow( c_x, 2 ) + pow( c_y, 2 ) ) );
    double ratio = dot_pro / dDen;
    double Rad;
                
        if(ratio >= 1.0)  // nan 문제 발생.
            Rad=  acos( ratio - 0.001 );
        else if(ratio <= -1.0)
            Rad=  acos( ratio + 0.001 );
        else
            Rad=  acos( dot_pro / dDen );
    
    return Rad;
}


Point2i Get_Intersect_Point(float r1, float t1, float r2, float t2){
    
    float rho1= r1;  
	float theta1= t1; 

    float rho2= r2;  
	float theta2= t2; 

    double diff_theta = abs(theta1 - theta2) < CV_PI ? abs(theta1 - theta2) : CV_PI- abs(theta1 - theta2);

    if(diff_theta < 0.2)
        return Point2i(-1, -1);

    float a1 = -cos(theta1)/sin(theta1);
    float c1 = rho1/sin(theta1);

    float a2 = -cos(theta2)/sin(theta2);
    float c2 = rho2/sin(theta2);


    float x = (c2-c1)/(a1-a2);
    float y = a1*x + c1;

    return Point2i(x,y);

}

bool cmp(const Vec3f &p1, const Vec3f &p2){

    if(p1[2] < p2[2]){
        return true;
    }
    else{
        return false;
    }
}

bool cmp2(const pair<int, int>& a, const pair<int,int>& b){
    return a.first > b.first;
}


void Match_ball_and_color(Mat* ball_colors, Mat& hole, vector<Mat>& ball_candidate, vector<vector<int>>& label_with_color){
    label_with_color.resize(4);
    vector<pair<int,int>> find_best_color_area;

    int total;
    Mat eroded;

    //erode(hole, eroded, Mat::ones(Size(3,3),CV_8UC1),Point(-1,-1),1); // 노이즈 제거용 *** 속도 개선이 필요하면 고려 ***


    
    Mat img_label, stats, centroids;
    int numOfLables = connectedComponentsWithStats(hole, img_label, stats, centroids, 4, CV_32S);

    if(numOfLables == 0 || numOfLables == 1){ cout<<"No balls detected!"; }
    


    vector<pair<int,int>> FBFL; // Find_Best_Four_Label
    for (int j = 1; j < numOfLables; j++) { 
        int* label = stats.ptr<int>(j);
        int area = label[4];
        FBFL.push_back(make_pair(area, j));

    }




    if((int)(FBFL.size()) >= 4){
        //nth_element(FBFL.begin(), FBFL.begin()+ 4, FBFL.end(), cmp2); 
        // nth는 4번째까지 빠르게 고르지만 4번째까지는 대신 정렬이 안되어있음.. 굳이 필요 없으면 위에걸로 바꾸자
        sort(FBFL.begin(), FBFL.end(), cmp2);
        total = 4;
    }
    else{
        sort(FBFL.begin(), FBFL.end(), cmp2);
        total = numOfLables-1;
    }

    /*
    for(int i=0; i<total ; i++){
        int l = FBFL[i].second;
        int temp = sum(l == img_label)[0]/255;

        int* label = stats.ptr<int>(l);
        int area = label[4];

        cout<<temp<<' '<<area<<endl;
        Mat t = l == img_label;
        imshow("t", t);
        waitKey();
    }
    cout<<endl;
    */

    // ball 후보들을 따로 분리해준다.
    // i 번쨰 라벨(hole) 은 몇 번째 j 컬러 일까?
    

    for(int i=0; i<total ; i++){
        int label = FBFL[i].second;
        Mat ball_candi = label == img_label;
        int sum1 = sum(ball_candi)[0]/255;
        bool is_the_ball=false;

        // ********* 특정 픽셀 비율 이하면 공이라고 판단 안하고 건너 띈다!! **************
        if(sum1 < 100)
            continue;


        for(int j=0; j<3 ; j++){
            Mat what_color;
            bitwise_and(ball_candi, ball_colors[j], what_color );

            int area = sum(255 == what_color)[0]/255;
            find_best_color_area.push_back(make_pair(area, j));

        }

        sort(find_best_color_area.begin(), find_best_color_area.end(), cmp2);
        int best_color_index = find_best_color_area[0].second;
        int best_color_area = find_best_color_area[0].first;
        double second_color_ratio; // 두 번째로 우세한 색의 첫 번쨰로 우세한 색의 비율
        double third_color_ratio;
        if(best_color_area > 0){
            second_color_ratio = (double)(find_best_color_area[1].first) / best_color_area;
            third_color_ratio = (double)(find_best_color_area[2].first) / best_color_area;

            label_with_color[i].push_back(best_color_index);

            if(second_color_ratio > 0.4) 
                label_with_color[i].push_back(find_best_color_area[1].second);  
            if(third_color_ratio > 0.4 )
                label_with_color[i].push_back(find_best_color_area[2].second);

            is_the_ball = true;
        }
        else{
            cout<<"This label is not a ball!";  // 겹치는 색이 없는 라벨이면.. 그냥 공이 아닌거다.
        }

        //cout<<"index: "<<best_color_index<<" "<<"Second ratio: "<< second_color_ratio<<" "<<"Third ratio: "<< third_color_ratio;
        /*
        cout<<sum1<<' ';
        imshow("label", ball_candi);
        waitKey();
        */
        
        if(is_the_ball)
            ball_candidate.push_back(ball_candi);

        find_best_color_area.clear();
    }
        cout<<endl;

    /*
    Mat edge = hole - eroded;

    vector<vector<Point> > contours;
    findContours( edge, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>centers( contours.size() );
    vector<float>radius( contours.size() );
    
    for( size_t i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( contours[i], contours_poly[i], 3, true );
        minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
    }
    Mat drawing = Mat::zeros( edge.size(), CV_8UC3 );
    
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( 0,0,125 );
        drawContours( drawing, contours_poly, (int)i, Scalar(255,255,255) );
        circle( drawing, centers[i], 1, color, 1);
        //(int)radius[i], color, 1 );
    }
    imshow("edge", edge);
    imshow( "Contours", drawing );
    waitKey();

    */

    /*
    for(int i=0; i<numOfLables ; i++){
        Mat temp = i == img_label;
        double * centroid = centroids.ptr<double>(i);


        cvtColor(temp, temp, COLOR_GRAY2BGR);
        int area = stats.at<int>(i, CC_STAT_AREA);
		int left = stats.at<int>(i, CC_STAT_LEFT);
		int top = stats.at<int>(i, CC_STAT_TOP);
		int width = stats.at<int>(i, CC_STAT_WIDTH);
		int height = stats.at<int>(i, CC_STAT_HEIGHT);


		rectangle(temp, Point(left, top), Point(left + width, top + height),
			Scalar(0, 0, 255), 1);

        

        circle(temp, Point(centroid[0], centroid[1]), 3, Scalar(0,200,0), 2, 8, 0);
        imshow("temp", temp);
        waitKey();
    }
    */

    //waitKey(0);

}


void Find_ball_center(vector<Mat>& ball_candidate, vector<vector<int>>& label_with_color, vector<Point2i>& balls_center){
    

    int size = ball_candidate.size();
    for(int i=0; i<size ; i++){
        Mat temp = ball_candidate[i];
        int overlapped = label_with_color[i].size();

        if(overlapped < 2){
        
        cout<<label_with_color[i][0]<<endl;

        Mat eroded;
        erode(temp, eroded, Mat::ones(Size(3,3),CV_8UC1),Point(-1,-1),1); // 노이즈 제거용
        Mat edge = temp - eroded;

        vector<vector<Point> > contours;
        findContours( edge, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );

        vector<vector<Point> > contours_poly( contours.size() );
        vector<Rect> boundRect( contours.size() );
        vector<Point2f>centers( contours.size() );
        vector<float>radius( contours.size() );
    
    for( size_t i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( contours[i], contours_poly[i], 3, true );
        minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
    }
    Mat drawing = Mat::zeros( edge.size(), CV_8UC3 );
    
    cvtColor(temp, temp, COLOR_GRAY2BGR);
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( 0,0,125 );
        //drawContours( drawing, contours_poly, (int)i, Scalar(255,255,255) );
        circle( temp, centers[i], 2, color, 2);
        //(int)radius[i], color, 1 );
    }

        
    }
    else
    {
        vector<Point2f> centers;
        Mat points;
        findNonZero(temp, points);
        points.convertTo(points, CV_32F);
        cout<<points<<endl;
        cout<<points.type()<<endl;


        kmeans(points, overlapped, Mat(), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 3,
        KMEANS_PP_CENTERS, centers );


        for( int i = 0; i< (int)centers.size(); i++ )
            circle( temp, centers[i], 2, Scalar( 0,255,0 ), 2);

        cout<<"overlapped!"<<endl;
    }
        imshow("mat", temp);
        waitKey();
    



    }

    /*
    int si = ball_candidate.size();
    
    for(int j=0; j<si ; j++){
        int s = label_with_color[j].size();
        for(int k=0; k<s ; k++)
            cout<<label_with_color[j][k]<<' ';
        imshow("mat", ball_candidate[j]);
        waitKey();
    }
    cout<<endl;
    */
}