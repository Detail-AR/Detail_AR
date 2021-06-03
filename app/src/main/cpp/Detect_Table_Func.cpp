#include "basic.h"
#include "add_ons.h"


void Detect_Billiard(Mat& img_hsv, Mat& output){


	Mat Blue, Blue_mask;
    Mat result;

	Scalar lower_blue = Scalar(97, 120, 50);
	Scalar upper_blue = Scalar(123, 255, 255);

	inRange(img_hsv, lower_blue, upper_blue, Blue_mask);

    // 모폴로지 부분
    Mat morph;

    Mat element(5,5, CV_8U, Scalar(1));    // 꼭 필요하지는 않다.
    morphologyEx(Blue_mask, morph, MORPH_OPEN, element);

    output = Blue_mask;
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
	HoughLines(edge, lines, 1, CV_PI / 180, 80);     /** 중요 파라미터 **/
	

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
                

                if(diff_theta < 0.17 && Rad < 0.5) // 비슷한 종류의 직선이다. //****** 파라미터
                    flag=true;
                
            }
        
        if(!flag)
            candidate_lines_info.push_back(Vec4f(rho,theta,foot_x, foot_y));

        ++it;
    }

    Candidate_lines = candidate_lines_info;  // 내적이 필요 없다면 수정

}

void Calculation_Billiard_Corner(vector<Vec4f>& Candidate_lines, Point2i& Big_blob_center,vector<Point2i>& corners){
    
    Mat border;
    float B_x = Big_blob_center.x;
    float B_y = Big_blob_center.y;

    float fx_1, fy_1,fx_2, fy_2; // 수선의 발 변수

    
    const int H_Size = 150;  // 교점 확인용이다. 나중에 지워야 한다.
   /* copyMakeBorder(temp, border, H_Size+1,H_Size+1,H_Size+1,H_Size+1, BORDER_CONSTANT, Scalar(0));
    circle(border, Point(B_x+H_Size, B_y+H_Size), 5, Scalar(0, 0, 255), 1, 8, 0);*/

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
