  
  /*
   Mat result;
    temp.copyTo(result);
    Scalar color(0,0,255);

    vector<cv::Vec4f>::const_iterator iter = Candidate_lines.begin();

	while (iter != Candidate_lines.end()) {

		float rho= (*iter)[0];   // first element is distance rho
		float theta= (*iter)[1]; // second element is angle theta
		
		if (theta < CV_PI/4. || theta > 3.*CV_PI/4.) { // ~vertical line
		
			// point of intersection of the line with first row
			cv::Point pt1(rho/cos(theta),0);        
			// point of intersection of the line with last row
			cv::Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
			// draw a white line
			cv::line( result, pt1, pt2, color, 3); 

		} else { // ~horizontal line

			// point of intersection of the line with first column
			cv::Point pt1(0,rho/sin(theta));        
			// point of intersection of the line with last column
			cv::Point pt2(result.cols,(rho-result.cols*cos(theta))/sin(theta));
			// draw a white line
			cv::line( result, pt1, pt2, color, 3); 
		}

		//std::cout << "line: (" << rho << "," << theta << ")\n"; 


		++iter;
	}
    cout<<endl;

    imshow("result", result);
    waitKey();
    */


   /*
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
            it2++;
            }
		++it;
	    }
        //  여기까지 4~6 개의 교차점이 발생한다.  최종 4개의 교차점을 골라내야한다.
        
        vector<pair<int,int>> temp(intersect_point.begin(), intersect_point.end());
        
        do
        {
           
            int line_table[4]={0,};
            int size = Candidate_lines.size();
            int cnt = 0;
                
            for (int i=0; i<size ; i++) {

		    float rho= Candidate_lines[i][0];  
		    float theta= Candidate_lines[i][1];
                    
            float a = -cos(theta)/sin(theta);
            float b = -1.0;
            float c = rho/sin(theta);

            // 앞의 4개의 점에 대하여
            vector<pair<int,int>>::const_iterator it = temp.begin();
                for(int j=0; j<4 ; j++){

                    int x = (*it).first;
                    int y = (*it).second;
                    
                    //cout<<a*x+b*y+c<<endl;
                    if(a*x+b*y+c >= -10 && a*x+b*y+c <=10){
                        line_table[i]++;
                    }
                    
                    it++;
                }
            }
            
            int clm=0;
            for(int i=0; i<4; i++){
                if(line_table[i] == 2)
                    cnt++;
 
                clm += line_table[i];
            }
            
            cout<<clm<<','<<size<<endl;
            if(cnt == 4){
                cout<<"pass"<<endl;
                cout<<(*temp.begin()).first<<endl;
                break;
            }

        }
        while(next_permutation(temp.begin(), temp.end()));
            
        vector<pair<int,int>>::const_iterator it = temp.begin();
        cout<<(*temp.begin()).first<<endl;

        for(int j=0; j<4 ; j++){

            int x = (*it).first;
            int y = (*it).second;
            cout<<x<<','<<y<<endl;
        corners.push_back(Point2i(x,y));
        circle(border, Point(x+H_Size, y+H_Size), 100, Scalar(0, 255, 0), 2, 8, 0);
         it++;
         
        }



   */



  /*
   // line 이 3개일떄 교차점 구하는 다른 방법
   
        while (it != Candidate_lines.end()) {

		float rho1= (*it)[0];  
		float theta1= (*it)[1]; 
        vector<cv::Vec4f>::const_iterator it2 = Candidate_lines.begin();

        while (it2 != Candidate_lines.end()) {
            
            //cout<<*it<<','<<*it2<<endl;

		    float rho2= (*it2)[0];  
		    float theta2= (*it2)[1]; 
            if(it == it2){
                it2++;
                continue;
            }

            Point2i sol = Get_Intersect_Point(rho1,theta1,rho2,theta2);
            float x = Big_blob_center.x;
            float y = Big_blob_center.y;
            //float dist =  sqrt( pow( x-sol.x, 2 ) + pow( y-sol.y, 2 ) );   // 영상 중심과 코너 사이의 거리


             circle(border, Point(x+H_Size, y+H_Size), 5, Scalar(0, 0, 255), 1, 8, 0);


            if( (sol.x == -1 && sol.y == -1 ) ){  // 각도차이가 너무작거나 동일한 직선이면 교점을 구하지 않는다.
                it2++;
                continue;
            }

            float fx_1 = (*it)[2];
            float fy_1 = (*it)[3];
            float fx_2 = (*it2)[2];
            float fy_2 = (*it2)[3];

            circle(border, Point(fx_1+H_Size, fy_1+H_Size), 5, Scalar(0, 0, 255), 1, 8, 0);
            circle(border, Point(fx_2+H_Size, fy_2+H_Size), 5, Scalar(0, 0, 255), 1, 8, 0);

            if(theta1 > CV_PI){ theta1 -= CV_PI;}
            if(theta2 > CV_PI){ theta2 -= CV_PI;}
            double diff_theta = abs(theta1 - theta2) < CV_PI ? abs(theta1 - theta2) : CV_PI- abs(theta1 - theta2);


            double Rad = Vector_Degree(fx_1-x, fx_2-x, fy_1-y, fy_2-y);

            candidate_corners.push_back(Vec3f(sol.x, sol.y, Rad));

            cout<<sol.x<<','<< sol.y<<','<< Rad<<endl;
            it2++;
        }

        sort(candidate_corners.begin(), candidate_corners.end(), cmp);
        
        if(line_num==3)
            corners.push_back(Point2i(candidate_corners[0][0], candidate_corners[0][1]));

        if(line_num == 4){
            corners.push_back(Point2i(candidate_corners[0][0], candidate_corners[0][1]));
            corners.push_back(Point2i(candidate_corners[1][0], candidate_corners[1][1]));
        }

        candidate_corners.clear();


        cout<<endl;
		 ++it;
	    }

        int size = corners.size();
        cout<<size<<endl;
        for(int i=0; i<size ; i++)
            circle(border, Point(corners[i].x+H_Size, corners[i].y+H_Size), 50, Scalar(0, 255, 0), 1, 8, 0);

  */
 

 /*


    corners.clear();

           vector<Vec3f> circles;
           Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    // Apply the Hough Transform to find the circles
    HoughCircles( gray, circles, HOUGH_GRADIENT, 1, 2, 200, 50, 1, 50 );

    cvtColor(gray, gray, COLOR_GRAY2BGR);
    // Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);     
        circle( gray, center, 3, Scalar(0,255,0), -1, 8, 0 );// circle center     
        circle( gray, center, radius, Scalar(0,0,255), 3, 8, 0 );// circle outline
        cout << "center : " << center << "\nradius : " << radius << endl;
    }

    imshow("gray", gray);
    waitKey();


    */


       // 당구공 채도실험
    /*
	lower_red1 = Scalar(170, 50, 50);
	upper_red1 = Scalar(179, 255, 255);

    lower_red2 = Scalar(0, 50, 50);
	upper_red2 = Scalar(10, 255, 255);
    
	inRange(img_hsv, lower_red1, upper_red1, Red_m_1);
    inRange(img_hsv, lower_red2, upper_red2, Red_m_2);
    bitwise_or(Red_m_1, Red_m_2, Red_mask);

    imshow("red ball: 50", Red_mask);
    */

   /*
   lower_yellow = Scalar(11, 130, 50);   // 그림자 부분은 거의 주황색임
    upper_yellow = Scalar(35, 255, 255);
    inRange(img_hsv, lower_yellow, upper_yellow, Yellow_mask);
    imshow("yellow ball: 120", Yellow_mask);
    */
    


/*


void Extract_Biggest_Blob(Mat& input, Mat& output)
{
    int w = input.cols;
    int h = input.rows;

    Mat Label = Mat::zeros(h,w, CV_8U);   // 더큰 공간 확보 필요할 수도.

    vector<int> cmp_num_list;
    cmp_num_list.resize(256);
    vector<Point2i> cmp_init_pos;
    cmp_init_pos.resize(256);

    int Label_num=1;
    int cmp_num;


    for(int i=0; i<h ; i++){
        for(int j=0; j<w ; j++){
            
            uchar* L_row = Label.ptr<uchar>(i);
            uchar* input_row = input.ptr<uchar>(i);
            if(!L_row[j] && input_row[j] && Label_num < 256){


                cmp_num = Labeling(input ,Label, Label_num, Point2i(j,i));
                cmp_init_pos[Label_num] = Point2i(j,i);
                cmp_num_list[Label_num] = cmp_num;

                Label_num++;
            }
        }
    }


    int max_cmp_label = max_element(cmp_num_list.begin(), cmp_num_list.end()) - cmp_num_list.begin();
    Point2i max_cmp_pos = cmp_init_pos[max_cmp_label];


   Mat result = Label == max_cmp_label;

    output = result;
}

int Labeling(Mat& input, Mat& Label, int L_n, Point2f init)
{
    int w = input.cols;
    int h = input.rows;
    int init_x = init.x;
    int init_y = init.y;

    int cmp_num=1;

    queue<Point2i> q;

    int next_x[4] = {0,0,-1,1};
    int next_y[4] = {1,-1,0,0};

    q.push(init);
               
    uchar* p_label= Label.ptr<uchar>(init_y);
    uchar* p_input;

    p_label[init_x] = L_n;

    
    while(!q.empty()){
        Point2i now = q.front();
        q.pop();

        for(int i=0; i<4 ; i++){
            int nx = next_x[i] + now.x; 
            int ny = next_y[i] + now.y;

            if(nx < 0 || nx >= w || ny < 0 || ny >= h)
                continue;
            
            p_label= Label.ptr<uchar>(ny);
            p_input= input.ptr<uchar>(ny);

            if(!p_label[nx] && p_input[nx]){
                p_label[nx] = L_n;
                q.push(Point2i(nx, ny));
                cmp_num++;
            }
            
        }
    }

    return cmp_num;
}

*/