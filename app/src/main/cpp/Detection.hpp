#ifndef Detection_h
#define Detection_h

#include "basic.h"

class Detection
{

    private:

    Mat img;
    Mat img_hsv;
    Mat blue_and_morph;
    Mat ball_colors[3]; 
    Mat hole;
    Mat Big_blob, Big_blob_without_hole;
    

    Point2i Big_blob_center;
    vector<Point2i> corners;
    vector<Vec4f> Candidate_lines;

    vector<Mat> ball_candidate;   // 최종 공의 후보들. 겹친 공도 존재
    vector<vector<Vec3i>> label_with_color;  // 공의 후보들의 색index, 그리고 색의 위치(겹친 공이면 어느 색 공이 어디있는지 알기위해)
    vector<Point2i> balls_center;
    vector<int> ball_color_ref;


    public:
    //Detection(); // 1000x562 픽셀 기준으로 탐지. 생성자 : 


    void Set_Image(Mat& input_img);

    int Detect_Billiard_Corners(vector<Point2i>& corners);
    int Detect_Billirad_Balls(vector<Point2i>& balls_center,  vector<int>& ball_color_ref);
    void Clear_prev_frame_info();

    // for test
    void Draw_Corners(Mat& img, vector<Point2i>& corners);
    void Draw_Balls(Mat& img, vector<Point2i>& balls_center,  vector<int>& ball_color_ref);
        

};

#endif