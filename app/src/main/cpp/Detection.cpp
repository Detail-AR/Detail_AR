#include "Detection.hpp"
#include "Detect_Table_Func.hpp"
#include "Detect_Ball_Func.hpp"
<<<<<<< HEAD

void Detection::Set_Image(Mat& input_img){
    img = input_img;
    cvtColor(img, img_hsv, COLOR_RGB2HSV); // 안드로이드 에서는 RGB2HSV
}

=======

void Detection::Set_Image(Mat& input_img, bool flag){
    img = input_img;

    if(flag)
        cvtColor(img, img_hsv, COLOR_BGR2HSV); // 안드로이드 에서는 RGB2HSV
    else
        cvtColor(img, img_hsv, COLOR_RGB2HSV); // 안드로이드 에서는 RGB2HSV
}

>>>>>>> f8462fda59353253e572f8ba133eb66e390ce58b
int Detection::Detect_Billiard_Corners(vector<Point2i>& corners){

    Detect_Billiard(img_hsv, blue_and_morph);

    Extract_Biggest_Blob_with_Center(blue_and_morph, Big_blob, Big_blob_center);

    Detect_Billiard_Hole(Big_blob, hole, Big_blob_without_hole);

    Detect_Billiard_Edge(Big_blob_without_hole, Big_blob_center, Candidate_lines);

    Calculation_Billiard_Corner(Candidate_lines, Big_blob_center, corners);
<<<<<<< HEAD


=======


>>>>>>> f8462fda59353253e572f8ba133eb66e390ce58b
    int corner_size = corners.size();
    if(corner_size <= 0 && corner_size >=5)
        return 0;
    else  
        return 1;
}


int Detection::Detect_Billirad_Balls(vector<Point2i>& balls_center,  vector<int>& ball_color_ref){
    
    Detect_ball_color(img_hsv, ball_colors);
    Match_ball_and_color(ball_colors, hole, ball_candidate, label_with_color);
    // R R Y W 조합만 가능한것도 고려해서 거르자

    Find_ball_center(ball_candidate, label_with_color, ball_colors,balls_center, ball_color_ref);

    int ball_num = balls_center.size();
    if(ball_num <= 0)
        return 0;
    else
        return 1;
}



void Detection::Clear_prev_frame_info(){
    corners.clear();
    ball_candidate.clear();
    label_with_color.clear();
    balls_center.clear();
    ball_color_ref.clear();
}

void Detection::Draw_Corners(Mat& img, vector<Point2i>& corners){
    int size = corners.size();
    for(int j=0; j< size ;j++){
        circle(img, Point(corners[j].x, corners[j].y), 50, Scalar(0, j*80, 0), 2, 8, 0);
    }

}


void Detection::Draw_Balls(Mat& img, vector<Point2i>& balls_center,  vector<int>& ball_color_ref){
    int size2 = balls_center.size();
    for(int j=0; j<size2 ; j++){
        int x = balls_center[j].x;
        int y = balls_center[j].y;
        circle(img, Point(x, y), 4, Scalar(0, 255, 0), 2);
        int color = ball_color_ref[j];

        if(color==0){
            putText(img, "RED", Point(x,y), 1,1.5, Scalar(0,255,0), 2,8);
        }
        else if(color==1){
            putText(img, "Yellow", Point(x,y), 1,1.5, Scalar(0,255,0), 2,8);
        }
        else
        {
            putText(img, "White", Point(x,y), 1,1.5, Scalar(0,255,0), 2,8);
        }
    }
}