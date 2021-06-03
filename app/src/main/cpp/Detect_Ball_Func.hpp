#ifndef Detect_Ball_Func_h
#define Detect_Ball_Func_h


void Detect_ball_color(Mat& img_hsv, Mat* output_array);
void Match_ball_and_color(Mat* ball_colors, Mat& hole, vector<Mat>& ball_candidate, vector<vector<Vec3i>>& label_with_color);
void Find_ball_center(vector<Mat>& ball_candidate, vector<vector<Vec3i>>& label_with_color, Mat* ball_colors,
vector<Point2i>& balls_center, vector<int>& ball_color_ref);

#endif