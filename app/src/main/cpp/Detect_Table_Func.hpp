#ifndef Detect_Table_Func_h
#define Detect_Table_Func_h

void Detect_Billiard(Mat& img_hsv, Mat& output);
void Detect_Billiard_Hole(Mat& input, Mat& output1, Mat& output2);
void Extract_Biggest_Blob_with_Center(Mat& input, Mat& output, Point2i& center);
void Detect_Billiard_Edge(Mat& Big_blob, Point2i& Big_blob_center, vector<Vec4f>& Candidate_lines);
void Calculation_Billiard_Corner(vector<Vec4f>& Candidate_lines, Point2i& Big_blob_center,vector<Point2i>& corners);

#endif