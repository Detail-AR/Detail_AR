void Detect_Billiard(Mat& img, Mat& output);
void Detection_Main(void);
void Detect_ball_color(Mat& img, Mat* output_array);
void Detect_Billiard_Hole(Mat& input, Mat& output1, Mat& output2);
void Extract_Biggest_Blob_with_Center(Mat& input, Mat& output, Point2i& center);


void Detect_Billiard_Edge(Mat& Big_blob, Point2i& Big_blob_center, vector<Vec4f>& Candidate_lines);
void Calculation_Billiard_Corner(vector<Vec4f>& Candidate_lines, Point2i& Big_blob_center,vector<Point2i>& corners, Mat& temp);

void Match_ball_and_color(Mat* ball_colors, Mat& hole, vector<Mat>& ball_candidate, vector<vector<int>>& label_with_color);
void Find_ball_center(vector<Mat>& ball_candidate, vector<vector<int>>& label_with_color, vector<Point2i>& balls_center);

double Vector_Degree(double x, double c_x, double y, double c_y);
Point2i Get_Intersect_Point(float r1, float t1, float r2, float t2);
bool cmp(const Vec3f &p1, const Vec3f &p2);
bool cmp2(const pair<int, int>& a, const pair<int,int>& b);