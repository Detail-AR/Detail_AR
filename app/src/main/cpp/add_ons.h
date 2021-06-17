#ifndef add_ons_h
#define add_ons_h



//Detection
void Find_one_blob_center(Mat& one_blob, Point2i& one_blob_center);
double Vector_Degree(double x, double c_x, double y, double c_y);
bool Get_Intersect_Point(float r1, float t1, float r2, float t2, Point2i& pt);
bool cmp(const Vec3f &p1, const Vec3f &p2);
bool cmp2(const pair<int, int>& a, const pair<int,int>& b);
bool Point_Duplicate_check(int x, int y, vector<Point2i>& pts);


//Geo_proc
void Sort_Corners_Clockwise(vector<Point2i>& corners);
int Get_Area(Point2i X, Point2i Y, Point2i Z);
double double_vector_dist_sum(vector<Point2d>&a, vector<Point2d>&b);
void Clockwise_Permutation(vector<Point2d>& pts);
double Dist_of_Rotation(Mat& rvec1, Mat& rvec2);
double Dist_of_Translation(Mat& tvec1, Mat& tvec2);


#endif