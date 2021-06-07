#ifndef add_ons_h
#define add_ons_h



//Detection
void Find_one_blob_center(Mat& one_blob, Point2i& one_blob_center);
double Vector_Degree(double x, double c_x, double y, double c_y);
Point2i Get_Intersect_Point(float r1, float t1, float r2, float t2);
bool cmp(const Vec3f &p1, const Vec3f &p2);
bool cmp2(const pair<int, int>& a, const pair<int,int>& b);
<<<<<<< HEAD
=======
bool Point_Duplicate_check(int x, int y, vector<Point2i>& pts);
>>>>>>> f8462fda59353253e572f8ba133eb66e390ce58b


//Geo_proc
void Sort_Corners_Clockwise(vector<Point2i>& corners);
int Get_Area(Point2i X, Point2i Y, Point2i Z);
float float_vector_dist_sum(vector<Point2f>&a, vector<Point2f>&b);
void Clockwise_Permutation(vector<Point2f>& pts);


#endif