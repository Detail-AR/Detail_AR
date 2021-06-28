#include "basic.h"


double Vector_Degree(double x, double c_x, double y, double c_y){

    double dot_pro = ( x * c_x ) + ( y * c_y );
    double dDen = ( sqrt( pow( x, 2 ) + pow( y, 2 ) ) * sqrt( pow( c_x, 2 ) + pow( c_y, 2 ) ) ) +1e-8;
    double ratio = dot_pro / dDen;
    double Rad;

            
        if(ratio >= 1.0)  // nan 문제 발생.
            Rad=  acos( ratio - 0.0001 );
        else if(ratio <= -1.0)
            Rad=  acos( ratio + 0.0001 );
        else
            Rad=  acos( dot_pro / dDen );
    
    return Rad;
}


bool Get_Intersect_Point(float r1, float t1, float r2, float t2, Point2i& pt){
    
    float rho1= r1;  
	float theta1= t1; 

    float rho2= r2;  
	float theta2= t2; 

    double diff_theta = abs(theta1 - theta2) < CV_PI/2 ? abs(theta1 - theta2) : CV_PI- abs(theta1 - theta2);

    if(diff_theta < 0.2)
        return false;

    float sin1 = sin(theta1);
    float sin2 = sin(theta2);
    float a1, c1, a2, c2;

    if(  sin1 < 0.001 ){
        a1 = -cos(theta1)/(sin1+0.001);
        c1 = rho1/(sin1+0.001);
    }
    else{
        a1 = -cos(theta1)/(sin1);
        c1 = rho1/(sin1);
    }

    if(  sin2 < 0.001 ){
        a2 = -cos(theta2)/(sin2+0.001);
        c2 = rho2/(sin2+0.001);
    }
    else{
        a2 = -cos(theta2)/(sin2);
        c2 = rho2/(sin2);
    }
    

/*
   
    float a1 = -cos(theta1)/(sin(theta1)+1e-8);
    float c1 = rho1/(sin(theta1)+1e-8);

    float a2 = -cos(theta2)/(sin(theta2)+1e-8);
    float c2 = rho2/(sin(theta2)+1e-8);
*/
    
    float x = (c2-c1)/(a1-a2);
    float y = a1*x + c1;
    

    pt =  Point2i(x,y);
    return true;
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

void Find_one_blob_center(Mat& one_blob, Point2i& one_blob_center){
    vector<Point2i> centers;
    Mat points;
    Mat labels;
    findNonZero(one_blob, points);
    points.convertTo(points, CV_32F);


    kmeans(points, 1, labels, TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 3,
    KMEANS_PP_CENTERS, centers );

    one_blob_center.x = centers[0].x;
    one_blob_center.y = centers[0].y;
}


int Get_Area(Point2i X1, Point2i X2, Point2i X3){
    return X1.x*(X2.y-X3.y) + X2.x*(X3.y-X1.y) + X3.x*(X1.y-X2.y);
}


void Sort_Corners_Clockwise(vector<Point2i>& corners){
    int size = corners.size();

    if(size == 4){ // size == 4

        if(Get_Area(corners[0], corners[1], corners[2]) < 0)
            std::swap(corners[0], corners[1]);

        if(Get_Area(corners[0], corners[2], corners[3]) > 0)
            return;
        else{
            if(Get_Area(corners[0],corners[1],corners[3]) > 0)
                std::swap(corners[2], corners[3]);
            else    
                std::swap(corners[0], corners[1]);
        }
    }
}

double double_vector_dist_sum(vector<Point2d>&a, vector<Point2d>&b){
    int size = a.size();
    double dist_sum = 0;

    for(int i=0; i <size ; i++){
        dist_sum += sqrt((a[i].x - b[i].x)*(a[i].x - b[i].x) + (a[i].y - b[i].y)*(a[i].y - b[i].y));
    }
    return dist_sum;
}
void Clockwise_Permutation(vector<Point2d>& pts){
    int size = pts.size();
    Point2d temp = pts[size-1];

    for(int i=size-1; i>0 ; i--)
        pts[i]=pts[i-1];
    
    pts[0] = temp;
}

bool Point_Duplicate_check(int x, int y, vector<Point2i>& pts){
    int size = pts.size();
    bool flag = true;
    for(int i=0; i<size ; i++){    
        int dist = abs(x-pts[i].x) + abs(y-pts[i].y);
        if(dist < 20)   // 20 픽셀 이내이면 같은 교점으로 간주이다.
            flag = false;
    }
    return flag;
}

double Dist_of_Rotation(Mat& rvec1, Mat& rvec2)
{
    Mat R1, R2;
    Rodrigues(rvec1, R1);
    Rodrigues(rvec2, R2);

    Mat R = R1*R2.t();
    double diff = (trace(R)[0] - 1)/2;
    double Rad;

    if(diff >= 1.0)  // nan 문제 발생.
        Rad=  acos( diff - 0.0001 );
    else if(diff <= -1.0)
        Rad=  acos( diff+ 0.0001 );
    else
        Rad=  acos( diff );

    return Rad;

}

double Dist_of_Translation(Mat& tvec1, Mat& tvec2)
{
    double sum=0;
    for(int i=0; i<3 ; i++){
        double diff = (tvec1.ptr<double>(i)[0] -  tvec2.ptr<double>(i)[0]);
        sum += diff*diff;
    }
    return sqrt(sum);
}
