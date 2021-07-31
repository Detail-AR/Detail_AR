#include "Solution.hpp"
#include "basic.h"


/**
 당구대 사이즈 : W(1224), H(2448)
 당구공 반지름 : 33 = sqrt(( x-x1)^2+ (y-y1)^2)
 Mat 1224 2448
 Mat templete = Mat(B_H,B_W, CV_8UC3, Scalar(0,0,0));
 */

void makeAdj(Ball &Red, Ball &Red2, Ball &Yellow, Ball &White){
    Red.other1 = &Red2; Red.other2 = &Yellow; Red.other3 = &White;
    Red2.other1 = &Red; Red2.other2 = &Yellow; Red2.other3 = &White;
    Yellow.other1 = &Red; Yellow.other2 = &Red2; Yellow.other3 = &White;
    White.other1 = &Red; White.other2 = &Red2; White.other3 = &Yellow;
}

bool isMoveFinish(Ball &Red, Ball &Red2, Ball &Yellow, Ball &White){
    if(White.speed.x != 0.0 || Red.speed.x != 0.0 || Red2.speed.x != 0.0 || Yellow.speed.x != 0.0) return false;
    if(White.speed.y != 0.0 || Red.speed.y != 0.0 || Red2.speed.y != 0.0 || Yellow.speed.y != 0.0) return false;
    return true;
}

vector<Point2d> findPath(Ball Red, Ball Red2, Ball Yellow, Ball White){
    vector<Point2d> ret;
    while(White.speed.x != 0.0){
//        White.move();
        if(White.isCollisionMove() != -1){
            ret.push_back(White.locate);
        }
        Red.move();
        Red2.move();
        Yellow.move();

        if(White.isValidCollision()){
            return ret;
        }
    }

    return {Point2d(-1,-1)};
}

// 공을 움직여라
void updateBall(Ball &Red, Ball &Red2, Ball &Yellow, Ball &White){
    while(White.speed.x != 0.0){
        White.move();
        Red.move();
        Red2.move();
        Yellow.move();
    }
}

// 힘이 약한걸 앞으로 오게 하자.
bool cmp(Point2d a, Point2d b){
    if((abs(a.x) + abs(a.y)) == (abs(b.x) + abs(b.y))){
        return abs(a.x) < abs(b.x);
    }
    return (abs(a.x) + abs(a.y)) < (abs(b.x) + abs(b.y));
}

// 중복된 충돌 제거
vector<Point2d> toUniquePath(vector<Point2d> paths){
    vector<Point2d> ret;

    Point2d past = paths[0];
    ret.push_back(past);
    bool flag = false;
    for(int i=1; i<paths.size(); i++){
        Point2d next = paths[i];
        if(abs(next.x - past.x) + abs(next.y - past.y) <= 0){
            continue;
        }
        ret.push_back(next);
        past = next;
    }

    return ret;
}

vector<pair<int, int>> setError(Ball &Red, Ball &Red2, Ball &Yellow, Ball &White){
    vector<Ball> Balls(4);
    Balls[0] = Red; Balls[1] = Red2; Balls[2] = Yellow; Balls[3] = White;

    vector<pair<int, int>> list;
    for(Ball ball : Balls){
        int xError = 0, yError = 0;
        if(ball.locate.x < 33) xError = 33 - ball.locate.x;
        if(ball.locate.x > 2415) xError = 2415 - ball.locate.x;
        if(ball.locate.y < 33) yError = 33 - ball.locate.y;
        if(ball.locate.y > 1191) yError = 1191 - ball.locate.y;
        list.push_back({xError, yError});
    }
    if(list[0].first != 0 || list[0].second != 0)
        Red.setLocate(Point2d(Red.locate.x + list[0].first, Red.locate.y + list[0].second));
    if(list[1].first != 0 || list[1].second != 0)
        Red2.setLocate(Point2d(Red2.locate.x + list[1].first, Red2.locate.y + list[1].second));
    if(list[2].first != 0 || list[2].second != 0)
        Yellow.setLocate(Point2d(Yellow.locate.x + list[2].first, Yellow.locate.y + list[2].second));
    if(list[3].first != 0 || list[3].second != 0)
        White.setLocate(Point2d(White.locate.x + list[3].first, White.locate.y + list[3].second));
    return list;
}

bool isRedCollision(Point2d point, Ball &Red){
    int sum = 0;
    sum += abs(point.x - Red.locate.x);
    sum += abs(point.y - Red.locate.y);

    if(sum > 95) return false;
    return true;
}

void BilliardSollution(Mat& bTemplate, vector<Point2i> balls_center, vector<int> ball_color_ref)
{
    Ball Red, Red2, Yellow, White;
    bool redFlag = false;
    for(int i=0; i<4; i++){
        if(ball_color_ref[i] == 0){
            if(!redFlag){
                redFlag = true;
                Red = Ball(Point2d(balls_center[i].y, balls_center[i].x), Point2d(0, 0), 0);
            }else{
                Red2 = Ball(Point2d(balls_center[i].y, balls_center[i].x), Point2d(0, 0), 0);
            }
        }else if(ball_color_ref[i] == 1){
            Yellow = Ball(Point2d(balls_center[i].y, balls_center[i].x), Point2d(0, 0), 1);
        }else if(ball_color_ref[i] == 2){
            White = Ball(Point2d(balls_center[i].y, balls_center[i].x), Point2d(0, 0), 2);
        }
    }
    Point2d past = White.locate;

    vector<pair<int, int>> errorList = setError(Red, Red2, Yellow, White);
    makeAdj(Red, Red2, Yellow, White);

    vector<Point2d> tempSpeedList;

    for(double dx=-80; dx<=80; dx+=0.5){
        for(double dy=-80; dy<=80; dy+=0.5){
            // 1사분면 ~ 4사분면 Screen 좌표계 (화이트가 0,0 가정)
            Ball redTemp = Ball(Point2d(Red.locate), Point2d(Red.speed), 2);
            Ball redTemp2 = Ball(Point2d(Red2.locate), Point2d(Red2.speed), 2);
            Ball yellowTemp = Ball(Point2d(Yellow.locate), Point2d(Yellow.speed), 1);
            Ball whiteTemp = Ball(Point2d(White.locate), Point2d(dx, dy), 0);
            makeAdj(redTemp, redTemp2, yellowTemp, whiteTemp);

            updateBall(redTemp, redTemp2, yellowTemp, whiteTemp);
            if(whiteTemp.isValidCollision()) tempSpeedList.push_back(Point2d(dx, dy));
        }
    }

    vector<Point2d> validSpeedList;
    for(auto speeds : tempSpeedList){
        White.setLocate(past);
        White.setSpeed(speeds);
        auto paths = findPath(Red, Red2, Yellow, White);
        if(paths[0].x == -1 && paths[0].y == -1) continue;
        if(paths.size() == 3){
            validSpeedList.push_back(speeds);
        }
    }
    sort(validSpeedList.begin(), validSpeedList.end(), cmp);
    vector<vector<Point2d>> list;
    for(auto speeds : validSpeedList){
        White.setLocate(past);
        White.setSpeed(speeds);
        list.push_back(findPath(Red, Red2, Yellow, White));
//        list[i] = toUniquePath(list[i]);
    }
    Ball Green = Ball(Point2d(437, 1486), Point2d(0, 0), 1);
    Green.paint(bTemplate);
    if(list.size() != 0){
        for(Point2d point : list[0]){
            if(isRedCollision(point, Red)){ // 제 1적구
                if(errorList[0].first != 0 || errorList[0].second != 0){
                    point.x -= errorList[0].first;
                    point.y -= errorList[0].second;
                }
            }
            else if(isRedCollision(point, Red2)){ // 제 2적구
                if(errorList[1].first != 0 || errorList[1].second != 0){
                    point.x -= errorList[1].first;
                    point.y -= errorList[1].second;
                }
            }else{ // 벽
                if(errorList[0].first != 0 || errorList[0].second != 0 ||
                   errorList[1].first != 0 || errorList[1].second != 0){
                    point.x = point.x - errorList[0].first - errorList[1].first;
                    point.y = point.y - errorList[0].second - errorList[1].second;
                }

            }
            White.setLocate(Point2d(point.y, point.x));
            White.paint(bTemplate);
            arrowedLine(bTemplate, Point2d(past.y, past.x), Point2d(point.y, point.x), Scalar(0, 255, 0), 3);
            past = point;
        }
    }


//    imshow("templateA", templateA);

//    waitKey();

}