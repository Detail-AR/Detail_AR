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

vector<Point2d> findPath(Ball Red, Ball Red2, Ball Yellow, Ball White, int targetColor){
    vector<Point2d> ret;
    if(targetColor == 1){
        while(Yellow.speed.x != 0.0){
//        White.move();
            if(Yellow.isCollisionMove() != -1){
                ret.push_back(Yellow.locate);
            }
            Red.move();
            Red2.move();
            White.move();

            if(Yellow.isValidCollision()){
                return ret;
            }
        }

    }else if(targetColor == 2){
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
    }


    return {Point2d(-1,-1)};
}

// 공을 움직여라
void updateBall(Ball &Red, Ball &Red2, Ball &Yellow, Ball &White, int targetColor){
    if(targetColor == 1){
        while(Yellow.speed.x != 0.0){
            White.move();
            Red.move();
            Red2.move();
            Yellow.move();
        }
    }else if(targetColor == 2){
        while(White.speed.x != 0.0){
            White.move();
            Red.move();
            Red2.move();
            Yellow.move();
        }
    }

}

// 힘이 약한걸 앞으로 오게 하자.
bool cmp10(const Point2d& a, const Point2d& b){
    if((abs(a.x) + abs(a.y)) == (abs(b.x) + abs(b.y))){
        return abs(a.x) < abs(b.x);
    }
    return (abs(a.x) + abs(a.y)) < (abs(b.x) + abs(b.y));
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

vector<Point2d> getSpeedList(Ball &Red, Ball &Red2, Ball &Yellow, Ball &White, int targetColor){
    vector<Point2d> ret;

    for(double dx=-80; dx<=80; dx+=0.5){
        for(double dy=-80; dy<=80; dy+=0.5){
            // 1사분면 ~ 4사분면 Screen 좌표계 (화이트가 0,0 가정)
            Ball redTemp = Ball(Point2d(Red.locate), Point2d(Red.speed), 2);
            Ball redTemp2 = Ball(Point2d(Red2.locate), Point2d(Red2.speed), 2);
            Ball yellowTemp = Ball(Point2d(Yellow.locate), Point2d(Yellow.speed), 1);
            Ball whiteTemp = Ball(Point2d(White.locate), Point2d(White.speed), 0);
            if(targetColor == 1) yellowTemp.setSpeed(Point2d(dx, dy));
            else if(targetColor == 2) whiteTemp.setSpeed(Point2d(dx,dy));
            makeAdj(redTemp, redTemp2, yellowTemp, whiteTemp);

            updateBall(redTemp, redTemp2, yellowTemp, whiteTemp, targetColor);
            if(targetColor == 1){
                if(yellowTemp.isValidCollision()) ret.push_back(Point2d(dx, dy));
            }else if(targetColor == 2){
                if(whiteTemp.isValidCollision()) ret.push_back(Point2d(dx, dy));
            }
        }
    }

    return ret;
}

vector<vector<Point2d>> getValidCollisionList(Ball &Red, Ball &Red2, Ball &Yellow, Ball &White,
                                       vector<Point2d>& tempSpeedList, Point2d past, int targetColor){
    vector<Point2d> validSpeedList;
    for(auto speeds : tempSpeedList){
        if(targetColor == 1){
            Yellow.setLocate(past);
            Yellow.setSpeed(speeds);
        }
        else if(targetColor == 2){
            White.setLocate(past);
            White.setSpeed(speeds);
        }
        auto paths = findPath(Red, Red2, Yellow, White, targetColor);
        if(paths[0].x == -1 && paths[0].y == -1) continue;
        if(paths.size() == 3){
            validSpeedList.push_back(speeds);
        }
    }
    sort(validSpeedList.begin(), validSpeedList.end(), cmp10);
    vector<vector<Point2d>> list;
    for(auto speeds : validSpeedList){
        if(targetColor == 1){
            Yellow.setLocate(past);
            Yellow.setSpeed(speeds);
        }else if(targetColor == 2){
            White.setLocate(past);
            White.setSpeed(speeds);
        }

        list.push_back(findPath(Red, Red2, Yellow, White, targetColor));
    }

    return list;
}

void showPaths( Ball &Red, Ball &Red2, Ball &Yellow, Ball &White,
                Mat &bTemplate, vector<vector<Point2d>> &list, Point2d past, int&pathIndex,
                int targetColor, vector<pair<int, int>> errorList) {
    if(list.size() != 0){
        for(Point2d point : list[pathIndex]){
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
            if(targetColor == 1) {
                Yellow.setLocate(Point2d(point.y, point.x));
                Yellow.paint(bTemplate);
            }else if(targetColor == 2){
                White.setLocate(Point2d(point.y, point.x));
                White.paint(bTemplate);
            }
            arrowedLine(bTemplate, Point2d(past.y, past.x), Point2d(point.y, point.x), Scalar(0, 255, 0), 15);
            past = point;
        }
    }
}

void BilliardSollution(Mat &bTemplate, vector<Point2i> balls_center, vector<int> ball_color_ref, int target_color, int btn_index)
{
    static vector<vector<Point2d>> white_path_list;
    static vector<vector<Point2d>> yellow_path_list;
    static vector<pair<int, int>> error_list;
    static int white_path_index = 0;
    static int yellow_path_index = 0;
    static Point2d white_past_loc;
    static Point2d yellow_past_loc;
    static Ball Red, Red2, Yellow, White;

    // 인식 버튼
    if(btn_index == 1) {
        bool redFlag = false;
        for(int i=0; i<4; i++){
            if(ball_color_ref[i] == 0){
                if(!redFlag){
                    redFlag = true;
                    Red = Ball(Point2d(balls_center[i].y, balls_center[i].x), Point2d(0, 0), 0);
                }else
                    Red2 = Ball(Point2d(balls_center[i].y, balls_center[i].x), Point2d(0, 0), 0);
            }else if(ball_color_ref[i] == 1)
                Yellow = Ball(Point2d(balls_center[i].y, balls_center[i].x), Point2d(0, 0), 1);
            else if(ball_color_ref[i] == 2)
                White = Ball(Point2d(balls_center[i].y, balls_center[i].x), Point2d(0, 0), 2);
        }

        white_past_loc = White.locate;
        yellow_past_loc = Yellow.locate;

        error_list = setError(Red, Red2, Yellow, White);
        makeAdj(Red, Red2, Yellow, White);

        vector<Point2d> tempYellowSpeedList = getSpeedList(Red, Red2, Yellow, White, 1);
        vector<Point2d> tempWhiteSpeedList = getSpeedList(Red, Red2, Yellow, White, 2);

        yellow_path_list = getValidCollisionList(Red, Red2, Yellow, White,
                                                 tempYellowSpeedList, yellow_past_loc, 1);
        white_path_list = getValidCollisionList(Red, Red2, Yellow, White,
                                                tempWhiteSpeedList, white_past_loc, 2);
    }
    // 색깔 변경 및 경로 변경
    else if ( btn_index == 2 || btn_index == 3 ) {
        if(target_color == 1) {
            if(btn_index == 3){
                yellow_path_index = (yellow_path_index + (yellow_path_list.size() / 5)) % yellow_path_list.size();
            }
            showPaths(Red, Red2, Yellow, White, bTemplate, yellow_path_list, yellow_past_loc, yellow_path_index, target_color, error_list);
        }
        else if(target_color == 2) {
            if(btn_index == 3){
                white_path_index = (white_path_index + (white_path_list.size() / 5)) % white_path_list.size();
            }
            showPaths(Red, Red2, Yellow, White, bTemplate, white_path_list, white_past_loc, white_path_index, target_color, error_list);
        }
    }
    // 취소 버튼
    else {
        white_path_index = 0;
        yellow_path_index = 0;
        for(auto list : white_path_list) list.clear();
        for(auto list : yellow_path_list) list.clear();
        error_list.clear();
        white_past_loc = Point2d(0, 0);
        yellow_past_loc = Point2d(0, 0);

    }

}
