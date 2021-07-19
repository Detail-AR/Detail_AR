#include "Solution.hpp"
#include "basic.h"


/**
 당구대 사이즈 : W(1224), H(2448)
 당구공 반지름 : 33 = sqrt(( x-x1)^2+ (y-y1)^2)
 Mat 1224 2448
 Mat templete = Mat(B_H,B_W, CV_8UC3, Scalar(0,0,0));
 */

void init(Ball &Red, Ball &Red2, Ball &Yellow, Ball &White){
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


void updateBall(Ball &Red, Ball &Red2, Ball &Yellow, Ball &White){
    while(!isMoveFinish(Red, Red2, Yellow, White)){
        White.move();
        Yellow.move();
        Red.move();
        Red2.move();
    }
}

void foo()
{
    Ball Red = Ball(Point2d(100, 100), Point2d(0, 0), 2);
    Ball Red2 = Ball(Point2d(200, 190), Point2d(0, 0), 2);
    Ball Yellow = Ball(Point2d(1111, 300), Point2d(0, 0), 1);
    Ball White = Ball(Point2d(500, 300), Point2d(0, 0), 0);
    
    init(Red, Red2, Yellow, White);
    
    Mat templateA = Mat(1224,2448, CV_8UC3, Scalar(255,0,0));
    Red.paint(templateA);
    Red2.paint(templateA);
    Yellow.paint(templateA);
    White.paint(templateA);
    
    vector<Point2d> tempList;
    
    for(int dx=-100; dx<=100; dx++){
        for(int dy=-100; dy<=100; dy++){
            // 1사분면
            Ball redTemp = Ball(Point2d(Red.locate), Point2d(Red.speed), 2);
            Ball redTemp2 = Ball(Point2d(Red2.locate), Point2d(Red2.speed), 2);
            Ball yellowTemp = Ball(Point2d(Yellow.locate), Point2d(Yellow.speed), 1);
            Ball whiteTemp = Ball(Point2d(White.locate), Point2d(dx, dy), 0);
            init(redTemp, redTemp2, yellowTemp, whiteTemp);
            
            updateBall(redTemp, redTemp2, yellowTemp, whiteTemp);
            if(whiteTemp.isValidCollision()) tempList.push_back(Point2d(dx, dy));
            
//            redTemp.setDefault();
//            redTemp2.setDefault();
//            yellowTemp.setDefault();
//            whiteTemp.setDefault();
        }
    }
    
    vector<Point2d> validList;
    for(auto s : tempList){
        if(abs(s.x) + abs(s.y) <= 30){
            validList.push_back(Point2d(s.x, s.y));
            cout << s << '\n';
        }
    }

    White.setSpeed(Point2d(-17, -9));
    while(!isMoveFinish(Red, Red2, Yellow, White)){
        White.move();
        Red.move();
        Red2.move();
        Yellow.move();
        White.paint(templateA);
        Red.paint(templateA);
        Red2.paint(templateA);
        Yellow.paint(templateA);
    }

    imshow("templateA", templateA);
    
    waitKey();
    
}
