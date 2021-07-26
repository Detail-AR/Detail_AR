#ifndef Solution_hpp
#define Solution_hpp

#include "basic.h"

class Ball{
public:
    Point2d locate;
    Point2d speed;
    int color;
    Ball *other1;
    Ball *other2;
    Ball *other3;
    bool c1 = false;
    bool c2 = false;
    bool c3 = false;

    Ball(){}
    Ball(Point2d l, Point2d s, int c){
        locate = l;
        speed = s;
        color = c;
    }

    ~Ball(){}

    void move(){
        if(speed.x == 0.0 && speed.y == 0.0) return;
        if(isnan(speed.x) || isnan(speed.y)){
            speed.x = speed.y = 0;
        }
        locate.x += speed.x;
        locate.y += speed.y;

        // x direct
        if(locate.x <= 33){
            locate.x = 33;
            speed.x = -speed.x * 0.987;
        }else if(locate.x >= 2415){
            locate.x = 2415;
            speed.x = -speed.x * 0.987;
        }
        // y direct
        if(locate.y <= 33){
            locate.y = 33;
            speed.y = -speed.y * 0.987;
        }else if(locate.y >= 1191){
            locate.y = 1191;
            speed.y = -speed.y * 0.987;
        }

        // speed Down
        speed.x *= 0.987;
        speed.y *= 0.987;

        if(abs(speed.x)<0.14 && abs(speed.y)<0.14) {
            speed.x = speed.y = 0.0;
        }


        // Check Collision
        if(checkCollision(other1)) c1 = true;
        if(checkCollision(other2)) c2 = true;
        if(checkCollision(other3)) c3 = true;
    }

    bool checkCollision(Ball *other){
        double dx = this->locate.x - other->locate.x;
        double dy = this->locate.y - other->locate.y;
        double dLen = (dx * dx) + (dy * dy);

        // Collision2
        if(dLen <= (66*66)){
            while(66*66 - dLen >= 0.01){
                if(this->speed.x == 0 && this->speed.y == 0) break;
                this->locate.x -= this->speed.x * 0.1;
                this->locate.y -= this->speed.y * 0.1;

                dx = this->locate.x - other->locate.x;
                dy = this->locate.y - other->locate.y;
                dLen = (dx * dx) + (dy * dy);
            }
            Point2d otherCenter = other->locate;
            Point2d nowCenter = this->locate;


            /* other dir */
            Point2d* dir = new Point2d(otherCenter.x - nowCenter.x, otherCenter.y - nowCenter.y);
            double length = sqrt((dir->x * dir->x) + (dir->y * dir->y));
            if(length == 0) length = 1;
            Point2d* dirVector = new Point2d(dir->x / length, dir->y / length); // normailize vector

            Point2d moveVector(this->speed.x, this->speed.y);
            double dv = (dirVector->x * moveVector.x) + (dirVector->y * moveVector.y); // dot product for scalar
            Point2d* otherDirVector = new Point2d(dirVector->x * dv, dirVector->y * dv);
//            other->speed.x = otherDirVector->x;
//            other->speed.y = otherDirVector->y;

            /* now dir */
            Point2d* nowDirVector = new Point2d(moveVector.x - otherDirVector->x, moveVector.y - otherDirVector->y);
            this->speed.x = nowDirVector->x;
            this->speed.y = nowDirVector->y;

            if(abs(speed.x) < 0.14 && abs(speed.y) < 0.14){
                speed.x = 0;
                speed.y = 0;
            }

//            while(dLen <= 66*67 and (this->speed.x != 0 and this->speed.y != 0)){
//                this->locate.x += this->speed.x;
//                this->locate.y += this->speed.y;
//
////                cout << locate << " " << speed << '\n';
//                dx = this->locate.x - other->locate.x;
//                dy = this->locate.y - other->locate.y;
//                dLen = (dx * dx) + (dy * dy);
//            }

            delete dir;
            delete dirVector;
            delete otherDirVector;
            delete nowDirVector;
            return true;
        }

        return false;
    }

    void paint(Mat &templateA){
        if(color == 0){
            circle(templateA, Point(locate.x, locate.y), 33, Scalar(255, 255, 255), 2, 8, 0);
        }else if(color == 1){
            circle(templateA, Point(locate.x, locate.y), 33, Scalar(0, 255, 255), 2, 8, 0);
        }else if(color == 2){
            circle(templateA, Point(locate.x, locate.y), 33, Scalar(0, 0, 255), 2, 8, 0);
        }
    }

    void setDefault(){
        locate.x = 0; locate.y = 0;
        speed.x = 0; speed.y = 0;
        color = -1;
        c1 = false; c2 = false; c3 = false;
    }

    void setSpeed(Point2d s){
        this->speed.x = s.x;
        this->speed.y = s.y;
    }

    void setLocate(Point2d l){
        this->locate.x = l.x;
        this->locate.y = l.y;
    }

    bool isValidCollision(){
        if(c1 && c2 && !c3){
            return true;
        }
        return false;
    }

    int isCollisionMove(){
        int collision = -1; // -1 : no, 0 : red1, 1 : red2, 2 : Yellow 3: wall
        if(speed.x == 0.0 && speed.y == 0.0) return -1;;
        if(isnan(speed.x) || isnan(speed.y)){
            speed.x = speed.y = 0;
            return -1;
        }
        locate.x += speed.x;
        locate.y += speed.y;

        // x direct
        if(locate.x <= 33){
            locate.x = 33;
            speed.x = -speed.x * 0.987;
            collision = 3;
        }else if(locate.x >= 2415){
            locate.x = 2415;
            speed.x = -speed.x * 0.987;
            collision = 3;
        }
        // y direct
        if(locate.y <= 33){
            locate.y = 33;
            speed.y = -speed.y * 0.987;
            collision = 3;
        }else if(locate.y >= 1191){
            locate.y = 1191;
            speed.y = -speed.y * 0.987;
            collision = 3;
        }

        // speed Down
        speed.x *= 0.987;
        speed.y *= 0.987;

        if(abs(speed.x)<0.14 && abs(speed.y)<0.14) {
            speed.x = speed.y = 0.0;
        }


        // Check Collision
        if(checkCollision(other1)) { c1 = true; collision = 0; }
        if(checkCollision(other2)) { c2 = true; collision = 1; }
        if(checkCollision(other3)) { c3 = true; collision = 2; }

        return collision;
    }

};

#endif /* Solution_hpp */
