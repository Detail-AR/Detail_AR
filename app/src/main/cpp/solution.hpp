#ifndef Solution_hpp
#define Solution_hpp

#include "basic.h"

class Ball
{
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

    Ball() {}
    Ball(Point2d l, Point2d s, int c)
    {
        locate = l;
        speed = s;
        color = c;
    }

    ~Ball() {}

    void move()
    {
        if (speed.x == 0.0 and speed.y == 0.0)
            return;
        locate.x += speed.x;
        locate.y += speed.y;

        // x direct
        if (locate.x <= 33)
        {
            locate.x = 33;
            speed.x = -speed.x;
        }
        else if (locate.x >= 2415)
        {
            locate.x = 2415;
            speed.x = -speed.x;
        }
        // y direct
        if (locate.y <= 33)
        {
            locate.y = 33;
            speed.y = -speed.y;
        }
        else if (locate.y >= 1191)
        {
            locate.y = 1191;
            speed.y = -speed.y;
        }

        // speed Down
        speed.x *= 0.96;
        speed.y *= 0.96;

        if (abs(speed.x) < 0.14 and abs(speed.y) < 0.14)
        {
            speed.x = speed.y = 0.0;
        }

        // Check Collision
        if (checkCollision(other1))
            c1 = true;
        if (checkCollision(other2))
            c2 = true;
        if (checkCollision(other3))
            c3 = true;
    }

    bool checkCollision(Ball *other)
    {
        double dx = this->locate.x - other->locate.x;
        double dy = this->locate.y - other->locate.y;

        // Collision2
        if ((dx * dx) + (dy * dy) <= (66 * 66))
        {
            Point2d otherCenter = other->locate;
            Point2d nowCenter = this->locate;

            /* other dir */
            Point2d *dir = new Point2d(otherCenter.x - nowCenter.x, otherCenter.y - nowCenter.y);
            double length = sqrt((dir->x * dir->x) + (dir->y * dir->y));
            if (length < 1)
                length = 1;
            Point2d *dirVector = new Point2d(dir->x / length, dir->y / length); // normailize vector

            Point2d moveVector(this->speed.x, this->speed.y);
            double dv = (dirVector->x * moveVector.x) + (dirVector->y * moveVector.y); // dot product for scalar
            Point2d *otherDirVector = new Point2d(dirVector->x * dv, dirVector->y * dv);
            other->speed.x += otherDirVector->x;
            other->speed.y += otherDirVector->y;

            /* now dir */
            Point2d *nowDirVector = new Point2d(moveVector.x - otherDirVector->x, moveVector.y - otherDirVector->y);
            this->speed.x += nowDirVector->x;
            this->speed.y += nowDirVector->y;

            delete dir;
            delete dirVector;
            delete otherDirVector;
            delete nowDirVector;
            return true;
        }

        return false;
    }

    void paint(Mat &templateA)
    {
        if (color == 0)
        {
            circle(templateA, Point(locate.x, locate.y), 33, Scalar(255, 255, 255), 2, 8, 0);
        }
        else if (color == 1)
        {
            circle(templateA, Point(locate.x, locate.y), 33, Scalar(0, 255, 255), 2, 8, 0);
        }
        else if (color == 2)
        {
            circle(templateA, Point(locate.x, locate.y), 33, Scalar(0, 0, 255), 2, 8, 0);
        }
    }

    void setCollisionDefault()
    {
        c1 = false;
        c2 = false;
        c3 = false;
    }

    void setDefault()
    {
        locate.x = 0;
        locate.y = 0;
        speed.x = 0;
        speed.y = 0;
        color = -1;
        c1 = false;
        c2 = false;
        c3 = false;
    }

    void setSpeed(Point2d a)
    {
        this->speed.x = a.x;
        this->speed.y = a.y;
    }

    void setLocate(Point2d a)
    {
        this->locate.x = a.x;
        this->locate.y = a.y;
    }

    bool isValidCollision()
    {
        if (c1 and c2 and !c3)
        {
            return true;
        }
        return false;
    }
};

#endif /* Solution_hpp */
