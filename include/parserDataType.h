#ifndef PARSER_DATATYPE_H	
#define PARSER_DATATYPE_H

#include "header.h"

namespace parser
{

template <typename T>
class Point2D
{
public:
    T x;
    T y;
    Point2D() { 
        x = 0;
        y = 0;
    }
    Point2D(T x, T y): x(x), y(y) { }
    bool empty()
    {
        return ((x == 0) && (y == 0));
    }
    void set(T x_new, T y_new)
    {
        this->x = x_new;
        this->y = y_new;
    }
    void clear()
    {
        x = 0;
        y = 0;
    }
    void reset()
    {
        x = 0;
        y = 0;
    }
    void print()
    {
        cout << x << " " << y << endl;
    }
    bool operator< (const Point2D<T> p) const
    {
        if(x < p.x)
            return true;
        else if(x > p.x)
            return false;
        else
            return (y < p.y);
    }
    bool operator> (const Point2D<T> p) const
    {
        if(x > p.x)
            return true;
        else if(x < p.x)
            return false;
        else
            return (y > p.y);
    }

};

template<typename T>
using Size2D = Point2D<T>;

template <typename T>
class Point3D
{
public:
    T x;
    T y;
    int z;
    Point3D() { }
    Point3D(T x, T y, int z): x(x), y(y), z(z) { }
    void set(T x_new, T y_new, int z_new)
    {
        this->x = x_new;
        this->y = y_new;
        this->z = z_new;
    }
    void print()
    {
        cout << x << " " << y << " " << z << endl;
    }

    bool operator< (const Point3D<T> p) const
    {
        if(Point2D<T>(x, y) < Point2D<T>(p.x, p.y))
            return true;
        else if(Point2D<T>(x, y) > Point2D<T>(p.x, p.y))
            return false;
        else 
        {
            return (z < p.z);
        }
    }

    bool operator> (const Point3D<T> p) const
    {
        if(Point2D<T>(x, y) > Point2D<T>(p.x, p.y))
            return true;
        else if(Point2D<T>(x, y) < Point2D<T>(p.x, p.y))
            return false;
        else 
        {
            return (z > p.z);
        }
    }

};

template <typename T>
class Rect2D
{
public: 
    Point2D<T> lowerLeft;
    Point2D<T> upperRight;
    Rect2D() { }
    Rect2D(Point2D<T> lowerLeft, Point2D<T> upperRight): lowerLeft(lowerLeft), upperRight(upperRight) { }
    Rect2D(T llx, T lly, T urx, T ury): lowerLeft(llx, lly), upperRight(urx, ury) { }
    bool empty()
    {
        return (lowerLeft.empty() && upperRight.empty());
    }
    void set(T llx, T lly, T urx, T ury)
    {   
        lowerLeft.set(llx, lly);
        upperRight.set(urx, ury);
    }
    void print()
    {
        cout << "rect: " ;
        lowerLeft.print();
        upperRight.print();
    }
    bool cover(Point2D<T> p)
    {
        return (lowerLeft.x <= p.x && lowerLeft.y <= p.y && upperRight.x >= p.x && upperRight.y >= p.y);
    }
};

template <typename T>
class Rect2DLayer : public Rect2D<T>
{
public: 
    string layer;
    Rect2DLayer(): Rect2D<T>() {}
    void set(string layer, T llx, T lly, T urx, T ury)
    {
        this->layer = layer;
        this->lowerLeft.set(llx, lly);
        this->upperRight.set(urx, ury);
    }
};



template <typename T>
class Rect3D
{
public: 
    Point3D<T> lowerLeft;
    Point3D<T> upperRight;
    Rect3D() { }
    Rect3D(T llx, T lly, int llz, T urx, T ury, int urz): lowerLeft(llx, lly, llz), upperRight(urx, ury, urz) { }
    void set(T llx, T lly, int llz, T urx, T ury, int urz)
    {   
        lowerLeft.set(llx, lly, llz);
        upperRight.set(urx, ury, urz);
    }
    void print()
    {
        lowerLeft.print();
        upperRight.print();
    }
};

template <typename T>
class Range
{
public:
    T start;
    T end;
    Range() { }
    Range(T s, T e) {
        start = s;
        end = e;
    }
};

}

template <typename T>
std::ostream& operator<< (std::ostream& os, const parser::Point2D<T> &p) {
    std::cout << " ( " << p.x << ", " << p.y << ") ";
    return os;
}

template <typename T>
std::ostream& operator<< (std::ostream& os, const parser::Rect2D<T> &r) {
    std::cout << "Rect: ";
    std::cout << r.lowerLeft << r.upperRight;
    return os;
}


#endif
