#ifndef WRITEFILE_H
#define WRITEFILE_H

namespace sproute_db
{

class Routed3DPoint
{
public:
    int x, y, z;
    bool horizontal;
    Routed3DPoint() { 
        x = 0;
        y = 0;
        z = 0;
        horizontal = false;
    }
    Routed3DPoint(int x, int y, int z, bool h): x(x), y(y), z(z), horizontal(h) { }

    bool operator< (const Routed3DPoint p) const
    {   
        if(z < p.z) {
            return true;
        }
        else if(z > p.z)
            return false;
        else {  //z == p.z
            if(horizontal) {
                if(y < p.y)
                    return true;
                else if(y > p.y)
                    return false;
                else {
                    if(x < p.x)
                        return true;
                    else 
                        return false;
                }
            }
            else { //vertical
                if(x < p.x)
                    return true;
                else if(x > p.x)
                    return false;
                else {
                    if(y < p.y)
                        return true;
                    else 
                        return false;
                }
            }
        }
    }
};

}


#endif