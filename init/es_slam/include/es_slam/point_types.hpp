#ifndef POINT_TYPES_HPP
#define POINT_TYPES_HPP

#include <vector>

struct LPoint2D {
    int sid;
    double x;
    double y;
    
    LPoint2D() : sid(0), x(0), y(0) {}
    LPoint2D(int s, double px, double py) : sid(s), x(px), y(py) {}
    
    void setData(int s, double px, double py) {
        sid = s;
        x = px;
        y = py;
    }
};

struct Scan2D {
    std::vector<LPoint2D> lps;
    
    void setLps(const std::vector<LPoint2D>& newLps) {
        lps = newLps;
    }
};

#endif // POINT_TYPES_HPP