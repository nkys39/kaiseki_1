#ifndef SCAN_POINT_RESAMPLER_HPP
#define SCAN_POINT_RESAMPLER_HPP

#include <vector>
#include <cmath>
#include "point_types.hpp"


class ScanPointResampler {
private:
    double dthreS;  // 点の距離間隔[m]
    double dthreL;  // 点の距離閾値[m]
    double dis;     // 累積距離

public:
    ScanPointResampler();
    void resamplePoints(Scan2D* scan);
    bool findInterpolatePoint(const LPoint2D& cp, const LPoint2D& pp, 
                            LPoint2D& np, bool& inserted);
    void setParameters(double distanceThreshold, double lengthThreshold);
};


#endif