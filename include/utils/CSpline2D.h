#ifndef CSPLINE2D_H_
#define CSPLINE2D_H_

#include <string>
#include <vector>
#include "CSpline.h"

#undef Success  // stupid X11
#include "Eigen/Dense"
using namespace Eigen;

struct dwVector2f {
    float x,y;
}; 

/**
    Cubic Spline2D class
*/
class CSpline2D {

public:

    CSpline2D() {};
    ~CSpline2D() {};
    // waypoints
    VectorXd x, y;
    int nx;

    //VectorXd s, d, cur_s, cur_d;
    //CSpline sx, sy;
    //double s_start, s_end;

    VectorXd s;
    double cur_s, cur_d;
    
    CSpline sx, sy;
    double s_start, s_end;

    //CSpline2D();
    //virtual ~CSpline2D();
 
    /**
        init
        @param waypoints x, y
    */
    bool init(std::vector<dwVector2f> waypoints);
    bool init(double* xs, double *ys, int size);
    bool init(std::string path);

    dwVector2f calc_position(double s);
    double calc_curvature(double s);
    double calc_yaw(double s);
    void update_current_s(double x, double y);

private:
    /**
        calc s
    */
    VectorXd calc_s();
};

#endif /* CSPLINE2D_H_ */
