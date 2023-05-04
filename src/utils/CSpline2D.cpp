#include "utils/CSpline2D.h"
#include <iostream>
#include <fstream>

bool CSpline2D::init(std::vector<dwVector2f> waypoints) {

    // number of x waypoints
    nx = waypoints.size();

    x = VectorXd(nx);
    y = VectorXd(nx);
    for(int i=0; i<nx; i++) {
        x(i) = waypoints[i].x;
        y(i) = waypoints[i].y;
    }

    s = calc_s();
    sx.init(s, x);
    sy.init(s, y);

    s_start = s(0);
    s_end   = s(s.size()-1);

    cur_s, cur_d = 0;

    return true;
}

bool CSpline2D::init(double* xs, double *ys, int size) {

    // number of x waypoints
    nx = size;

    x = VectorXd(nx);
    y = VectorXd(nx);
    for(int i=0; i<nx; i++) {
        x(i) = xs[i];
        y(i) = ys[i];
    }

    s = calc_s();
    sx.init(s, x);
    sy.init(s, y);

    s_start = s(0);
    s_end   = s(s.size()-1);

    cur_s, cur_d = 0;

    return true;
}

bool CSpline2D::init(std::string path_file) {

    std::vector<dwVector2f> waypoints;


    std::ifstream fin(path_file);
    if(fin) {
        std::string s;
        while(std::getline(fin,s)) {
            std::cout<<s<<"\n";
            float x,y,z;
            fin >>s>>x
                >>s>>y
                >>s>>z
                >>s;
            std::cout<<x<<" "<<y<<" "<<z<<"\n";
            
            waypoints.push_back(dwVector2f{x, y});
        }
        fin.close();
    }
    waypoints.push_back(waypoints[0]);

    return waypoints.size() > 0 && init(waypoints);
}

VectorXd CSpline2D::calc_s() {

    // build diff vector
    VectorXd dx = VectorXd(nx-1);
    for(int i=0; i<nx-1; i++) 
        dx(i) = x(i+1) - x(i);

    // build diff vector
    VectorXd dy = VectorXd(nx-1);
    for(int i=0; i<nx-1; i++) 
        dy(i) = y(i+1) - y(i);

    VectorXd ds = VectorXd(nx-1);
    for(int i=0; i<nx-1; i++)
        ds(i) = sqrt( dx(i)*dx(i) + dy(i)*dy(i) );
    
    // comulative sum
    VectorXd s = VectorXd(nx);
    s(0) = 0.0;
    for(int i=0; i<nx-1; i++)
        s(i+1) = s(i) + ds(i);

    return s;
}


dwVector2f CSpline2D::calc_position(double s) {
    
    dwVector2f p;
    p.x = sx.calc(s);
    p.y = sy.calc(s);
    return p;
}

double CSpline2D::calc_curvature(double s) {
    double dx  = sx.calcd(s);
    double ddx = sx.calcdd(s);
    double dy  = sy.calcd(s);
    double ddy = sy.calcdd(s);
    double k   = (ddy * dx - ddx * dy) / (dx*dx + dy*dy);
    return k;
}

double CSpline2D::calc_yaw(double s) {
    double dx  = sx.calcd(s);
    double dy  = sy.calcd(s);
    double yaw = atan2(dy, dx);
    return yaw;
}

void CSpline2D::update_current_s(double x, double y){
    double interval = 0.05;
    dwVector2f p0, p1;
    double d0, d1, delta, i, s1;
    int counter;
    double s_temp, d_temp;

    s_temp = cur_s;
    d_temp = cur_d;

    //reset for loop spline
    if(s_temp >= s[s.size()-1] - interval)
    {
        s_temp = 0;
    }

    p0 = calc_position(s_temp);
    d0 = pow((x - p0.x), 2) + pow((y - p0.y), 2);

    if(d0 > 2.0)
    {
        delta = (s[s.size()-1] - s[0]) / 499;
        i = s[0];
        for(counter = 1; i<500; counter++)
        {
            p1 = calc_position(i);
            d1 = pow((x - p1.x), 2) + pow((y - p1.y), 2);

            if(d1 < d0)
            {
                d0 = d1;
                s_temp = i;
            }

            i += (delta*counter);
        }
    }    
    else
    {
        s1 = s_temp;
        d1 = d_temp;

        while(s_temp < s[s.size()-1])
        {
            s1 = s1 + interval;
            p1 = calc_position(s1);
            
            d1 = pow((x - p1.x), 2) + pow((y - p1.y), 2);

            if(d1 <= d0)
            {
                s_temp = s1;
                d0 = d1;
            }
            else
            {
                break;
            }
            d_temp = sqrt(d1);
        }

    }

    cur_s = s_temp;
    cur_d = d_temp;

}
