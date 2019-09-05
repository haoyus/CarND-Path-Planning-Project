#ifndef COMMON_H
#define COMMON_H

#include "helpers.h"

const double WP_STRIDE = 30.0;//meters
const unsigned int ADD_WP_NUM = 3;//additional wp number
const double MAX_SPD_MPH = 49.5;
const double MAX_ACC = 0.224;
const double FOLLOW_DIST = 30.0;

struct sWayPoints{
    vector<double> x_vec;
    vector<double> y_vec;
    vector<double> s_vec;
    vector<double> d_vec;
};

enum eLaneLabel{
    LL_UNKNOWN = -1,//lane label global coordinate
    LL_LEFT = 0,
    LL_CENT = 1,
    LL_RIGH = 2
};

// governs PLC state in FSM
struct sLaneStatus{
    eLaneLabel label;//borrow eLaneLabel concept to indicate lane label relative to Ego coord
    bool isExist;
    bool isOpen;
    double rvSpdMph;
    sLaneStatus(eLaneLabel lbl) : label(lbl), isExist(false), isOpen(false), rvSpdMph(0.0) {}
};


double getSpd(double vx,double vy)
{
  return sqrt(vx*vx+vy*vy);
}

double mph2mps(double spd)
{
  return spd*0.447;
}

double mps2mph(double spd)
{
  return spd/0.447;
}

void setWpFrenet(sWayPoints& wps, double s, double d, int index)
{
//
}

void lcs2vcs(vector<double>& x_vec, vector<double>& y_vec, double veh_x, double veh_y, double veh_yaw_rad)
{
    for(int i=0;i<x_vec.size();++i){
        double tmpx = x_vec[i], tmpy = y_vec[i];
        tmpx -= veh_x;
        tmpy -= veh_y;
        x_vec[i] = tmpx*cos(veh_yaw_rad)+tmpy*sin(veh_yaw_rad);
        y_vec[i] = -1.0*tmpx*sin(veh_yaw_rad)+tmpy*cos(veh_yaw_rad);
    }
}

void vcs2lcs(vector<double>& x_vec, vector<double>& y_vec, double veh_x, double veh_y, double veh_yaw_rad)
{
    for(int i=0;i<x_vec.size();++i){
        double tmpx = x_vec[i]*cos(veh_yaw_rad)-y_vec[i]*sin(veh_yaw_rad);
        double tmpy = x_vec[i]*sin(veh_yaw_rad)-y_vec[i]*cos(veh_yaw_rad);
        x_vec[i] = tmpx + veh_x;
        y_vec[i] = tmpy + veh_y;
    }
}

vector<double> vcs2lcs(double x, double y, double veh_x, double veh_y, double veh_yaw_rad)
{
    double tmpx = x*cos(veh_yaw_rad)-y*sin(veh_yaw_rad);
    double tmpy = x*sin(veh_yaw_rad)+y*cos(veh_yaw_rad);
    tmpx += veh_x;
    tmpy += veh_y;
    return {tmpx,tmpy};
}

double lane2d(int lane)
{
    return (2+4*lane);
}

bool laneChangeOK(const sLaneStatus& lane)
{
    if(lane.isExist && lane.isOpen){
        return true;
    }
    return false;
}

#endif //COMMON_H