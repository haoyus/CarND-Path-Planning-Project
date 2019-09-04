#ifndef COMMON_H
#define COMMON_H

#include "helpers.h"

const double WP_STRIDE = 30;//meters
const unsigned int ADD_WP_NUM = 3;//additional wp number
const double MAX_VEL = 49.5;
const double MAX_ACC = 0.224;

struct sWayPoints{
    vector<double> x_vec;
    vector<double> y_vec;
    vector<double> s_vec;
    vector<double> d_vec;
};


double getSpd(double vx,double vy)
{
  return sqrt(vx*vx+vy*vy);
}

double mph2mps(double spd)
{
  return spd*0.447;
}

void setWpFrenet(sWayPoints& wps, double s, double d, int index)
{
//
}

void lcs2vcs(vector<double>& x_vec, vector<double>& y_vec, double veh_x, double veh_y, double veh_yaw_rds)
{
    for(int i=0;i<x_vec.size();++i){
        double tmpx = x_vec[i], tmpy = y_vec[i];
        tmpx -= veh_x;
        tmpy -= veh_y;
        x_vec[i] = tmpx*cos(veh_yaw_rds)+tmpy*sin(veh_yaw_rds);
        y_vec[i] = -1.0*tmpx*sin(veh_yaw_rds)+tmpy*cos(veh_yaw_rds);
    }
}

void vcs2lcs(vector<double>& x_vec, vector<double>& y_vec, double veh_x, double veh_y, double veh_yaw_rds)
{
    for(int i=0;i<x_vec.size();++i){
        double tmpx = x_vec[i]*cos(veh_yaw_rds)-y_vec[i]*sin(veh_yaw_rds);
        double tmpy = x_vec[i]*sin(veh_yaw_rds)-y_vec[i]*cos(veh_yaw_rds);
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

#endif //COMMON_H