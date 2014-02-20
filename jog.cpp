#include <math.h>

#include "jog.h"

jog::jog()
{
    x = 20;
    y = 20;
    th = 0;
    xt_old = -1;
    yt_old = -1;
}

double sqr(double a)
{
    return a*a;
}

int jog::cmd(double xt, double yt, double dt)
{
    double vxt, vyt;

    if(xt_old!=-1){
        vxt = (xt - xt_old)/dt;
        vyt = (yt - yt_old)/dt;
    }
    else{
        xt_old = xt;
        yt_old = yt;
        return -1;
    }

    //Définition du vecteur vitesse consigne
    double d, t, vxj, vyj, vl;
    int thresh = 50;

    d = sqrt(sqr(xt-x)+sqr(yt-y));
    if(d<50) vl = 10;
    else vl = 70;
    t = d/vl;

    vxj = (xt-x);///t;// + vxt;
    vyj = (yt-y);///t;// + vyt;

    //Transformation consigne
    double theta, theta_0, err;
    theta_0 = 0;
    theta = theta_0 + atan2(vyj,vxj);



    err = atan2(sin(th - theta),cos(th-theta));
//    if(err < -M_PI) err += 360;
//    if(err > 180) err -= 360;


    //Euler
    double kp = 1.5;
    x += vl*cos(th)*dt;
    y += vl*sin(th)*dt;
    th += -kp*err*dt;

    xt_old = xt;
    yt_old = yt;

    return 0;
}



