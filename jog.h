#include <math.h>

class jog
{
public:
    jog();
    double x;
    double y;
    double th;
    double xt_old;
    double yt_old;
    int cmd(double xt, double yt, double dt);
};

