#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include "discretepath.h"

class MotionController
{
public:
    MotionController(DiscretePath * path);

    void updateVel(double pos_x, double pos_y, double pos_yaw, int interval);
    double getLinearVelocity();
    double getAngularVelocity();

    void setP(double p);
    void setD(double d);

    bool isFinished() { return mFinished; };
    bool isTargetReached(double pos_x, double pos_y);

protected:
    void next();

    double mLinearVel;
    double mAngularVel;

    double mP;
    double mD;

    double mOldErrX;
    double mOldErrY;
    double mOldErrYaw;

    bool mFinished;

    double mReachThreshold;

    int mCtrlIdx;
    DiscretePath * mpPath;
};

#endif // MOTIONCONTROLLER_H
