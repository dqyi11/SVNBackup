#include "motioncontroller.h"
#include <QtCore/qmath.h>

MotionController::MotionController(DiscretePath * path)
{
    mpPath = path;
    mCtrlIdx = 0;

    mP = 0.8;
    mD = 0.1;

    mOldErrX = 0.0;
    mOldErrY = 0.0;
    mOldErrYaw = 0.0;

    mFinished = false;
    mReachThreshold = 2.0;
}

void MotionController::setP(double p)
{
    mP = p;
}

void MotionController::setD(double d)
{
    mD = d;
}

void MotionController::updateVel(double pos_x, double pos_y, double pos_yaw, int interval)
{
    QPointF target = mpPath->mSequence[mCtrlIdx];

    double errX = target.x() - pos_x;
    double errY = target.y() - pos_y;

    double currentYaw = pos_yaw * 3.141592 / 180.0;

    double targetYaw = qAtan2(errX, errY);
    double errYaw = targetYaw - currentYaw;

    mLinearVel = 2.0;
    mAngularVel =  (mP * errYaw + mD * (errYaw - mOldErrYaw));

    qDebug("YAW - current %f target %f", currentYaw, targetYaw);

    mOldErrX = errX;
    mOldErrY = errY;
    mOldErrYaw = errYaw;

    if(isTargetReached(pos_x, pos_y) == true)
    {
        next();
    }
}

void MotionController::next()
{
    mCtrlIdx++;
    if(mCtrlIdx >= mpPath->length())
    {
        mFinished = true;
        mCtrlIdx = 0;
    }
}

bool MotionController::isTargetReached(double pos_x, double pos_y)
{
    QPointF target = mpPath->mSequence[mCtrlIdx];
    double dist = 0.0;
    dist = qSqrt(qPow(pos_x - target.x(), 2) + qPow(pos_y - target.y(), 2));

    if(dist <= mReachThreshold)
    {
        qDebug("target %d reached!", mCtrlIdx);
        return true;
    }

    return false;
}


double MotionController::getLinearVelocity()
{
    return mLinearVel;
}

double MotionController::getAngularVelocity()
{
    return mAngularVel;
}

