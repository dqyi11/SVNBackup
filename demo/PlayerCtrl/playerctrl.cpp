#include "playerctrl.h"
#include "playercommunicator.h"


PlayerCtrl::PlayerCtrl()
{
    mCommunicator = new PlayerCommunicator();
}

void PlayerCtrl::connect()
{
    if(mCommunicator)
    {
        mCommunicator->connect();
    }
}

void PlayerCtrl::update()
{
    if(mCommunicator)
    {
        mCommunicator->update();
    }
}

void PlayerCtrl::setScale(double scale)
{
    mScale = scale;
}

double PlayerCtrl::getXPos()
{
    if(mCommunicator)
    {
        return mCommunicator->getXPos() * mScale;
    }

    return 0.0;
}

double PlayerCtrl::getYPos()
{
    if(mCommunicator)
    {
        return mCommunicator->getYPos() * mScale;
    }

    return 0.0;
}

double PlayerCtrl::getYaw()
{
    if(mCommunicator)
    {
        return mCommunicator->getYaw() * mScale;
    }

    return 0.0;
}

double PlayerCtrl::getXSpeed()
{
    if(mCommunicator)
    {
        return mCommunicator->getXSpeed() * mScale;
    }

    return 0.0;
}

double PlayerCtrl::getYSpeed()
{
    if(mCommunicator)
    {
        return mCommunicator->getYSpeed() * mScale;
    }

    return 0.0;
}

double PlayerCtrl::getYawSpeed()
{
    if(mCommunicator)
    {
        return mCommunicator->getYawSpeed() * mScale;
    }

    return 0.0;
}

bool PlayerCtrl::setSpeed(double linearVel, double angularVel)
{
    if(mCommunicator)
    {
        mCommunicator->setSpeed(linearVel / mScale, angularVel);
        return true;
    }
    return false;
}

bool PlayerCtrl::setOdemetry(double x, double y, double yaw)
{
    if(mCommunicator)
    {
        mCommunicator->setOdemetry(x / mScale, y/mScale, yaw);
        return true;
    }
    return false;
}
