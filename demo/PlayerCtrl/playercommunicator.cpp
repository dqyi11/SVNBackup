#include "playercommunicator.h"
#include <QDebug>

PlayerCommunicator::PlayerCommunicator()
{
    robot = NULL;
    pp = NULL;
    mXPos = 0.0;
    mYPos = 0.0;
    mYaw = 0.0;
    mXSpeed = 0.0;
    mYSpeed = 0.0;
    mYawSpeed = 0.0;
}

void PlayerCommunicator::connect()
{
    robot = new PlayerClient("localhost");
    pp = new Position2dProxy(robot, 0);
}

void PlayerCommunicator::update()
{
    robot->Read();
    mXPos = pp->GetXPos();
    mYPos = pp->GetYPos();
    mYaw = pp->GetYaw();
    mXSpeed = pp->GetXSpeed();
    mYSpeed = pp->GetYSpeed();
    mYawSpeed = pp->GetYawSpeed();
    qDebug("X %f Y %f Yaw %f", mXPos, mYPos, mYaw);
    qDebug("XSpeed %f YSpeed %f YawSpeed %f", mXSpeed, mYSpeed, mYawSpeed);

}

void PlayerCommunicator::setSpeed(double linearVel, double angularVel)
{
    pp->SetCarlike(linearVel, angularVel);
    pp->SetMotorEnable(true);
}

void PlayerCommunicator::setOdemetry(double x, double y, double yaw)
{
    pp->SetOdometry(x, y, yaw);
}

double PlayerCommunicator::getXPos()
{
    return mXPos;
}

double PlayerCommunicator::getYPos()
{
    return mYPos;
}

double PlayerCommunicator::getYaw()
{
    return mYaw;
}

double PlayerCommunicator::getXSpeed()
{
    return mXSpeed;
}

double PlayerCommunicator::getYSpeed()
{
    return mYSpeed;
}

double PlayerCommunicator::getYawSpeed()
{
    return mYawSpeed;
}
