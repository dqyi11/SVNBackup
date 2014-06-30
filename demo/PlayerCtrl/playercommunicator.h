#ifndef PLAYERCOMMUNICATOR_H
#define PLAYERCOMMUNICATOR_H

#include <libplayerc++/playerc++.h>
using namespace PlayerCc;

class PlayerCommunicator
{
public:
    PlayerCommunicator();

    void update();
    void connect();

    double getXPos();
    double getYPos();
    double getYaw();
    double getXSpeed();
    double getYSpeed();
    double getYawSpeed();

    void setSpeed(double linearVel, double angularVel);
    void setOdemetry(double x, double y, double yaw);

private:
    PlayerClient * robot;
    Position2dProxy * pp;

    double mXPos;
    double mYPos;
    double mYaw;
    double mXSpeed;
    double mYSpeed;
    double mYawSpeed;
};

#endif // PLAYERCOMMUNICATOR_H
