#ifndef PLAYERCTRL_H
#define PLAYERCTRL_H

class PlayerCommunicator;

class PlayerCtrl
{

public:
    PlayerCtrl();
    void connect();
    void update();

    void setScale(double scale);
    bool setSpeed(double linearVel, double angularVel);
    bool setOdemetry(double x, double y, double yaw);

    double getXPos();
    double getYPos();
    double getYaw();
    double getXSpeed();
    double getYSpeed();
    double getYawSpeed();

private:
    PlayerCommunicator * mCommunicator;
    double mScale;
};

#endif // PLAYERCTRL_H
