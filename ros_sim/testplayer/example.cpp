#include <iostream>
#include <libplayerc++/playerc++.h>

int main(int argc, char *argv[])
{
    using namespace PlayerCc;

    PlayerClient robot("localhost");
    Position2dProxy pp(&robot, 0);

    for(;;)
    {
        robot.Read();
        std::cout << pp << std::endl; 
    }
}
