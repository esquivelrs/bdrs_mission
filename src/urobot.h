#ifndef ROBOT_H
#define ROBOT_H

#include <vector>

struct Obstacle {
    double x;
    double y;
};

class Robot {
public:
    Robot(double start_x, double start_y, double dest_x, double dest_y);
    void addObstacle(double x, double y);
    void navigate();
private:
    double x;
    double y;
    double dest_x;
    double dest_y;
    double theta; // heading of the robot in radians
    std::vector<Obstacle> obstacles;
    const double ROBOT_RADIUS = 0.5;
    const double ROBOT_SPEED = 1.0;
    const double X_MIN = -10.0;
    const double X_MAX = 10.0;
    const double Y_MIN = -10.0;
    const double Y_MAX = 10.0;
};

#endif
