#include "urobot.h"
#include <cmath>

Robot::Robot(double start_x, double start_y, double dest_x, double dest_y) {
    x = start_x;
    y = start_y;
    this->dest_x = dest_x;
    this->dest_y = dest_y;
    theta = atan2(dest_y - start_y, dest_x - start_x);
}

void Robot::addObstacle(double x, double y) {
    obstacles.push_back({ x, y });
}

void Robot::navigate() {
    while (x != dest_x || y != dest_y) {
        // calculate direction and distance to move the robot
        double dx = dest_x - x;
        double dy = dest_y - y;
        double dist = sqrt(dx*dx + dy*dy);
        dx /= dist;
        dy /= dist;

        // check for obstacles in the path of the robot
        bool obstacle_found = false;
        for (Obstacle obs : obstacles) {
            double obs_dx = obs.x - x;
            double obs_dy = obs.y - y;
            double obs_dist = sqrt(obs_dx*obs_dx + obs_dy*obs_dy);
            obs_dx /= obs_dist;
            obs_dy /= obs_dist;

            // check if the line between the robot and the destination intersects with an obstacle
            if (abs((dx*obs_dy - dy*obs_dx)*obs_dist) < ROBOT_RADIUS) {
                // obstacle found, adjust robot direction to avoid it
                dx += obs_dy;
                dy -= obs_dx;
                dist = sqrt(dx*dx + dy*dy);
                dx /= dist;
                dy /= dist;
                obstacle_found = true;
                break;
            }
        }

        if (!obstacle_found) {
            // no obstacles in the way, move the robot towards the destination
            double new_x = x + dx * ROBOT_SPEED;
            double new_y = y + dy * ROBOT_SPEED;

            // check if the robot would go outside the boundaries
            if (new_x < X_MIN || new_x > X_MAX || new_y < Y_MIN || new_y > Y_MAX) {
                // robot would go outside the boundaries, do not move it
            } else {
                x = new_x;
                y = new_y;
                theta = atan2(dest_y - y, dest_x - x);
            }
        }
    }
}
