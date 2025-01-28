#ifndef GNC_H
#define GNC_H

#include "../c_library_v2/common/mavlink.h"
#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <SDL2/SDL.h>
#include <math.h>

#define M_PI 3.14159265358979323846



#define WINDOW_WIDTH 1600
#define WINDOW_HEIGHT 1200
#define POINT_RADIUS 5
#define TRAJECTORY_RADIUS 100
#define SPEED 0.0005

typedef struct {
    SDL_Window *win;
    SDL_Renderer *ren;
} sdl_prep;


typedef struct{
    double lat; //representing x
    //double lon;
    double alt;// representing y
    double vel;
    
    //target
    double t_lat;
    //double t_lon;
    double t_alt;
    double t_vel;

    double prev_lat;
    double prev_alt;
    double prev_t_lat;
    double prev_t_alt;
    double dx;
    double dy;

    double dt;
    double vel_x;
    double vel_y;
    double t_vel_x;
    double t_vel_y;

    double distance;
    double theta_deg;
    double theta;
    double nx;
    double ny;
    double theta_dot;
    double ax;
    double ay;
} sts;

void normalization(sts *sts);


#endif