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



#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define POINT_RADIUS 10
#define TRAJECTORY_RADIUS 100
#define SPEED 0.05

typedef struct {
    SDL_Window *win;
    SDL_Renderer *ren;
} sdl_prep;




#endif