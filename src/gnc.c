#include "../include/gnc.h"

static sdl_prep win_prep()
{
    sdl_prep sdl_prep;

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
        sdl_prep.win = NULL;
        sdl_prep.ren = NULL;
        return sdl_prep;
    }

    sdl_prep.win = SDL_CreateWindow("SDL Trajectory", 100, 50, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);
    if (sdl_prep.win == NULL) {
        fprintf(stderr, "SDL_CreateWindow Error: %s\n", SDL_GetError());
        SDL_Quit();
        sdl_prep.win = NULL;
        sdl_prep.ren = NULL;
        return sdl_prep;
    }

    sdl_prep.ren = SDL_CreateRenderer(sdl_prep.win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (sdl_prep.ren == NULL) {
        SDL_DestroyWindow(sdl_prep.win);
        fprintf(stderr, "SDL_CreateRenderer Error: %s\n", SDL_GetError());
        SDL_Quit();
        sdl_prep.win = NULL;
        sdl_prep.ren = NULL;
        return sdl_prep;
    }

    return sdl_prep;
}

void gnc()
{
    /*
    * setup the intial state of fixed wing
    * measure relative velocity
    * calculate the line of sight
    * rate of the line of sight
    * velocity of persuer and target
    * attitude and oriantation
    * pure proportional navigation
    * set limits of speed and acceleration
    * get the required command of velocity and acceleration
    */
    //missile
    double lat = 0; //representing x
    //double lon = 0;
    double alt = 0;// representing y
    double vel = 0;
    
    //target
    double t_lat = 200;
    //double t_lon = 0;
    double t_alt = 400;
    double t_vel = 0;

    // normalization of position
    double dx = t_lat - lat;
    double dy = t_alt - alt;

    double distance = sqrt(pow(dx, 2) + pow(dy, 2));
    while(distance > 0.5)
    {
        dx = t_lat - lat;
        dy = t_alt - alt;
        distance = sqrt(pow(dx, 2) + pow(dy, 2));
        
        double theta = atan2(dy, dx);
        double theta_deg = theta * 180 / M_PI;
        double nx = dx / distance;
        double ny = dy / distance;
        printf("distance: %f, theta: %f, nx: %f, ny: %f\n", distance, theta_deg, nx, ny);

        //relative velocity

    }

    



}

int main() 
{ 
    sdl_prep sdl_prep = win_prep();
    if (sdl_prep.win == NULL || sdl_prep.ren == NULL) {
        return 1;
    }

    int running = 1;
    float angle = 0.0f;

    while (running)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
                running = 0;
        }

        // Calculate the point's position
        int x = WINDOW_WIDTH / 2 + TRAJECTORY_RADIUS * cos(angle);
        int y = WINDOW_HEIGHT / 2 + TRAJECTORY_RADIUS * sin(angle);
        angle += SPEED;

        // Clear the screen
        SDL_SetRenderDrawColor(sdl_prep.ren, 0, 0, 150, 150); // Black color
        SDL_RenderClear(sdl_prep.ren);

        // Draw the point
        SDL_SetRenderDrawColor(sdl_prep.ren, 0, 255, 0, 255); // Green color
        for (int w = 0; w < POINT_RADIUS * 2; w++) {
            for (int h = 0; h < POINT_RADIUS * 2; h++) {
                int dx = POINT_RADIUS - w; // horizontal offset
                int dy = POINT_RADIUS - h; // vertical offset
                if ((dx*dx + dy*dy) <= (POINT_RADIUS * POINT_RADIUS)) {
                    SDL_RenderDrawPoint(sdl_prep.ren, x + dx, y + dy);
                    SDL_RenderDrawPoint(sdl_prep.ren, 200+dx, 200+dy);
                    gnc();
                }
            }
        }

        // Present the renderer
        SDL_RenderPresent(sdl_prep.ren);

        // Delay to control frame rate
        SDL_Delay(16); // Approximately 60 frames per second
    }

    SDL_DestroyRenderer(sdl_prep.ren);
    SDL_DestroyWindow(sdl_prep.win);
    SDL_Quit();

    return 0;
}