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

void normalization(sts *sts)
{
    sts->prev_lat = sts->lat;
    sts->prev_alt = sts->alt;
    sts->prev_t_lat = sts->t_lat;
    sts->prev_t_alt = sts->t_alt;
    sts->dx = sts->t_lat - sts->lat;
    sts->dy = sts->t_alt - sts->alt;
    sts->distance = sqrt(pow(sts->dx, 2) + pow(sts->dy, 2));
    sts->theta = atan2(sts->dy, sts->dx);
    sts->theta_deg = sts->theta * 180 / M_PI;
    sts->nx = sts->dx / sts->distance;
    sts->ny = sts->dy / sts->distance;
    printf("dx = %f, dy = %f\n", sts->dx, sts->dy);
    printf("distance: %f, theta: %f, nx: %f, ny: %f\n", sts->distance, sts->theta_deg, sts->nx, sts->ny);


}
void calculate_acceleration(sts *state, double n)
{
    double los_rate = (state->t_vel * sin(state->theta) - state->vel * sin(state->theta)) / state->distance;
    double a_n = n * state->vel * los_rate;
    state->ax = a_n * cos(state->theta + M_PI / 2);
    state->ay = a_n * sin(state->theta + M_PI / 2);
    printf("ax = %f, ay = %f\n", state->ax, state->ay);
}
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

int main() 
{ 
    sdl_prep sdl_prep = win_prep();
    if (sdl_prep.win == NULL || sdl_prep.ren == NULL) {
        return 1;
    }

    int running = 1;
    float angle = 0.0f;
    sts sts;
    sts.lat = 0; //representing x
    //sts.lon = 0;
    sts.alt = 0;// representing y
    sts.vel = 21;
    
    //target
    sts.t_lat = 800;
    //sts.t_lon = 0;
    sts.t_alt = 1000;
    sts.t_vel = 0;
    sts.dt = 0.1;
    double n = 30;
    normalization(&sts);


    while (running && sts.distance > 8)
    {
        normalization(&sts);
        calculate_acceleration(&sts, n);
        printf("d: %f, theta: %f, nx: %f, ny: %f\n", sts.distance, sts.theta_deg, sts.nx, sts.ny);

        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
                running = 0;
        }
        // Clear the screen
        SDL_SetRenderDrawColor(sdl_prep.ren, 0, 0, 150, 150); // Black color
        SDL_RenderClear(sdl_prep.ren);

        // Draw the point
        SDL_SetRenderDrawColor(sdl_prep.ren, 0, 255, 0, 255); // Green color
        // Update positions (example update, replace with actual logic)
        sts.vel_x += sts.ax * sts.dt;
        sts.vel_y += sts.ay * sts.dt;
        sts.theta_dot = sts.vel * (sts.ny * sts.t_vel_x - sts.nx * sts.t_vel_y) / pow(sts.distance, 2);
        sts.lat += sts.nx * sts.vel_x * sts.dt;
        sts.alt += sts.ny * (-sts.vel_y) * sts.dt;
        sts.t_lat += sts.nx * sts.t_vel * sts.dt;
        sts.t_alt += sts.ny * sts.t_vel * sts.dt;

        printf("theta_dot : %f lat: %f, alt: %f, t_lat: %f, t_alt: %f\n", sts.theta_dot,sts.lat, sts.alt, sts.t_lat, sts.t_alt);
        sleep(0.7);
        for (int w = 0; w < POINT_RADIUS * 2; w++) {
            for (int h = 0; h < POINT_RADIUS * 2; h++) {
                int dx = POINT_RADIUS - w; // horizontal offset
                int dy = POINT_RADIUS - h; // vertical offset
                if ((dx*dx + dy*dy) <= (POINT_RADIUS * POINT_RADIUS)) {
                    SDL_RenderDrawPoint(sdl_prep.ren, sts.lat + dx, sts.alt + dy);
                    SDL_RenderDrawPoint(sdl_prep.ren, sts.t_lat+dx, sts.t_alt+dy);
                    SDL_RenderDrawLine(sdl_prep.ren, sts.lat, sts.alt, sts.t_lat, sts.t_alt);
                    //gnc();
                    
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