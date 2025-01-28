// filepath: /home/joddb/mavlink_control/src/gnc.c
#include "../include/gnc.h"

void normalization(sts *state)
{
    state->prev_lat = state->lat;
    state->prev_alt = state->alt;
    state->prev_t_lat = state->t_lat;
    state->prev_t_alt = state->t_alt;
    state->dx = state->t_lat - state->lat;
    state->dy = state->t_alt - state->alt;
    state->distance = sqrt(pow(state->dx, 2) + pow(state->dy, 2));
    state->theta = atan2(state->dy, state->dx);
    state->theta_deg = state->theta * 180 / M_PI;
    state->nx = state->dx / state->distance;
    state->ny = state->dy / state->distance;
    printf("dx = %f, dy = %f\n", state->dx, state->dy);
    printf("distance: %f, theta: %f, nx: %f, ny: %f\n", state->distance, state->theta_deg, state->nx, state->ny);
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
    sts state;
    state.lat = 0; //representing x
    //state.lon = 0;
    state.alt = 0;// representing y
    state.vel = 21;
    
    //target
    state.t_lat = 200;
    //state.t_lon = 0;
    state.t_alt = 400;
    state.t_vel = 0;



    state.dt = 0.1;

    normalization(&state);

    while(state.distance > 0.5)
    {
        normalization(&state); 
        printf("d: %f, theta: %f, nx: %f, ny: %f\n", state.distance, state.theta_deg, state.nx, state.ny);

        // Update positions (example update, replace with actual logic)
        state.theta_dot = state.vel * (state.ny * state.t_vel_x - state.nx * state.t_vel_y) / pow(state.distance, 2);
        state.lat += state.nx * state.vel * state.dt;
        state.alt += state.ny * state.vel * state.dt;
        state.t_lat += state.nx * state.t_vel * state.dt;
        state.t_alt += state.ny * state.t_vel * state.dt;
        printf("theta_dot : %f lat: %f, alt: %f, t_lat: %f, t_alt: %f\n", state.theta_dot,state.lat, state.alt, state.t_lat, state.t_alt);
        sleep(1);
    }
}

int main() {
    gnc();
    return 0;
}