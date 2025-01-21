#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#define PI 3.141592653589793
#define MAX_TIMESTEPS 1000
#define NUM_POINTS 3

// Helper macros
#define DEG_TO_RAD(deg) ((deg) * (PI / 180))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

// Function prototypes
double calculate_distance(double x1, double y1, double x2, double y2);
void calculate_attractive_potential(double x, double y, double goal_x, double goal_y, double zeta, double dstar, double *nablaU_att);
void calculate_repulsive_potential(double x, double y, double **obstacles, int num_obstacles, double eta, double Qstar, double *nablaU_rep);
void update_kinematics(double *x, double *y, double *theta, double v_ref, double omega_ref, double dT);
void print_path(double *x_path, double *y_path, int steps);

int main() {
    // Initialize positions and orientations for the points
    double x[NUM_POINTS] = {-0.5, -0.5, 4};
    double y[NUM_POINTS] = {1, 0.8, 0.5};
    double theta[NUM_POINTS] = {0, 0, 0};

    // Goal position
    double x_goal = 3.5, y_goal = 2.75;
    double position_accuracy = 0.05;

    // APF parameters
    double zeta = 1.1547, eta = 0.0732, dstar = 0.3, Qstar = 0.75;

    // Allocate and initialize obstacles
    int num_obstacles = 400;
    double **obst1_points = (double **)malloc(num_obstacles * sizeof(double *));
    for (int i = 0; i < num_obstacles; i++) {
        obst1_points[i] = (double *)malloc(2 * sizeof(double));
        // Initialize obstacle points (example values)
        obst1_points[i][0] = (double)i / 10.0;
        obst1_points[i][1] = (double)i / 20.0;
    }

    double nablaU_rep[2] = {0, 0};

    // Debug prints
    printf("Before calculate_repulsive_potential\n");
    printf("x[0] = %f, y[0] = %f\n", x[0], y[0]);
    printf("obst1_points[0] = (%f, %f)\n", obst1_points[0][0], obst1_points[0][1]);

    // Call the function
    calculate_repulsive_potential(x[0], y[0], obst1_points, num_obstacles, eta, Qstar, nablaU_rep);

    // Debug prints
    printf("After calculate_repulsive_potential\n");
    printf("nablaU_rep[0] = %f, nablaU_rep[1] = %f\n", nablaU_rep[0], nablaU_rep[1]);

    // Free allocated memory
    for (int i = 0; i < num_obstacles; i++) {
        free(obst1_points[i]);
    }
    free(obst1_points);

    return 0;
}

double calculate_distance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

void calculate_attractive_potential(double x, double y, double goal_x, double goal_y, double zeta, double dstar, double *nablaU_att) {
    double distance = calculate_distance(x, y, goal_x, goal_y);
    if (distance <= dstar) {
        nablaU_att[0] = zeta * (x - goal_x);
        nablaU_att[1] = zeta * (y - goal_y);
    } else {
        nablaU_att[0] = dstar * zeta * (x - goal_x) / distance;
        nablaU_att[1] = dstar * zeta * (y - goal_y) / distance;
    }
}

void calculate_repulsive_potential(double x, double y, double **obstacles, int num_obstacles, double eta, double Qstar, double *nablaU_rep) {
    nablaU_rep[0] = 0;
    nablaU_rep[1] = 0;
    for (int i = 0; i < num_obstacles; i++) {
        double obs_x = obstacles[i][0];
        double obs_y = obstacles[i][1];
        double distance = calculate_distance(x, y, obs_x, obs_y);
        if (distance <= Qstar) {
            nablaU_rep[0] += eta * (1.0 / Qstar - 1.0 / distance) * (1.0 / (distance * distance)) * (x - obs_x);
            nablaU_rep[1] += eta * (1.0 / Qstar - 1.0 / distance) * (1.0 / (distance * distance)) * (y - obs_y);
        }
    }
}

void update_kinematics(double *x, double *y, double *theta, double v_ref, double omega_ref, double dT) {
    *x += v_ref * cos(*theta) * dT;
    *y += v_ref * sin(*theta) * dT;
    *theta += omega_ref * dT;
}

void print_path(double *x_path, double *y_path, int steps) {
    for (int i = 0; i < steps; i++) {
        printf("Step %d: x = %f, y = %f\n", i, x_path[i], y_path[i]);
    }
}
