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

    // Kinematic model parameters
    double error_theta_max = DEG_TO_RAD(45);
    double v_max = 0.2, Kp_omega = 1.5, omega_max = 0.5 * PI;

    // Obstacle setup
    double obst1_points[400][2]; // Placeholder for obstacle 1
    double obst2_points[400][2]; // Placeholder for obstacle 2
    // ... Populate obstacles as needed (similar to MATLAB setup)

    // V-formation parameters
    double v_distance = 0.3, v_angle = DEG_TO_RAD(45);

    // Simulation variables
    double X[NUM_POINTS][MAX_TIMESTEPS] = {0}, Y[NUM_POINTS][MAX_TIMESTEPS] = {0};
    double V_history[MAX_TIMESTEPS] = {0};
    double dT = 0.1;
    int t_max = MAX_TIMESTEPS, t = 0;
    bool all_reached_goal;

    // Initialize paths
    for (int i = 0; i < NUM_POINTS; i++) {
        X[i][0] = x[i];
        Y[i][0] = y[i];
    }

    while (t < t_max) {
        all_reached_goal = true;

        // Calculate total Lyapunov function
        double V_leader = 0.5 * (pow(x[0] - x_goal, 2) + pow(y[0] - y_goal, 2));
        double V_follower1 = 0.5 * (pow(x[1] - (x[0] - v_distance * cos(theta[0] + v_angle)), 2) +
                                    pow(y[1] - (y[0] - v_distance * sin(theta[0] + v_angle)), 2));
        double V_follower2 = 0.5 * (pow(x[2] - (x[0] - v_distance * cos(theta[0] - v_angle)), 2) +
                                    pow(y[2] - (y[0] - v_distance * sin(theta[0] - v_angle)), 2));
        double V_total = V_leader + V_follower1 + V_follower2;
        V_history[t] = V_total;

        for (int i = 0; i < NUM_POINTS; i++) {
            if (i == 0) {
                // Leader behavior
                if (calculate_distance(x[0], y[0], x_goal, y_goal) > position_accuracy) {
                    all_reached_goal = false;

                    double nablaU_att[2] = {0}, nablaU_rep[2] = {0}, nablaU[2] = {0};
                    calculate_attractive_potential(x[0], y[0], x_goal, y_goal, zeta, dstar, nablaU_att);
                    calculate_repulsive_potential(x[0], y[0], obst1_points, 400, eta, Qstar, nablaU_rep);

                    // Combine potentials
                    nablaU[0] = nablaU_att[0] + nablaU_rep[0];
                    nablaU[1] = nablaU_att[1] + nablaU_rep[1];

                    // Calculate theta_ref and velocity
                    double theta_ref = atan2(-nablaU[1], -nablaU[0]);
                    double error_theta = theta_ref - theta[0];
                    double alpha = (fabs(error_theta) <= error_theta_max) ? 
                                   (error_theta_max - fabs(error_theta)) / error_theta_max : 0;
                    double v_ref = MIN(alpha * sqrt(pow(nablaU[0], 2) + pow(nablaU[1], 2)), v_max);
                    double omega_ref = Kp_omega * error_theta;
                    omega_ref = MAX(MIN(omega_ref, omega_max), -omega_max);

                    // Update kinematics
                    update_kinematics(&x[0], &y[0], &theta[0], v_ref, omega_ref, dT);
                }
            } else {
                // Follower behavior (similar to leader, adjusting for V-formation position)
                double angle_offset = (i == 1) ? v_angle : -v_angle;
                double formation_x = x[0] - v_distance * cos(theta[0] + angle_offset);
                double formation_y = y[0] - v_distance * sin(theta[0] + angle_offset);

                if (calculate_distance(x[i], y[i], formation_x, formation_y) > position_accuracy) {
                    all_reached_goal = false;

                    double nablaU_att[2] = {0}, nablaU_rep[2] = {0}, nablaU[2] = {0};
                    calculate_attractive_potential(x[i], y[i], formation_x, formation_y, zeta, dstar, nablaU_att);
                    calculate_repulsive_potential(x[i], y[i], obst1_points, 400, eta, Qstar, nablaU_rep);

                    // Combine potentials
                    nablaU[0] = nablaU_att[0] + nablaU_rep[0];
                    nablaU[1] = nablaU_att[1] + nablaU_rep[1];

                    // Calculate theta_ref and velocity
                    double theta_ref = atan2(-nablaU[1], -nablaU[0]);
                    double error_theta = theta_ref - theta[i];
                    double alpha = (fabs(error_theta) <= error_theta_max) ? 
                                   (error_theta_max - fabs(error_theta)) / error_theta_max : 0;
                    double v_ref = MIN(alpha * sqrt(pow(nablaU[0], 2) + pow(nablaU[1], 2)), v_max);
                    double omega_ref = Kp_omega * error_theta;
                    omega_ref = MAX(MIN(omega_ref, omega_max), -omega_max);

                    // Update kinematics
                    update_kinematics(&x[i], &y[i], &theta[i], v_ref, omega_ref, dT);
                }
            }
        }

        // Stop simulation if all reached the goal
        if (all_reached_goal) break;

        // Store the current positions
        for (int i = 0; i < NUM_POINTS; i++) {
            X[i][t] = x[i];
            Y[i][t] = y[i];
        }

        t++;
    }

    // Display results
    printf("Simulation completed in %.2f seconds\n", t * dT);
    print_path(X[0], Y[0], t);

    return 0;
}
