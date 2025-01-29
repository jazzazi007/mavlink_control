#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <conio.h>

void set_nonblocking_mode(int enable) {
    struct termios tty;
    tcgetattr(STDIN_FILENO, &tty);
    if (enable) {
        tty.c_lflag &= ~(ICANON | ECHO);
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 0;
    } else {
        tty.c_lflag |= (ICANON | ECHO);
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &tty);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (enable) {
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    } else {
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
    }
}

int main() {
    char c;
    printf("Press space bar to perform action, any other key to continue...\n");

    set_nonblocking_mode(1);
    int i = 0;

    while (1) {
        c = getchar();
        if (c != EOF) {
            if (c == ' ') {
                // Perform action if space bar is pressed
                printf("Space bar pressed. Performing action...\n");
                i = 27;
                // Add your logic here
            } else {
                // Continue with the rest of the code
                printf("Continuing with the rest of the code...\n");
                            i=0; 

                // Add the rest of your code here
                // break; // Exit the loop if not space bar
            }
        } else {
            // No input, continue with the rest of the code
            printf("No input. Continuing...\n");
            // Add the rest of your code here
        }
        if (i == 27) {
            printf("Exiting the loop...\n");
        }

        usleep(1000); // Sleep for 100 milliseconds
    }

    set_nonblocking_mode(0);
    return 0;
}