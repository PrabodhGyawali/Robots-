#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <signal.h>

// Serial port configuration
#define SERIAL_PORT "/dev/ttyUSB0" // Adjust to your ESP32's serial port
#define BAUD_RATE B115200

// Servo and gripper state
float base_angle = 90.0f; // Mid-point (51-99 duty -> 0-180°)
float link_angle = 90.0f;
char gripper_state[10] = "Closed";
const float DUTY_STEP = 5.0f;
const float DUTY_RANGE = 48.0f; // 99 - 51
const float ANGLE_PER_DUTY = 180.0f / DUTY_RANGE;

// Serial file descriptor
int serial_fd = -1;

// Signal handler for clean exit
void handle_signal(int sig) {
    if (serial_fd != -1) {
        write(serial_fd, "q", 1); // Send LED blink command
        close(serial_fd);
    }
    printf("\033[0m\nExiting...\n"); // Reset terminal colors
    exit(0);
}

// Initialize serial port
int init_serial() {
    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        perror("Error opening serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, BAUD_RATE);
    cfsetospeed(&options, BAUD_RATE);
    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~(PARENB | CSTOPB);
    tcsetattr(serial_fd, TCSANOW, &options);
    return 0;
}

// Initialize terminal for non-blocking input
void init_terminal(struct termios *old_termios) {
    struct termios new_termios;
    tcgetattr(STDIN_FILENO, old_termios);
    new_termios = *old_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN] = 0;
    new_termios.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
}

// Restore terminal settings
void restore_terminal(struct termios *old_termios) {
    tcsetattr(STDIN_FILENO, TCSANOW, old_termios);
}

// Clear screen
void clear_screen() {
    printf("\033[2J\033[H"); // ANSI clear screen and move cursor to top-left
}

// Print ASCII interface
void print_interface(char pressed_key) {
    clear_screen();
    printf("=== 2 DOF Robot CLI Control ===\n");
    printf("Base Angle: %.1f° | Link Angle: %.1f° | Gripper: %s\n", base_angle, link_angle, gripper_state);
    printf("\nControls:\n");
    printf("  W: Base clockwise | S: Base counterclockwise\n");
    printf("  A: Link up        | D: Link down\n");
    printf("  O: Open gripper   | C: Close gripper\n");
    printf("  Q: Blink LED (and exit)\n");
    printf("\nLayout:\n");

    // ANSI colors: green background (\033[42m), reset (\033[0m)
    char *w_color = (pressed_key == 'w' || pressed_key == 'W') ? "\033[42m" : "\033[0m";
    char *a_color = (pressed_key == 'a' || pressed_key == 'A') ? "\033[42m" : "\033[0m";
    char *s_color = (pressed_key == 's' || pressed_key == 'S') ? "\033[42m" : "\033[0m";
    char *d_color = (pressed_key == 'd' || pressed_key == 'D') ? "\033[42m" : "\033[0m";
    char *o_color = (pressed_key == 'o' || pressed_key == 'O') ? "\033[42m" : "\033[0m";
    char *c_color = (pressed_key == 'c' || pressed_key == 'C') ? "\033[42m" : "\033[0m";
    char *q_color = (pressed_key == 'q' || pressed_key == 'Q') ? "\033[42m" : "\033[0m";

    printf("     %s[W]\033[0m\n", w_color);
    printf("%s[A]\033[0m   %s[S]\033[0m   %s[D]\033[0m\n", a_color, s_color, d_color);
    printf("%s[O]\033[0m   %s[C]\033[0m   %s[Q]\033[0m\n", o_color, c_color, q_color);
    printf("\nPress keys to control. Hold for continuous movement. Ctrl+C or Q to exit.\n");
}

int main() {
    // Initialize signal handler
    signal(SIGINT, handle_signal);

    // Initialize serial port
    if (init_serial() != 0) {
        return 1;
    }

    // Initialize terminal for non-blocking input
    struct termios old_termios;
    init_terminal(&old_termios);

    // Main loop
    char key = '\0';
    print_interface('\0');
    while (1) {
        // Check for key press
        char new_key = '\0';
        if (read(STDIN_FILENO, &new_key, 1) > 0) {
            // Process valid keys (case-insensitive)
            if (strchr("wasdocqWASDOCQ", new_key)) {
                key = new_key | 32; // Convert to lowercase
                write(serial_fd, &key, 1); // Send to ESP32

                // Update local state
                if (key == 'w' && base_angle < 180.0f) {
                    base_angle += DUTY_STEP * ANGLE_PER_DUTY;
                } else if (key == 's' && base_angle > 0.0f) {
                    base_angle -= DUTY_STEP * ANGLE_PER_DUTY;
                } else if (key == 'a' && link_angle < 180.0f) {
                    link_angle += DUTY_STEP * ANGLE_PER_DUTY;
                } else if (key == 'd' && link_angle > 0.0f) {
                    link_angle -= DUTY_STEP * ANGLE_PER_DUTY;
                } else if (key == 'o') {
                    strcpy(gripper_state, "Open");
                } else if (key == 'c') {
                    strcpy(gripper_state, "Closed");
                } else if (key == 'q') {
                    print_interface(key);
                    usleep(100000); // Brief highlight
                    write(serial_fd, "q", 1);
                    close(serial_fd);
                    restore_terminal(&old_termios);
                    printf("\033[0m\nExiting...\n");
                    return 0;
                }
                print_interface(key);
            }
        } else {
            // No key pressed, reset highlight
            if (key != '\0') {
                print_interface('\0');
                key = '\0';
            }
        }

        usleep(10000); // 10ms sleep to prevent CPU overload
    }

    // Cleanup (unreachable due to loop, but included for completeness)
    close(serial_fd);
    restore_terminal(&old_termios);
    printf("\033[0m");
    return 0;
}