#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <signal.h>

#define SERIAL_PORT             "/dev/ttyUSB0"
#define BAUD_RATE               B115200
#define GRIPPER_STATE_OPEN      "Open"
#define GRIPPER_STATE_CLOSED    "Closed"
#define DUTY_STEP               5.0f
#define BASE_DUTY_MIN           25.0f
#define BASE_DUTY_MAX           50.0f
#define LINK_DUTY_MIN           51.0f
#define LINK_DUTY_MAX           99.0f
#define MAX_ANGLE               180.0f
#define MIN_ANGLE               0.0f
#define LOOP_SLEEP_US           10000U
#define HIGHLIGHT_SLEEP_US      100000U
#define STATUS_BUFFER_SIZE      128U

static const float BASE_DUTY_RANGE = BASE_DUTY_MAX - BASE_DUTY_MIN;
static const float LINK_DUTY_RANGE = LINK_DUTY_MAX - LINK_DUTY_MIN;
static const float BASE_ANGLE_PER_DUTY = MAX_ANGLE / BASE_DUTY_RANGE;
static const float LINK_ANGLE_PER_DUTY = MAX_ANGLE / LINK_DUTY_RANGE;

static float base_angle = ((BASE_DUTY_MIN + BASE_DUTY_MAX) / 2.0f - BASE_DUTY_MIN) * BASE_ANGLE_PER_DUTY;
static float link_angle = ((LINK_DUTY_MIN + LINK_DUTY_MAX) / 2.0f - LINK_DUTY_MIN) * LINK_ANGLE_PER_DUTY;
static char gripper_state[10] = GRIPPER_STATE_CLOSED;
static char status_message[STATUS_BUFFER_SIZE] = ""; // Buffer for latest status
static int serial_fd = -1;

static void handle_signal(int sig) {
    (void)sig; // Unused parameter
    if (serial_fd != -1) {
        write(serial_fd, "q", 1U);
        close(serial_fd);
    }
    printf("\033[0m\nExiting...\n");
    exit(0);
}

static int init_serial(void) {
    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        perror("Error opening serial port");
        return -1;
    }

    struct termios options;
    if (tcgetattr(serial_fd, &options) != 0) {
        perror("Error getting serial attributes");
        close(serial_fd);
        return -1;
    }
    cfsetispeed(&options, BAUD_RATE);
    cfsetospeed(&options, BAUD_RATE);
    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~(PARENB | CSTOPB);
    if (tcsetattr(serial_fd, TCSANOW, &options) != 0) {
        perror("Error setting serial attributes");
        close(serial_fd);
        return -1;
    }
    return 0;
}

static void init_terminal(struct termios *old_termios) {
    tcgetattr(STDIN_FILENO, old_termios);
    struct termios new_termios = *old_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN] = 0U;
    new_termios.c_cc[VTIME] = 0U;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
}

static void restore_terminal(const struct termios *old_termios) {
    tcsetattr(STDIN_FILENO, TCSANOW, old_termios);
}

static void clear_screen(void) {
    printf("\033[2J\033[H");
}

static void print_interface(char pressed_key) {
    clear_screen();
    printf("=== 2 DOF Robot CLI Control ===\n");
    printf("Base Angle: %.1f° | Link Angle: %.1f° | Gripper: %s\n", base_angle, link_angle, gripper_state);
    printf("\nLatest Status from Robot: %s\n", status_message);
    printf("\nControls:\n");
    printf("  W: Base clockwise | S: Base counterclockwise\n");
    printf("  A: Link up        | D: Link down\n");
    printf("  O: Open gripper   | C: Close gripper\n");
    printf("  Q: Blink LED (and exit)\n");
    printf("\nLayout:\n");

    const char *w_color = (pressed_key == 'w') ? "\033[42m" : "\033[0m";
    const char *a_color = (pressed_key == 'a') ? "\033[42m" : "\033[0m";
    const char *s_color = (pressed_key == 's') ? "\033[42m" : "\033[0m";
    const char *d_color = (pressed_key == 'd') ? "\033[42m" : "\033[0m";
    const char *o_color = (pressed_key == 'o') ? "\033[42m" : "\033[0m";
    const char *c_color = (pressed_key == 'c') ? "\033[42m" : "\033[0m";
    const char *q_color = (pressed_key == 'q') ? "\033[42m" : "\033[0m";

    printf("     %s[W]\033[0m\n", w_color);
    printf("%s[A]\033[0m   %s[S]\033[0m   %s[D]\033[0m\n", a_color, s_color, d_color);
    printf("%s[O]\033[0m   %s[C]\033[0m   %s[Q]\033[0m\n", o_color, c_color, q_color);
    printf("\nPress keys to control. Hold for continuous movement. Ctrl+C or Q to exit.\n");
}

static void read_status(void) {
    static char rx_buffer[STATUS_BUFFER_SIZE];
    static size_t rx_index = 0U;

    char byte;
    while (read(serial_fd, &byte, 1U) > 0) {
        if (rx_index < (sizeof(rx_buffer) - 1U)) {
            rx_buffer[rx_index++] = byte;
        }
        if (byte == '\n') {
            rx_buffer[rx_index - 1U] = '\0'; // Remove newline
            strncpy(status_message, rx_buffer, sizeof(status_message) - 1U);
            status_message[sizeof(status_message) - 1U] = '\0';
            rx_index = 0U;
            break;
        }
    }
}

int main(void) {
    signal(SIGINT, handle_signal);

    if (init_serial() != 0) {
        return 1;
    }

    struct termios old_termios;
    init_terminal(&old_termios);

    char key = '\0';
    print_interface('\0');
    while (1) {
        // Read status from serial
        read_status();

        char new_key = '\0';
        ssize_t bytes_read = read(STDIN_FILENO, &new_key, 1U);
        if (bytes_read > 0) {
            new_key |= 32; // Convert to lowercase (assuming ASCII)
            switch (new_key) {
                case 'w':
                case 's':
                case 'a':
                case 'd':
                case 'o':
                case 'c':
                case 'q':
                    key = new_key;
                    write(serial_fd, &key, 1U);

                    if (key == 'w' && base_angle < MAX_ANGLE) {
                        base_angle += DUTY_STEP * BASE_ANGLE_PER_DUTY;
                    } else if (key == 's' && base_angle > MIN_ANGLE) {
                        base_angle -= DUTY_STEP * BASE_ANGLE_PER_DUTY;
                    } else if (key == 'a' && link_angle < MAX_ANGLE) {
                        link_angle += DUTY_STEP * LINK_ANGLE_PER_DUTY;
                    } else if (key == 'd' && link_angle > MIN_ANGLE) {
                        link_angle -= DUTY_STEP * LINK_ANGLE_PER_DUTY;
                    } else if (key == 'o') {
                        strncpy(gripper_state, GRIPPER_STATE_OPEN, sizeof(gripper_state) - 1U);
                        gripper_state[sizeof(gripper_state) - 1U] = '\0';
                    } else if (key == 'c') {
                        strncpy(gripper_state, GRIPPER_STATE_CLOSED, sizeof(gripper_state) - 1U);
                        gripper_state[sizeof(gripper_state) - 1U] = '\0';
                    } else if (key == 'q') {
                        print_interface(key);
                        usleep(HIGHLIGHT_SLEEP_US);
                        write(serial_fd, "q", 1U);
                        close(serial_fd);
                        restore_terminal(&old_termios);
                        printf("\033[0m\nExiting...\n");
                        return 0;
                    }
                    print_interface(key);
                    break;
                default:
                    break; // Ignore invalid keys
            }
        } else {
            if (key != '\0') {
                print_interface('\0');
                key = '\0';
            }
        }

        usleep(LOOP_SLEEP_US);
    }

    // Unreachable, but for completeness
    close(serial_fd);
    restore_terminal(&old_termios);
    printf("\033[0m");
    return 0;
}