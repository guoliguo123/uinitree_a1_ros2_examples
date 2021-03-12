#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/poll.h>
#include <boost/thread/thread.hpp>
#include "msg_pub.h"
enum SetCmd{
    CMD_SET_MODE_STAND = 0,
    CMD_SET_MODE_FORCE_STAND,
    CMD_SET_MODE_WALK,
    CMD_SET_BODY_HEIGH_UP,
    CMD_SET_BODY_HEIGH_DOWN,
    CMD_SET_YAW_UP,
    CMD_SET_YAW_DOWN,
    CMD_SET_PITCH_UP,
    CMD_SET_PITCH_DOWN,
    CMD_SET_ROLL_LEFT,
    CMD_SET_ROLL_RIGHT,
    CMD_SET_CLEAR_ALL
};

#define KEYCODE_I 0x69
#define KEYCODE_U 0x75
#define KEYCODE_R 0x72
#define KEYCODE_P 0x70
#define KEYCODE_Y 0x79
#define KEYCODE_E 0x65
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_H 0x68
#define KEYCODE_0 0x30
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32

#define KEYCODE_SHIFT_I 0x49
#define KEYCODE_SHIFT_U 0x55
#define KEYCODE_SHIFT_R 0x52
#define KEYCODE_SHIFT_P 0x50
#define KEYCODE_SHIFT_Y 0x59
#define KEYCODE_SHIFT_E 0x45
#define KEYCODE_SHIFT_Q 0x51
#define KEYCODE_SHIFT_H 0x48

#define KEYCODE_SHIFT_W 0x57
#define KEYCODE_SHIFT_A 0x41
#define KEYCODE_SHIFT_S 0x53
#define KEYCODE_SHIFT_D 0x44

class KeyBoard {
public:
    KeyBoard():kfd(0),pub(){

    }
    PubNode pub;
    void keyboardLoop();
    void freeKeyBoard();
    private:
    int kfd;
    struct termios cooked = {0};
    struct termios raw = {0};
    struct pollfd ufd = {0};
};
