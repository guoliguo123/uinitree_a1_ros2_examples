#include "keyboard.h"
#include <iostream>
//#include "msg_pub.h"
int kfd = 0;
struct termios cooked, raw;
void KeyBoard::keyboardLoop()
{
    char c;
    bool dirty = false;
    struct pollfd ufd;
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));

    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    ufd.fd = kfd;
    ufd.events = POLLIN;

    while(1)
    {
        boost::this_thread::interruption_point();
	int rcv_num;
	if ((rcv_num = poll(&ufd, 1, 250)) < 0)
	{
	    std::cout << "poll rcv error code:" << rcv_num << std::endl;
	    return;
	}
	else if (rcv_num > 0)
	{
	    if (read(kfd, &c, 1) < 0)
	    {
	        std::cout << "read kfd error" << std::endl;
		return;
	    }
	}
        else
        {
             if (dirty == true)
            {
                pub.pub_clear();
                dirty = false;
            }
            continue;
        }
        dirty = pub.keyboardPressProcess(c);
    }
}


void KeyBoard::exitKeyBoardLoop()
{
    tcsetattr(kfd, TCSANOW, &cooked);
}
