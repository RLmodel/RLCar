/*
 * Serial.h
 *
 *  Created on: Aug 12, 2015
 *      Author: odroid
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/signal.h>
#include <sys/ioctl.h>
#include <sys/poll.h>

int open_serial(char *dev_name, int baud, int vtime, int vmin);
void close_serial(int fd);
void sendMessage(char* text, int cnt, int fd);


class xSerial
{
private:

public:
    int open_serial(char *dev_name, int baud, int vtime, int vmin);
    void close_serial(int fd);
    void sendMessage(char* text, int cnt, int fd);

};

#endif /* SERIAL_H_ */
