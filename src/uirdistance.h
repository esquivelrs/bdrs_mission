#ifndef UIRDISTANCE_H
#define UIRDISTANCE_H

#include <iostream>
#include <sys/time.h>
#include <cstdlib>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

using namespace std;


class UIrdistance{
    public:

    void setup();
        \returns
    bool decode(char * msg)

    public:

     mutex dataLock;
};

extern Uirdistance irdistance;