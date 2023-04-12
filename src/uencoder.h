#ifndef UENCODER_H
#define UENCODER_H

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


class UEncoder{

public:
    void setup();
    bool decode(char * msg);

public:
    uint32_t e1, e2;
    int32_t e3, e4;
    mutex dataLock;
};

extern UEncoder enc;

#endif