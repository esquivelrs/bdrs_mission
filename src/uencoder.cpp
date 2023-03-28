#include <string>
#include <string.h>
#include "uencoder.h"
#include "ubridge.h"


UEncoder enc;

void UEncoder::setup()
{
    bridge.tx("regbot:enc subscribe -1\n");
}

bool Uencoder::decode(char* msg)
{
  bool used = true;
  const char * p1 = strchrnul(msg, ':');
  if (strncmp(p1, ":enc ", 5) == 0)
  { // decode pose message
    // advance to first parameter
    if (strlen(p1) > 5)
      p1 += 5;
    else
      return false;
    // get data
    dataLock.lock();
    // time in seconds
    v_unknown = strtof64(p1, (char**)&p1);
    e1 = strtof(p1, (char**)&p1); // x
    e2 = strtof(p1, (char**)&p1); // y
    dataLock.unlock();
  }
  else
    used = false;
  
  
  return used;
}

