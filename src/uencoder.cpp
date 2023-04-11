#include <string>
#include <string.h>
#include "uencoder.h"
#include "ubridge.h"


UEncoder enc;

void UEncoder::setup()
{
    bridge.tx("regbot:enc subscribe -1\n");
    bridge.tx("regbot sub enc 10 \n");

}

bool UEncoder::decode(char* msg)
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

    e1 = strtoll(p1, (char**)&p1, 10);
    e2 = strtoll(p1, (char**)&p1, 10);
    e3 = int32_t(e1);
    e4 = int32_t(e2);
    dataLock.unlock();
  }
  else
    used = false;
  
  
  return used;
}