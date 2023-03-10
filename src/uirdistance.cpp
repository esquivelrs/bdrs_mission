#include <string>
#include <string.h>
#include "uirdistance.h"
#include "ubridge.h"


Uirdistance irdistance;

void Uirdistance::setup()
{
    bridge.tx("regbot:irdistance subscribe -1\n");
}

bool Uirdistance::decode(char* msg)
{
    bool used  true;
    cout << msg << endl;
}