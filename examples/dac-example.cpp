
#include <iostream>
using namespace std;
#include <unistd.h>

#include "max127.h"
using namespace USU;


int main()
{

    Max127 dac127("/dev/i2c-3");

    while(1)
    {

        for (unsigned char i = 0; i<8; ++i)
        {
            cout << (int) i << ":\t" << (int) dac127.readRaw(i) << "\t" << dac127.readVoltage(i) << endl;
        }
        cout << endl << endl;
        usleep(1000000);

    }
    return 0;
}
