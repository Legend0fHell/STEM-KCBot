#include "kmotor.h"

int mdig[] = {7, 8}, manl[] = {6, 3};

Kmotor::Kmotor(bool msg)
{
    _msg = msg;
}

void Kmotor::init()
{
    for (int i = 0; i < 2; i++)
    {
        pinMode(mdig[i], OUTPUT); pinMode(manl[i], OUTPUT);
    }
}

void Kmotor::engine(int cnt, int spd)
{
    digitalWrite(mdig[cnt], bool(spd >= 0));
    analogWrite(manl[cnt], abs(spd));
}

void Kmotor::run(int mode, int spd)
{
    switch(mode)
    {
        case 0: // fwd
            Kmotor::engine(0, spd);
            Kmotor::engine(1, spd);
            break;
        case 1: //bwd
            Kmotor::engine(0, -spd);
            Kmotor::engine(1, -spd);
            break;
        case 2: //turn left
            Kmotor::engine(0, -spd);
            Kmotor::engine(1, spd);
            break;
        case 3: // turn right
            Kmotor::engine(0, spd);
            Kmotor::engine(1, -spd);
    }
}

void Kmotor::stop()
{
    Kmotor::run(0, 0);
}