#ifndef kmotor_h
#define kmotor_H
#include <Arduino.h>
class Kmotor
{
    public:
        Kmotor(bool msg);
        void init();
        void engine(int cnt, int spd);
        void run(int mode, int spd);
        void stop();
    private:
        bool _msg;
};
#endif
