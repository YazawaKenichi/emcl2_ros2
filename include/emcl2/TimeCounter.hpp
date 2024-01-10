#ifndef __TIMECOUNTER_HPP__
#define __TIMECOUNTER_HPP__

#include <iostream>
#include <string>
#include <fstream>

namespace TimeCounter
{
    class TimeCounter
    {
        public:
            TimeCounter(std::string, std::string);
            void startCounter();
            void stopCounter();
            void printf(std::string);
            void writef(std::string);
            std::string path;
            std::string node;
            clock_t start;
            clock_t stop;
        private:
    };
}

#endif

