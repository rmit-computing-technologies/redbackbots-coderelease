#pragma once

#include <cstdio>
#include <errno.h>
#include <iostream>

#include "utils/Timer.hpp"

class Speech {
public:
   static Speech &instance();

   void say(const char *text);

private:
   Speech();

   ~Speech();

   // Declare copy constructors privately and don't implement them
   // This is to ensure singleton class
   Speech(Speech const &copy);

   Speech &operator=(Speech const &copy);

   FILE *pipe;
};

const inline void SAY(const std::string text) {
    std::cout<<text<<std::endl;
   // SEE COMMENTS IN speech.cpp
   //(Speech::instance()).say(text.c_str());
}
