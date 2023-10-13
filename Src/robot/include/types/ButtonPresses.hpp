#pragma once

#include <stdint.h>
#include <ostream>

class ButtonPresses {
public:
   ButtonPresses();

   bool pop(int n);

   bool peek(int n) const;

   void push(int n);

   void operator|=(const ButtonPresses &b);

   void clear();

private:
   uint8_t mask;
};

std::ostream &operator<<(std::ostream &stream, const ButtonPresses &buttonPresses);
