//
// Created by jayen on 12/03/19.
//

#include "types/ButtonPresses.hpp"
#include <ostream>

ButtonPresses::ButtonPresses() : mask(0) {}

bool ButtonPresses::pop(int n) {
   bool ret = peek(n);
   mask &= ~(1 << (n - 1));
   return ret;
}

bool ButtonPresses::peek(int n) const {
   return mask & (1 << (n - 1));
}

void ButtonPresses::push(int n) {
   mask |= 1 << (n - 1);
}

void ButtonPresses::operator|=(const ButtonPresses &b) {
   mask |= b.mask;
}

void ButtonPresses::clear() {
   mask = 0;
}

std::ostream &operator<<(std::ostream &stream, const ButtonPresses &buttonPresses) {
   for (unsigned int i = 1; i <= sizeof(uint8_t) * 8; ++i) {
      if (buttonPresses.peek(i)) {
         stream << i << ", ";
      }
   }
   return stream;
}
