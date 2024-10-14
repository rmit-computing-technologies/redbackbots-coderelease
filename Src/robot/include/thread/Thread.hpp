#pragma once

class Thread {
   public:
      // Record thread name as static const
      // thread_local, C++11, to guarantee thread-duration storage of the static
      // static const thread_local char* name;
      static const __thread char* name;
};

