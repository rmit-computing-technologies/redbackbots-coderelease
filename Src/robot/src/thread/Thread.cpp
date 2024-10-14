#include "thread/Thread.hpp"

// const thread_local char* Thread::name = nullptr;
const __thread char* Thread::name = nullptr;
