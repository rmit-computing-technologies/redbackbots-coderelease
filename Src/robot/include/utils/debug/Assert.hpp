
/**
 * @file utils/BHAssert.h
 * 
 * This file contains macros for low level debugging
 * 
 * @author Thomas Röfer
 * @author Colin Graf
 * @author RedbackBots
 */

#include "utils/Logger.hpp"

#include <cstdlib>

/**
 * ASSERT prints a message if \c cond is \c false and NDEBUG is not defined.
 * ASSERT does not evaluate \c cond if NDEBUG is defined.
 * @param c The condition to be checked.
 */
#ifdef NDEBUG
#define ASSERT(cond) static_cast<void>(0)
#else
#define ASSERT(cond) static_cast<void>((cond) ? 0 : (llog(ERROR) << __FILE__ << ":" << __LINE__ << " ASSERT(" <<  #cond <<  ") failed", std::abort(), 0))
#endif


/**
 * VERIFY prints a message if \c cond is \c false and NDEBUG is not defined, but does not abort
 * VERIFY does not evaluate \c cond if NDEBUG is defined.
 * @param c The condition to be checked.
 */
#ifdef NDEBUG
#define VERIFY(cond) static_cast<void>(0)
#else
#define VERIFY(cond) static_cast<void>((cond) ? 0 : (llog(WARNING) << __FILE__ << ":" << __LINE__ << " ASSERT(" <<  #cond <<  ") failed", 0))
#endif

