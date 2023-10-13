#pragma once

#include <string>
#include <ostream>
#include <boost/program_options/variables_map.hpp>

#define llog(X) ((Logger::instance())->realLlog(X))

/**
 * Possible log levels
 * When something is to be logged an associated level is passed with it.
 * Only messages with levels greater than logLevel are actually logged at runtime.
 */
enum LogLevel {
    SILENT  = -100,
    FATAL   = -33,
    ERROR   = 0,
    WARNING = 10,
    INFO    = 20,
    DEBUG   = 50,
    TRACE   = 100
};

class Logger {
public:
    Logger(std::string name);
    virtual ~Logger();

    /**
     * Retrieve logger instance for the current thread
     */
    static Logger *instance();

    /**
     * Read configuration options for the logger from program options
     */
    static void readOptions(const boost::program_options::variables_map &config);

    /**
     * Initialise logger (from previously read or default options)
     */
    static void initialise();

    std::ostream &realLlog(int logLevel);

    // so we can add more files to it
    static std::string getLogDir();

private:
    // Logger instance for current thread
    static thread_local Logger *logger;

    // Global logger configuration options
    static std::string rootLogDirectory;
    static std::string logDirectory;
    static enum LogLevel logLevel;
    static bool logMotion;
    static bool logStdOut;
    
    // Is the logger initialised
    static bool initialised;
    
    // Logging stream for current logger (per thread)
    std::ostream *logStream;
};
