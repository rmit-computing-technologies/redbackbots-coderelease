#include <utils/Logger.hpp>

#include <boost/iostreams/device/null.hpp>
#include <boost/iostreams/stream.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

#include <thread/Thread.hpp>

Logger::Logger(std::string name) {
   logStream = nullptr;

   if (!initialised) {
      throw std::runtime_error("Logging framework not initialized.");
   } else {
      if (name == "") {
         logStream = &std::cerr;
      } else if (!logMotion && name == "Motion") {
         // Replace motion logstreams with a nullstream, so motion won't make syscalls. 
         // This is done after making an ofstream so the log files still get wiped.
         logStream = new boost::iostreams::stream< boost::iostreams::null_sink >(
               boost::iostreams::null_sink());
      } else if (logStdOut) {
         logStream = &std::cout;
      } else {
         std::string logFile = logDirectory + "/" + name;
         logStream = new std::ofstream(logFile, std::ios_base::out);
      }
   }
}

Logger::~Logger() {
   if (logStream != nullptr 
       && logStream != &std::cout
       && logStream != &std::cerr) {
      delete logStream;
   }
   logStream = NULL;
}

void Logger::initialise() {
   // Generate date string in yyyy-mm-dd-hh-mm-ss format
   auto now = std::chrono::system_clock::now();
   auto nowTimeT = std::chrono::system_clock::to_time_t(now);
   char timestr[20];
   strftime(timestr, 20, "%Y-%m-%d-%H-%M-%S", std::localtime(&nowTimeT));
   logDirectory = rootLogDirectory + "/" + timestr;

   // TODO: Properly exit and fail initialisation on system command errors
   // std::string timestr = std::string(std::put_time(std::localtime(&notTimeT), "%Y-%m-%d-%H-%M-%S"));
   // Create directory
   int sysresult = system((std::string("/bin/mkdir -p ") + logDirectory).c_str());
   if (sysresult != 0) {
      std::cerr << "Failed to create log-directory" << std::endl;
   }

   // Move latest - don't need to really worry about  failure
   sysresult = system((std::string("/bin/ln -sfT ") + logDirectory + " " + logDirectory + "/../latest").c_str());

   // Initialisation done
   initialised = true;
}

void Logger::readOptions(const boost::program_options::variables_map &config) {
   std::string logLevelStr = config["debug.log"].as<std::string>();
   rootLogDirectory = config["debug.log.dir"].as<std::string>();
   logMotion = config["debug.log.motion"].as<bool>();
   logStdOut = config["debug.log.stdout"].as<bool>();

   // Set log level
   std::map<std::string, LogLevel> logLevels;
   logLevels["SILENT"] = SILENT;
   logLevels["FATAL"] = FATAL;
   logLevels["ERROR"] = ERROR;
   logLevels["WARNING"] = WARNING;
   logLevels["INFO"] = INFO;
   logLevels["DEBUG"] = DEBUG;
   logLevels["TRACE"] = TRACE;
   logLevel = logLevels[logLevelStr];
   std::cout<<"Log level is "<<logLevel<<":"<<logLevelStr<<std::endl;
}

std::string Logger::getLogDir() {
   return logDirectory;
}

Logger *Logger::instance() {
   if (logger == NULL) {
      logger = new Logger(Thread::name);
   }
   return logger;
}

class NullBuffer : public std::ostream
{
public:
    NullBuffer() : std::ostream(nullptr) {}
};

std::ostream &Logger::realLlog(int logLevel_) {
    static NullBuffer null;
   if (logLevel >= logLevel_) {
      return *logStream;
   } else {
      return null;
   }
}

// Initialisate Static Variables
thread_local Logger *Logger::logger = NULL;
std::string Logger::rootLogDirectory = "";
std::string Logger::logDirectory = "";
bool Logger::logMotion = false;
bool Logger::logStdOut = false;
bool Logger::initialised = false;
enum LogLevel Logger::logLevel = LogLevel::SILENT;
