// #include <QtCore/qglobal.h>
#include <boost/program_options.hpp>
// #include <boost/shared_ptr.hpp>
#include <QApplication>

#include <iostream>
#include <fstream>

#include "redbackbots.hpp"
#include "visualiser.hpp"
#include "utils/Logger.hpp"
#include "utils/options.hpp"

namespace po = boost::program_options;

po::variables_map config;

int main(int argc, char *argv[]) {
   std::cout << RBB_NAME_CAPS << ": Offnao" << std::endl;

   // Ensure running in offnao mode
   offNao = true;

   /**
    * Read command line options. QT removes its recognized options
    * through the QApplication call. Currently only used for preloading
    * Vision files. If this gets too large, we may want to move this to
    * an Options.cpp helper file
    */
   po::options_description cmdline_options;
   po::options_description generic_options("Generic options");
   generic_options.add_options()
      ("help,h", "produce help message")
      ("dump,d", po::value<std::string>(), "open dump.[ofn|yuv|bbd|ofn2|bbd2]")
      ("host,H", po::value<std::string>(), "connect to redbackbots hostname")
      ("port,p", po::value<uint16_t>()->default_value(10125), "connect to redbackbots port");


   po::options_description vision_options("Vision options");

   cmdline_options.add(generic_options);
   cmdline_options.add(vision_options);

   po::variables_map vm;
   po::store(po::parse_command_line(argc, argv, cmdline_options), vm);
   po::notify(vm);

   if (vm.count("help")) {
      std::cout << cmdline_options << std::endl;
      return 1;
   }

   /** 
    * Load options from config file into 'config' variable 
    */
   po::options_description config_file_options;
   populate_options(config_file_options);

   std::ifstream ifs;
   std::string redbackbosCfgFilename = 
      std::string(getenv("REDBACKBOTS_CHECKOUT_DIR")) + "/Install/NaoHome/config/redbackbots.cfg";
   ifs.open(redbackbosCfgFilename.c_str());
   store(parse_config_file(ifs, config_file_options), config);
   ifs.close();
   po::notify(config);

   // Initialise Logger (so robot code doesn't segfault when logging)
   std::cout << RBB_NAME_CAPS << ": Initialising Logger - read options" << std::endl;
   Logger::readOptions(config);
   std::cout << RBB_NAME_CAPS << ": Initialising Logger" << std::endl;
   Logger::initialise();

   // /** Start the QT application */
   std::cout << RBB_NAME_CAPS << ": Starting Application" << std::endl;
   std::cout << RBB_NAME_CAPS << ": *** Loading of ML Model errors can be ignored - these are not used by OffNao, but loaded by default by Blackboard ***" << std::endl;
   QApplication a(argc, argv);
   Visualiser w;

   if(vm.count("host")) {
      w.connectToNao(vm["host"].as<std::string>().c_str(), vm["port"].as<uint16_t>());
   } else if (vm.count("dump")) {
      w.openFile(vm["dump"].as<std::string>().c_str());
   }

   w.show();

   std::cerr << "Logging Dir: " << config["debug.log.dir"].as<std::string>() << std::endl;

   return a.exec();

   return 0;
}
