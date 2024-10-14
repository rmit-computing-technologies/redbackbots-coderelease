
/**
 * These examples are derived from the following repository:
 * https://github.com/TNG/boost-python-examples
*/

#include <iostream>
#include <string>
#include <thread>

#include <boost/program_options.hpp>
#include <boost/python.hpp>
#include <boost/regex.hpp>
#include <boost/system/error_code.hpp>
#include <boost/thread.hpp>

#include "my_module.hpp"

namespace bpy = boost::python;
namespace bsys = boost::system;
namespace po = boost::program_options;

#define INIT_MODULE PyInit_mymodule
extern "C" PyObject* INIT_MODULE();

void thread() {
    for (int i = 0; i < 3; ++i) {
        std::chrono::seconds duration(1);
        std::this_thread::sleep_for(duration);
        
        std::cout << "Thread iterate: " << i << std::endl;
    }
}

int main(int argc, char *argv[]) {
    std::cout << "Testing Boost Library compilation and availability" << std::endl;
    std::cout << std::endl;

    std::cout << "Testing System Library" << std::endl;
    bsys::error_code code = bsys::errc::make_error_code(boost::system::errc::not_supported);
    std::cout << "- Error code 'Not Supported': " << code.value() << std::endl;
    std::cout << std::endl;

    std::cout << "Testing Program Options" << std::endl;
    po::options_description desc("Program Options for this BoostTest program");
    desc.add_options()
        ("help", "produce help message")
        ("intvalue", po::value<int>(), "test integer value")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    
    if (vm.count("help")) {
        std::cout << desc << std::endl;
    }
    if (vm.count("intvalue")) {
        std::cout << "intvalue set to " << vm["intvalue"].as<int>() << std::endl;
    } else {
        std::cout << "intvalue was not set." << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Testing Regex" << std::endl;
    std::string s = "Boost Libraries";
    boost::regex expr{"\\w+\\s\\w+"};
    std::cout << "Regex match - " << std::boolalpha << boost::regex_match(s, expr) << '\n';
    std::cout << std::endl;

    std::cout << "Testing Python" << std::endl;
    // std::string REDBACKBOTS_CHECKOUT_DIR = std::getenv("REDBACKBOTS_CHECKOUT_DIR");
    // std::string testFile = REDBACKBOTS_CHECKOUT_DIR + "/Src/boost_test/py_boosttest.py";
    std::string testFile = "py_boosttest.py";
    // std::cout << "- REDBACKBOTS_CHECKOUT_DIR: " << REDBACKBOTS_CHECKOUT_DIR << std::endl;
    std::cout << "- testFile: " << testFile << std::endl;
    try {
        PyImport_AppendInittab((char*)"mymodule", INIT_MODULE);
        Py_Initialize();
        std::cout << "Python major version: " << PY_MAJOR_VERSION << std::endl;        

        bpy::object main_module = bpy::import("__main__");
        bpy::dict main_namespace = bpy::extract<bpy::dict>(main_module.attr("__dict__"));
        bpy::object mymodule = bpy::import("mymodule");
        
        main_namespace["precreated_object"] = Base("created on C++ side");
        bpy::exec_file(testFile.c_str(), main_namespace, main_namespace);
    } catch (bpy::error_already_set& e) {
        PyErr_PrintEx(0);
    }
    std::cout << std::endl;

    std::cout << "Testing Thread" << std::endl;
    boost::thread t{thread};
    t.join();
    std::cout << std::endl;

    return 0;
}
