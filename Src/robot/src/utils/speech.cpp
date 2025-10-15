//
// Created by jayen on 6/03/19.
//

#include "utils/speech.hpp"

#include "utils/Logger.hpp"

/**
 *  THIS SPEECH INTERFACE IS COMPLETELY BROKEN
 *  IN CONJUNCTION WITH THE ./BIN/SAY.PY PYTHON SCRIPT
 *  NO TIME TO FIX THIS AT THE MOMENT
 */

using namespace std;

Speech &Speech::instance() {
   static Speech instance;
   return instance;
}

FILE *openSay() {
    return NULL;

   const char *command = "/home/nao/bin/say.py";
   FILE       *pipe    = popen(command, "w");
   cerr << "open say" << endl;
   if (!pipe) {
      string command_string = getenv("REDBACKBOTS_CHECKOUT_DIR");
      command_string += "/image";
      command_string += command;

      pipe = popen(command_string.c_str(), "w");
      cerr << "command: " << command_string << endl;
      if (!pipe) {
         llog(ERROR) << "error starting " << command << "\n";
         cerr << "error starting " << command << "\n";
      }
   }
   return pipe;
}

FILE *closeSay(FILE *pipe) {
   return NULL;

   // say.py doesn't always die, even if i explicitly send an EOF
   // may already be fixed by the len(s) check but we keep it anyway
   //cerr << "*** close say" << endl;
   system("/usr/bin/pkill -9 say.py");
   if (pipe) {
      // this waits for say.py to exit, so be sure to kill it first
      pclose(pipe);
   }
   return NULL;
}

void Speech::say(const char *text) {
   // SEE COMMENT ABOVE
   return;


   //if (pipe && (feof(pipe) || ferror(pipe))) {
   //   // not sure why say.py is going defunct - only started happening recently, and we recently fell back to flite
   //   llog(ERROR) << "Restarting say pipe" << endl;
   //   pipe = closeSay(pipe);
   //   pipe = openSay();
   //}
   if (pipe) {
      cerr << "** SAY:\t\t" << text << endl;
      int value = fputs(text, pipe);
      if (value >= 0) {
         value = fputc('\n', pipe);
      }
      char *errorString = strerror(errno);
      llog(ERROR) << errorString << endl;

      char message[1000];
      fgets(message, 1000, pipe);
      cerr << "message: " << message << endl;
   } else {
      //pipe = openSay();
      
      // TW: HACK - for testing purposes
      //std::string command = "/opt/aldebaran/bin/say ";
      //command += text;
      //cerr << "** SAY command:\t\t" << command << endl;
      //system(command.c_str());
   }
}

Speech::Speech() {
   //pipe = openSay();
}

Speech::~Speech() {
   pipe = closeSay(pipe);
}
