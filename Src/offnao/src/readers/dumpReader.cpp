#include <QDebug>
#include <QString>
#include <QMessageBox>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/VisionBlackboard.hpp"
#include "progopts.hpp"
#include "readers/dumpReader.hpp"
#include "utils/basic_maths.hpp"


DumpReader::DumpReader(QString fileName) {
   dumpFile = fopen(fileName.toLatin1().data(), "r");
   if (dumpFile == NULL) {
      // QMessageBox::warning(this, "Error", "Could not open dump file");
   }
}

DumpReader::DumpReader(QString fileName, const NaoData &naoData) :
Reader(naoData) {
   dumpFile = fopen(fileName.toLatin1().data(), "r");
   if (dumpFile == NULL) {
      // QMessageBox::warning(this, "Error", "Could not open dump file");
   }
}

void DumpReader::run() {

   int currentIndex = 0;
   //yuy2 encoding uses 16 bits per pixel, thus 2x8bits per pixel
   // load whole file into memory  dodgey for now.
   int frameLoaded = 1;
   do {
      uint8_t* top =
            (uint8_t *) malloc(sizeof(uint8_t)*TOP_SIZE);
      uint8_t* bot =
    		(uint8_t *) malloc(sizeof(uint8_t)*BOT_SIZE);
      if(fread(top, TOP_SIZE, 1, dumpFile) != 1){
         free(top);
         break;
      } else {
         if(fread(bot, BOT_SIZE ,1, dumpFile)!= 1){
        	 free(bot);
        	 break;
         } else {
			 Frame frame;
			 Blackboard *blackboard = new Blackboard(config);
			 OffNaoMask_t mask = RAW_IMAGE_MASK;
			 // writeTo(, mask, mask);
			 blackboard->write(&(blackboard->mask), mask);
			 writeTo(vision, topFrame, (const uint8_t*) top);
			 writeTo(vision, botFrame, (const uint8_t*) bot);
			 frame.blackboard = blackboard;
			 naoData.appendFrame(frame);
			 std::stringstream s;
			 s << "Loading frame " << frameLoaded << " from YUV dump.";
			 Q_EMIT showMessage(s.str().c_str(), 2000);
			 frameLoaded++;
         }
      }
   } while (1);
   std::stringstream s;
   s << "Finshed loading YUV dump which consisted of " <<
        frameLoaded << " frames.";
   Q_EMIT showMessage(s.str().c_str(), 5000);
   Q_EMIT newNaoData(&naoData);

   isAlive = true;
   while (isAlive) {
      if (!naoData.getIsPaused() && naoData.getCurrentFrameIndex() <
           naoData.getFramesTotal() - 1) {
         naoData.nextFrame();
         Q_EMIT newNaoData(&naoData);
      } else if (currentIndex != naoData.getCurrentFrameIndex()) {
         Q_EMIT newNaoData(&naoData);
      }
      currentIndex = naoData.getCurrentFrameIndex();
      msleep(500);
   }
   Q_EMIT newNaoData(NULL);
}


void DumpReader::stopMediaTrigger() {
}
