
#include "tabs/overviewTab.hpp"

#include <QMenuBar>

#include <utility>
#include <iostream>

#include "renderers/OverlayPainter.hpp"
#include "renderers/overlays/BallOverlay.hpp"
#include "renderers/overlays/FieldBoundaryOverlay.hpp"
#include "renderers/overlays/FieldFeaturesOverlay.hpp"
#include "renderers/overlays/HorizonOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/VisionBlackboard.hpp"

#define DEBUG_IMAGE_COLS      320
#define DEBUG_IMAGE_ROWS      240
// #define DEBUG_IMAGE_COLS      640
// #define DEBUG_IMAGE_ROWS      480

#define NO_IMAGE_DIM          100


OverviewTab::OverviewTab(QTabWidget *parent, QMenuBar *menuBar) : 
   blackboard(nullptr) 
{
   initMenu(menuBar);
   init();
   initOverlays();
   
   this->parent = parent;

   // Do first draw
   redraw();
}

OverviewTab::~OverviewTab() {
   delete topImageRender;
   delete botImageRender;
   delete layout;
}


void OverviewTab::initMenu(QMenuBar * menuBar) {
}

void OverviewTab::init() {
   layout = new QGridLayout(this);
   setLayout(layout);
   layout->setAlignment(layout, Qt::AlignTop);

   layout->setHorizontalSpacing(5);

   layout->addWidget(&fieldView, 0, 0, 2, 1);

   /* draw the field with nothing on it */
   fieldView.redraw(NULL);

   topImageRender = new CameraImageRender(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS, CameraInfo::Camera::top);
   layout->addWidget(topImageRender, 0, 1, 1, 2);

   botImageRender = new CameraImageRender(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS, CameraInfo::Camera::bot);
   layout->addWidget(botImageRender, 1, 1, 1, 2);

   layout->addWidget(&variableView, 0, 3, 2, 2);
}

void OverviewTab::initOverlays() {
   // Top Camera Overlays
   topImageRender->addOverlay(new HorizonOverlay());
   topImageRender->addOverlay(new FieldBoundaryOverlay(CameraInfo::Camera::top));
   topImageRender->addOverlay(new BallOverlay(CameraInfo::Camera::top));
   topImageRender->addOverlay(new FieldFeaturesOverlay(CameraInfo::Camera::top));

   // Bottom Camera Overlays
   botImageRender->addOverlay(new FieldBoundaryOverlay(CameraInfo::Camera::bot));
   botImageRender->addOverlay(new BallOverlay(CameraInfo::Camera::bot));
   botImageRender->addOverlay(new FieldFeaturesOverlay(CameraInfo::Camera::bot));
}

void OverviewTab::redraw() {
   topImageRender->redraw();
   botImageRender->redraw();
}

void OverviewTab::newNaoData(NaoData *naoData) {
   if (naoData != nullptr && naoData->getFramesTotal() != 0) {
      blackboard = (naoData->getCurrentFrame().blackboard);
      topImageRender->setBlackboard(blackboard);
      botImageRender->setBlackboard(blackboard);

      if(blackboard->vision->topImage.width != 0 &&
         blackboard->vision->topImage.height != 0) {
            topCameraImage = blackboard->vision->topImage;
            topImageRender->setCameraImage(blackboard->vision->topImage);
      }
      if(blackboard->vision->botImage.width != 0 &&
         blackboard->vision->botImage.height != 0) {
            botCameraImage = blackboard->vision->botImage;
            botImageRender->setCameraImage(blackboard->vision->botImage);
      }

      if (parent->currentIndex() == parent->indexOf(this)) {
         redraw();
         fieldView.redraw(naoData);
         variableView.redraw(naoData);
      }
   }
}

void OverviewTab::setNao(const QString &naoName) {
   this->naoName = naoName;
}
