
#include "tabs/visionTab.hpp"

#include <QLabel>
#include <QMenuBar>

#include <utility>
#include <iostream>

#include "renderers/OverlayPainter.hpp"
#include "renderers/overlays/BallOverlay.hpp"
#include "renderers/overlays/GridOverlay.hpp"
#include "renderers/overlays/FieldBoundaryOverlay.hpp"
#include "renderers/overlays/FieldBoundaryPointsOverlay.hpp"
#include "renderers/overlays/HorizonOverlay.hpp"
#include "renderers/overlays/FieldFeaturesOverlay.hpp"
#include "renderers/overlays/LineSpotsCandidatesOverlay.hpp"
#include "renderers/overlays/CircleCandidatesOverlay.hpp"
#include "renderers/overlays/LineSpotsOverlay.hpp"
#include "renderers/overlays/PlaneSpotsOverlay.hpp"
#include "renderers/overlays/IsWhiteSpotsOverlay.hpp"
#include "renderers/overlays/RefereeKeypointsOverlay.hpp"
#include "renderers/overlays/RobotOverlay.hpp"
#ifdef TMP_NDEBUG
   
#else
   #include "renderers/overlays/BallSpotsOverlay.hpp"
   #include "renderers/overlays/ScanGridOverlay.hpp"
   #include "renderers/overlays/HorizontalScanLineOverlay.hpp"
   #include "renderers/overlays/VerticalScanLineOverlay.hpp"
   #include "renderers/overlays/PenaltyMarkRegionsOverlay.hpp"
   #include "renderers/overlays/CentreCircleOverlay.hpp"
   #include "renderers/overlays/IntersectionCandidatesOverlay.hpp"
#endif

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/VisionBlackboard.hpp"

#define DEFAULT_IMAGE_COLS    (640/1.5)
#define DEFAULT_IMAGE_ROWS    (480/1.5)


VisionTab::VisionTab(QTabWidget *parent, QMenuBar *menuBar) :
   blackboard(nullptr) 
{
   init();
   
   this->parent = parent;

   // Do first draw
   redraw();
}

VisionTab::~VisionTab() {
   delete topImageRender;
   delete botImageRender;
   delete baseLayout;
}

void VisionTab::init() {
   baseLayout = new QGridLayout(this);
   setLayout(baseLayout);
   baseLayout->setAlignment(Qt::AlignTop);

   baseLayout->setHorizontalSpacing(2);
   baseLayout->setVerticalSpacing(2);

   // Documentation
   // addWidget(QWidget *widget, int fromRow, int fromColumn, int rowSpan, int columnSpan, Qt::Alignment alignment = Qt::Alignment())

   topImageRender = new CameraImageRender(DEFAULT_IMAGE_COLS, DEFAULT_IMAGE_ROWS, CameraInfo::Camera::top);
   topImageRender->setStyleSheet("border: 2px solid black");
   baseLayout->addWidget(topImageRender, 0, 1, 2, 1);

   botImageRender = new CameraImageRender(DEFAULT_IMAGE_COLS, DEFAULT_IMAGE_ROWS, CameraInfo::Camera::bot);
   botImageRender->setStyleSheet("border: 2px solid black");
   baseLayout->addWidget(botImageRender, 0, 3, 2, 1);

   // Setup displayable options
   optionsFrame = new QFrame(this);
   optionsFrame->setLineWidth(1);
   optionsFrame->setFrameShadow(QFrame::Sunken);
   optionsFrame->setFrameShape(QFrame::StyledPanel);
   optionsLayout = new QGridLayout();
   optionsLayout->setAlignment(Qt::AlignTop);
   QLabel* lb = new QLabel(QString("Algorithms/Displays"));
   optionsLayout->addWidget(lb, 0, 0, 1, 1);

   RenderSwitch* rSwitch = nullptr;

   auto addSwitch = [&](std::string text, bool checked,
                        OverlayPainter* opT, OverlayPainter* opB,
                        int rowOffset = 0) {
      rSwitch = new RenderSwitch(text, checked);
      // opT = new HorizonOverlay();
      if (opT != nullptr) {
         topImageRender->addOverlay(opT);
         rSwitch->addOverlay(opT);
      }
      if (opB != nullptr) {
         botImageRender->addOverlay(opB);
         rSwitch->addOverlay(opB);
      }
      checkboxes.push_back(rSwitch);
      optionsLayout->addWidget(rSwitch, checkboxes.size() + rowOffset, 0, 1, 1);
   };

   // Always present vision blackboard information
   addSwitch("Kinematics Horizon", true, 
             new HorizonOverlay(), nullptr, 0);
   addSwitch("Field Boundary Line", true,
             new FieldBoundaryOverlay(CameraInfo::Camera::top),
             new FieldBoundaryOverlay(CameraInfo::Camera::bot),
             0);
   addSwitch("Field Boundary Points", false, 
             new FieldBoundaryPointsOverlay(CameraInfo::Camera::top),
             new FieldBoundaryPointsOverlay(CameraInfo::Camera::bot),
             0);
   addSwitch("Balls", true, 
             new BallOverlay(CameraInfo::Camera::top),
             new BallOverlay(CameraInfo::Camera::bot),
             0);
   addSwitch("Field Features", true, 
             new FieldFeaturesOverlay(CameraInfo::Camera::top),
             new FieldFeaturesOverlay(CameraInfo::Camera::bot),
             0);
   addSwitch("Grid Lines", false, 
             new GridOverlay(CameraInfo::Camera::top),
             new GridOverlay(CameraInfo::Camera::bot),
             0);

   // Only debug present vision blackboard
   QLabel* lvdi = new QLabel(QString("Vision Debug Info"));
   optionsLayout->addWidget(lvdi, checkboxes.size() + 1, 0, 1, 1);
   #ifdef TMP_NDEBUG
      QLabel* lndebug = new QLabel(QString("NOT COMPILED IN DEBUG/DEVELOP"));
      optionsLayout->addWidget(lndebug, checkboxes.size() + 2, 0, 1, 1);
   #else
      addSwitch("ScanGrid", false, 
               new ScanGridOverlay(CameraInfo::Camera::top), 
               new ScanGridOverlay(CameraInfo::Camera::bot),
               1);
      addSwitch("Horizontal Scan Lines", false, 
               new HorizontalScanLineOverlay(CameraInfo::Camera::top),
               new HorizontalScanLineOverlay(CameraInfo::Camera::bot),
               1);
      addSwitch("Vertical Scan Lines", false, 
               new VerticalScanLineOverlay(CameraInfo::Camera::top),
               new VerticalScanLineOverlay(CameraInfo::Camera::bot),
               1);
      addSwitch("Penalty Mark Regions", false, 
               new PenaltyMarkRegionsOverlay(CameraInfo::Camera::top),
               new PenaltyMarkRegionsOverlay(CameraInfo::Camera::bot),
               1);
      addSwitch("BallSpots", false, 
               new BallSpotsOverlay(CameraInfo::Camera::top), 
               new BallSpotsOverlay(CameraInfo::Camera::bot),
               1);
      addSwitch("LineSpots Candidates Final", false,
               new LineSpotsCandidatesOverlay(CameraInfo::Camera::top, topCameraImage.maxResolutionWidth, topCameraImage.maxResolutionHeight, LineSpotsCandidatesOverlay::Type::Final, false),
               new LineSpotsCandidatesOverlay(CameraInfo::Camera::bot, botCameraImage.maxResolutionWidth, botCameraImage.maxResolutionHeight, LineSpotsCandidatesOverlay::Type::Final, false),
               1);
      addSwitch("LineSpots Candidates Before", false,
               new LineSpotsCandidatesOverlay(CameraInfo::Camera::top, topCameraImage.maxResolutionWidth, topCameraImage.maxResolutionHeight, LineSpotsCandidatesOverlay::Type::Before, false),
               new LineSpotsCandidatesOverlay(CameraInfo::Camera::bot, botCameraImage.maxResolutionWidth, botCameraImage.maxResolutionHeight, LineSpotsCandidatesOverlay::Type::Before, false),
               1);
      addSwitch("LineSpots Candidates After", false,
               new LineSpotsCandidatesOverlay(CameraInfo::Camera::top, topCameraImage.maxResolutionWidth, topCameraImage.maxResolutionHeight, LineSpotsCandidatesOverlay::Type::After, false),
               new LineSpotsCandidatesOverlay(CameraInfo::Camera::bot, botCameraImage.maxResolutionWidth, botCameraImage.maxResolutionHeight, LineSpotsCandidatesOverlay::Type::After, false),
               1);
      addSwitch("Circle Candidates", false,
               new CircleCandidatesOverlay(CameraInfo::Camera::top, false),
               new CircleCandidatesOverlay(CameraInfo::Camera::bot, false),
               1);
      addSwitch("Grouped Circle Candidates", false,
               new CircleCandidatesOverlay(CameraInfo::Camera::top, true),
               new CircleCandidatesOverlay(CameraInfo::Camera::bot, true),
               1);
      addSwitch("Intersection Candidates", false, 
               new IntersectionCandidatesOverlay(CameraInfo::Camera::top), 
               new IntersectionCandidatesOverlay(CameraInfo::Camera::bot),
               1);
      addSwitch("Grouped LineSpots Candidates", false,
               new LineSpotsCandidatesOverlay(CameraInfo::Camera::top, topCameraImage.maxResolutionWidth, topCameraImage.maxResolutionHeight, LineSpotsCandidatesOverlay::Type::Final, true),
               new LineSpotsCandidatesOverlay(CameraInfo::Camera::bot, botCameraImage.maxResolutionWidth, botCameraImage.maxResolutionHeight, LineSpotsCandidatesOverlay::Type::Final, true),
               1);
      addSwitch("Lines", false,
               new LineSpotsOverlay(CameraInfo::Camera::top, topCameraImage.maxResolutionWidth, topCameraImage.maxResolutionHeight),
               new LineSpotsOverlay(CameraInfo::Camera::bot, botCameraImage.maxResolutionWidth, botCameraImage.maxResolutionHeight),
               1);
      addSwitch("Plane", false,
               new PlaneSpotsOverlay(CameraInfo::Camera::top, topCameraImage.maxResolutionWidth, topCameraImage.maxResolutionHeight),
               new PlaneSpotsOverlay(CameraInfo::Camera::bot, botCameraImage.maxResolutionWidth, botCameraImage.maxResolutionHeight),
               1);
      addSwitch("Centre Circle", false,
               new CentreCircleOverlay(CameraInfo::Camera::top, topCameraImage.maxResolutionWidth, topCameraImage.maxResolutionHeight),
               new CentreCircleOverlay(CameraInfo::Camera::bot, topCameraImage.maxResolutionWidth, topCameraImage.maxResolutionHeight),
               1);
      addSwitch("Is White Spots", false,
               new IsWhiteSpotsOverlay(CameraInfo::Camera::top),
               new IsWhiteSpotsOverlay(CameraInfo::Camera::bot),
               1);
      addSwitch("Robots", false, 
               new RobotOverlay(CameraInfo::Camera::top), 
               new RobotOverlay(CameraInfo::Camera::bot),
               1);
      addSwitch("Referee Keypoints", false, 
               new RefereeKeypointsOverlay(CameraInfo::Camera::top), // only in top camera
               nullptr,
               1);
   #endif

   optionsFrame->setLayout(optionsLayout);
   baseLayout->addWidget(optionsFrame, 0, 5, 1, 1);
}

void VisionTab::redraw() {
   topImageRender->redraw();
   botImageRender->redraw();
}

void VisionTab::newNaoData(NaoData *naoData) {
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
      }
   }
}

RenderSwitch::RenderSwitch(std::string label, bool checked) :
   QCheckBox(QString(label.c_str()))
{
   if (checked) {
      this->setCheckState(Qt::Checked);
   } else {
      this->setCheckState(Qt::Unchecked);
   }

   connect(this, SIGNAL(clicked(bool)), this, SLOT(toggleState(bool)));
}

RenderSwitch::~RenderSwitch() {
}

void RenderSwitch::toggleState(bool enable) {
	for(OverlayPainter* op: overlays) {
      op->setEnabled(enable);
   }
}

void RenderSwitch::addOverlay(OverlayPainter* overlay) {
   overlays.push_back(overlay);

   overlay->setEnabled(this->checkState() == Qt::Checked);
}