#include "tabs/cameraTab.hpp"

#include <QCheckBox>
#include <QDebug>
#include <QFileDialog>
#include <QMenu>
#include <QMenuBar>
#include <QMouseEvent>
#include <QPainter>

#include <algorithm>
#include <iostream>
#include <unistd.h>

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/VisionBlackboard.hpp"
#include "naoData.hpp"
#include "perception/vision/other/YUV.hpp"
#include "tabs/yuvHistogram.hpp"
#include "utils/classifier.hpp"
#include "renderers/OverlayPainter.hpp"


#define PIXMAP_ROWS           (480 / 1)
#define PIXMAP_COLS           (640 / 1)

// There are 12 controls that we care about setting
#define NUM_CONTROLS 12
const int NUM_CAMERAS = 2;

enum ControlType {
   TOGGLE,
   NUMBER
};

// For manual tuning
static std::string controlNames[NUM_CONTROLS] = {
   "Horizontal Flip",
   "Vertical Flip",
   "Brightness",
   "Contrast",
   "Saturation",
   "Hue",
   "Sharpness",
   "Auto Exposure",
   "Exposure",
   "Gain",
   "Auto White Balance",
   "White Balance",
};

static ControlType controlTypes[NUM_CONTROLS] = {
	TOGGLE, // Hflip
	TOGGLE, // Vflip
	NUMBER, // Brightness
	NUMBER, // Contrast
	NUMBER, // Saturation
   NUMBER, // Hue
	NUMBER, // Sharpness
	TOGGLE, // Auto Exposure
	NUMBER, // Exposure
	NUMBER, // Gain
   TOGGLE, // Auto WB
	NUMBER  // White Balance
};

// Index of the control's parent (toggle) which enables/disables this control
// Number is the order of the checkbox (which looks up checkInputBoxes)
// (-1 otherwise)
static int parentControl[NUM_CONTROLS] = {
	-1, // Hflip
	-1, // Vflip
	-1, // Brightness
	-1, // Contrast
	-1, // Saturation
   -1, // Hue
	-1, // Sharpness
	-1, // Auto Exposure
	2,  // Exposure
	2,  // Gain
   -1, // Auto WB
	3  // White Balance
};

static int controlMin[NUM_CONTROLS] = {
	0, // Hflip
	0, // Vflip
	-255, // Brightness
	0, // Contrast
	0, // Saturation
	-180, // Hue
	0, // Sharpness
	0, // Auto Exposure
	0, // Exposure
	0, // Gain
   0, // Auto WB
	0  // White Balance
};

static int controlMax[NUM_CONTROLS] = {
	1, // Hflip
	1, // Vflip
	255, // Brightness
	255, // Contrast
	255, // Saturation
   180, // Hue
	9, // Sharpness
	1,  // Auto Exposure
	100000, // Exposure
	1023, // Gain
   1, // Auto WB
	6500 // White Balance
};


CameraTab::CameraTab(QTabWidget *parent, QMenuBar *menuBar)  {
   currentSettings = CameraSettings();
   topCameraSettings = CameraSettings();
   botCameraSettings = CameraSettings();
   
   init();
   
   this->parent = parent;
   framesSinceReset = INT_MAX;

   resetCameraImages();

   numLocalMax = 0;

   // Do first draw
   redraw();
}

void CameraTab::init() {
   int i = 0;

   // Lay them all out
   layout = new QGridLayout();
   this->setLayout(layout);

   // Camera image render
   imageRender = new CameraImageRender(PIXMAP_COLS, PIXMAP_ROWS, CameraInfo::Camera::top);
   layout->addWidget(imageRender, 0, 0, 1, 1);

   // Set up pixmaps for camera image and RGB, YUV, HSV histograms
   for (i = 0; i < NUM_IMG_TYPES; i++) {
      // Create pixmap and label
      imagePixmaps[i] = QPixmap(320, 240);
      imagePixmaps[i].fill(Qt::darkGray);
      imageLabels[i] = new QLabel();
      imageLabels[i]->setPixmap(imagePixmaps[i]);

      // Set alignment and size of pixmaps
      imageLabels[i]->setAlignment(Qt::AlignTop);
      imageLabels[i]->setMinimumSize(PIXMAP_COLS, PIXMAP_ROWS);
      imageLabels[i]->setMaximumSize(PIXMAP_COLS, PIXMAP_ROWS);
   }

   // Position image and histogram pixmaps
   //layout->addWidget(imageLabels[RGB], 1,0,1,1); // RGB histogram below camera
   layout->addWidget(&pointCloud,      1,0,1,1);   // Point cloud to bottom left
   layout->addWidget(imageLabels[YUV], 0,1,1,1); // YUV histogram to right
   //layout->addWidget(imageLabels[HSV], 1,1,1,1); // HSV histogram below YUV

   // Add controls
   QVBoxLayout *controlLayout = new QVBoxLayout();

   // Camera selector
   QGroupBox *cameraBox = new QGroupBox(tr("Camera"));
   QVBoxLayout *cameraGroupLayout = new QVBoxLayout();
   cameraBox->setLayout(cameraGroupLayout);
   controlLayout->addWidget(cameraBox);
   cameraGroupLayout->setAlignment(Qt::AlignTop);

   topCam = new QRadioButton(QString("Top Camera"), cameraBox);
   bottomCam = new QRadioButton(QString("Bottom Camera"), cameraBox);
   topCam->setChecked(true);
   cameraGroupLayout->addWidget(topCam);
   cameraGroupLayout->addWidget(bottomCam);

   connect(topCam, SIGNAL(toggled(bool)), this, SLOT(toggleChange(bool)));
   connect(bottomCam, SIGNAL(toggled(bool)), this, SLOT(toggleChange(bool)));

   // Camera controls
   QGroupBox *controlsBox = new QGroupBox(tr("Camera Controls"));
   QGridLayout *controlsGroupLayout = new QGridLayout();
   controlsBox->setLayout(controlsGroupLayout);
   controlLayout->addWidget(controlsBox);

   checkInputBoxes.clear();
   valueInputBoxes.clear();
   for (i = 0; i != NUM_CONTROLS; ++i) {
      if (controlTypes[i] == ControlType::TOGGLE) {
         QLabel *nameLabel = new QLabel(controlNames[i].c_str());
         controlsGroupLayout->addWidget(nameLabel, i * 6, 0, 3, 1);
         QCheckBox *checkbox = new QCheckBox(this);
         controlsGroupLayout->addWidget(checkbox, i * 6, 3, 3, 1);
         checkInputBoxes.push_back(checkbox);
         
         // Setup handler for this checkbox - only setup here if not a parent of something else
         // If a parent, then setup the checkbox later
         bool isParent = false;
         for (int j = 0; j != NUM_CONTROLS; ++j) {
            if (parentControl[j] == i) {
               isParent = true;
            }
         }
         if (!isParent) {
            connect(checkbox, 
                  SIGNAL(clicked(bool)),
                  new CheckboxHandler(this, nullptr, nullptr, nullptr, nullptr, i),
                  SLOT(toggleState(bool))
            );
         }
      } else if (controlTypes[i] == ControlType::NUMBER) {
         QLabel *nameLabel = new QLabel(controlNames[i].c_str());
         QPushButton *decButton = new QPushButton("-");
         decButton->setMaximumSize(30,30);
         QLineEdit *currentValue = new QLineEdit(QString::number(0));
         QPushButton *incButton = new QPushButton("+");
         incButton->setMaximumSize(30,30);
         QSlider *slider = new QSlider(Qt::Horizontal,this);
         slider->setMaximum(controlMax[i]);
         slider->setMinimum(controlMin[i]);
         slider->setTracking(true);
         slider->setSliderPosition(0);
         CameraButtonHandler* cbh = new CameraButtonHandler(this, i, NULL, currentValue, slider);
         valueInputBoxes.push_back(cbh);
         connect(decButton, SIGNAL(clicked(bool)),
                 new CameraButtonHandler(this, i, decButton, currentValue, slider),
                 SLOT(clicked(bool)));
         connect(incButton, SIGNAL(clicked(bool)),
                 new CameraButtonHandler(this, i, incButton, currentValue, slider),
                 SLOT(clicked(bool)));
         connect(currentValue, SIGNAL(editingFinished()),
                 cbh,
                 SLOT(directUpdate()));
         connect(slider, SIGNAL(valueChanged(int)),
                 new SliderHandler(this, i, slider, currentValue),
                 SLOT(setValue(int)));
         controlsGroupLayout->addWidget(nameLabel, i * 6, 0, 3, 1);
         controlsGroupLayout->addWidget(decButton, i * 6, 2, 3, 1);
         controlsGroupLayout->addWidget(currentValue, i * 6, 3, 3, 1);
         controlsGroupLayout->addWidget(incButton, i * 6, 4, 3, 1);
         controlsGroupLayout->addWidget(slider,(i*6)+3,0,3,5);

         // Set parent toggle
         if (parentControl[i] != -1) {
            QCheckBox *pcCheckbox = checkInputBoxes[parentControl[i]];
            connect(pcCheckbox, 
                    SIGNAL(clicked(bool)),
                    new CheckboxHandler(this, slider, decButton, currentValue, incButton, parentControl[i]),
                    SLOT(toggleState(bool))
            );
         }
      }
   }
   i++;
   i++;

   // Add load values button
   QPushButton *loadValuesButton = new QPushButton("Load Values");
   controlsGroupLayout->addWidget(loadValuesButton, i * 6, 0, 4, 1);
   connect(loadValuesButton, SIGNAL(clicked(bool)), this, SLOT(fetchValues()));

   // Add send values button
   QPushButton *sendValuesButton = new QPushButton("Send Values");
   controlsGroupLayout->addWidget(sendValuesButton, i * 6, 1, 4, 1);
   connect(sendValuesButton, SIGNAL(clicked(bool)), this, SLOT(sendValues()));

   // Add auto tuning button
   // QPushButton *autoTuneButton = new QPushButton("Auto Tune");
   // controlsGroupLayout->addWidget(autoTuneButton, i * 6, 2, 4, 1);
   // connect(autoTuneButton, SIGNAL(clicked(bool)), this, SLOT(autoTuner()));

   layout->addLayout(controlLayout, 0, 2, 2, 1);

   // Initialise variables
   bestEntropy = 0.0;
   globalBestEntropy = 0.0;
   autoTuneOn = false;
   cameraToggled = false;
}

void CameraTab::resetCameraImages() {
   topCameraImage.setResolution(CameraImageRender::NO_IMAGE_DIM, CameraImageRender::NO_IMAGE_DIM);
   botCameraImage.setResolution(CameraImageRender::NO_IMAGE_DIM, CameraImageRender::NO_IMAGE_DIM);

   // TODO TW: Reset image render camera - needs method
}

void CameraTab::redraw() {
   // QImage qImage;
   CameraImage *cImage = nullptr;

   unsigned int imageRows, imageCols;
   const uint8_t* frame;

   if (topCam->isChecked()) {
      // qImage = QImage(topCameraImage.width * 2, topCameraImage.height, QImage::Format_RGB32);
      cImage = &topCameraImage;
   } else {
      // qImage = QImage(botCameraImage.width * 2, botCameraImage.height, QImage::Format_RGB32);
      cImage = &botCameraImage;
   }

   // Reset the point cloud
   pointCloud.points.resize(0);

   // Draw selected image
   imageRender->setCameraImage(*cImage);
   imageRender->redraw();

   // Generate Point Cloud
   PixelTypes::YUVPixel yuv;
   QRgb rgb;
   for (int row = 0; row < cImage->height; ++row) {
      for (int col = 0; col < cImage->width * 2; ++col) {
         yuv = cImage->getYUV(col, row);
         rgb = Classifier::yuv2rgb(yuv.y,yuv.u,yuv.v);
         int r, g, b;
         QColor::fromRgb(rgb).getRgb(&r, &g, &b);
         // llog(INFO) << "rbg: " << r << " " << g << " " << b << std::endl;
         // llog(INFO) << "\tyuv: " << (int)(yuv.y) << " " << (int)(yuv.u) << " " << (int)(yuv.v) << std::endl;
         pointCloud.points.push_back(
            std::make_pair(qglviewer::Vec(r / 255.0, g / 255.0, b / 255.0),
                           qglviewer::Vec(yuv.y / 255.0, yuv.u / 255.0, yuv.v / 255.0))
         );
      }
   }
   // llog(INFO) << "Point cloud updated: " << pointCloud.points.size() << std::endl;

   // If we are auto-tuning or have switched cameras, update the parameters
   if (autoTuneOn || cameraToggled) {
      updateVisualsFromCS();
      cameraToggled = false;
   }

   // Create histograms
   //Histogram RGBHistogram(0x00000000U, 0x00ffffffU);
   YUVHistogram yuvHistogram(0x00000000U, 0x00ffffffU);

   // Process image
   for (unsigned int row = 0; row < imageRows; ++row) {
      for (unsigned int col = 0; col < imageCols; ++col) {
         unsigned int YUVValue = 0x00ffffff &
                                 ((gety(frame, row, col, imageCols) << 16) |
                                  (getu(frame, row, col, imageCols) << 8) |
                                   getv(frame, row, col, imageCols));

         yuvHistogram.addDatapoint(YUVValue);
      }
   }

   // Set initial background of pixmaps
   imagePixmaps[YUV].fill(Qt::darkGray);
   // Draw Histogram
   yuvHistogram.drawHistogram(imagePixmaps[YUV], (unsigned int)(TOP_IMAGE_COLS/2), (unsigned int)(TOP_IMAGE_ROWS/2), YUVHistogram::eYUV);
   // Display Histograph in tab
   imageLabels[YUV]->setPixmap(imagePixmaps[YUV]);

   if (autoTuneOn) {
      //tuneValue(yuvHistogram.getEntropyValue());
      tuneValue(getEntropyValue());
   }
}

void CameraTab::newNaoData(NaoData *naoData) {
   if (!naoData || !naoData->getCurrentFrame().blackboard) {
      resetCameraImages();
   } else {
      Blackboard *blackboard = naoData->getCurrentFrame().blackboard;
      topCameraSettings = readFrom(vision, topCameraSettings);
      botCameraSettings = readFrom(vision, botCameraSettings);

      if(blackboard->vision->topImage.width != 0 &&
         blackboard->vision->topImage.height != 0) {
            topCameraImage = blackboard->vision->topImage;
      }
      if(blackboard->vision->botImage.width != 0 &&
         blackboard->vision->botImage.height != 0) {
            botCameraImage = blackboard->vision->botImage;
      }
      redraw();
   }
}

// Called when user has switched cameras
void CameraTab::toggleChange(bool active) {
   cameraToggled = true;
   
   // Both radio buttons activate the toggleChange.
   // Only update once, on the "activated" button
   if (active) {
      updateVisualsFromCS();
   }
}

// Update camera control values in OffNao to reflect robot values
void CameraTab::updateVisualsFromCS() {
	/** This is used for autotuning
   for (std::vector<CameraButtonHandler*>::iterator it = valueInputBoxes.begin(); it != valueInputBoxes.end(); ++it) {
      CameraButtonHandler *cbh = *it;
      int controlIndex = cbh->getControl();
      int value = getCSParamValue(controlNames[controlIndex]);
      cbh->setValue(value);
   }
   **/

   // llog(INFO) << "Updating sliders with current setting values from blackboard" << std::endl;
	if (topCam->isChecked()) {
		currentSettings = topCameraSettings;
	} else {
		currentSettings = botCameraSettings;
	}

   // Update checkboxes
   for(int i = 0; i != checkInputBoxes.size(); ++i) {
      QCheckBox* checkBox = checkInputBoxes[i];
      if (i == 0) {
         // HFlip
         checkBox->setChecked(currentSettings.hflip == 1);
      } else if (i == 1) {
         // VFlip
         checkBox->setChecked(currentSettings.vflip == 1);
      } else if (i == 2) {
         // Auto Exposure
         checkBox->setChecked(currentSettings.settings[CameraSettings::CameraSetting::AUTO_EXPOSURE] == 1);
      } else if (i == 3) {
         // Auto White Balance
         checkBox->setChecked(currentSettings.settings[CameraSettings::CameraSetting::AUTO_WHITE_BALANCE] == 1);
      }
   }

   // Update Values
	int i = 0;
	for (std::vector<CameraButtonHandler*>::iterator it = valueInputBoxes.begin(); it != valueInputBoxes.end(); ++it) {
	    CameraButtonHandler *cbh = *it;
		switch(i) {
	       //brightness
		  case 0:
           cbh->setValue(currentSettings.settings[CameraSettings::CameraSetting::BRIGHTNESS]);
			  break;
		  //contrast
		  case 1:
			  cbh->setValue(currentSettings.settings[CameraSettings::CameraSetting::CONTRAST]);
			  break;
	      //saturation
		  case 2:
			  cbh->setValue(currentSettings.settings[CameraSettings::CameraSetting::SATURATION]);
			  break;
		  //hue
		  case 3:
			  cbh->setValue(currentSettings.settings[CameraSettings::CameraSetting::HUE]);
			  break;
		  //sharpness
		  case 4:
			  cbh->setValue(currentSettings.settings[CameraSettings::CameraSetting::SHARPNESS]);
			  break;
	      //exposure
		  case 5:
			  cbh->setValue(currentSettings.settings[CameraSettings::CameraSetting::EXPOSURE]);
			  break;
		  //gain
		  case 6:
			  cbh->setValue(currentSettings.settings[CameraSettings::CameraSetting::GAIN]);
			  break;
        //white bal
		  case 7:
			  cbh->setValue(currentSettings.whiteBalance);
			  break;
	   }
	   i++;
	}
}

// SLOT for loading values from Nao packet
void CameraTab::fetchValues(void) {
   updateVisualsFromCS();
}

void CameraTab::updateCSfromVisual() {
   // llog(INFO) << "updateCSfromVisual()" << std::endl;

   // Update checkboxes
   for(int i = 0; i != checkInputBoxes.size(); ++i) {
      QCheckBox* checkBox = checkInputBoxes[i];
      if (i == 0) {
         // HFlip
         currentSettings.hflip = checkBox->isChecked() ? 1 : 0;
      } else if (i == 1) {
         // VFlip
         currentSettings.vflip = checkBox->isChecked() ? 1 : 0;
      } else if (i == 2) {
         // Auto Exposure
         currentSettings.settings[CameraSettings::CameraSetting::AUTO_EXPOSURE] = checkBox->isChecked() ? 1 : 0;
      } else if (i == 3) {
         // Auto White Balance
         currentSettings.settings[CameraSettings::CameraSetting::AUTO_WHITE_BALANCE] = checkBox->isChecked() ? 1 : 0;
      }
   }

   // Update Values
	int i = 0;
	for (std::vector<CameraButtonHandler*>::iterator it = valueInputBoxes.begin(); it != valueInputBoxes.end(); ++it) {
	    CameraButtonHandler *cbh = *it;
		switch(i) {
	       //brightness
		  case 0:
           currentSettings.settings[CameraSettings::CameraSetting::BRIGHTNESS] = cbh->getValue();
			  break;
		  //contrast
		  case 1:
			  currentSettings.settings[CameraSettings::CameraSetting::CONTRAST] = cbh->getValue();
			  break;
	      //saturation
		  case 2:
			  currentSettings.settings[CameraSettings::CameraSetting::SATURATION] = cbh->getValue();
			  break;
		  //hue
		  case 3:
			  currentSettings.settings[CameraSettings::CameraSetting::HUE] = cbh->getValue();
			  break;
		  //sharpness
		  case 4:
			  currentSettings.settings[CameraSettings::CameraSetting::SHARPNESS] = cbh->getValue();
			  break;
	      //exposure
		  case 5:
			  currentSettings.settings[CameraSettings::CameraSetting::EXPOSURE] = cbh->getValue();
			  break;
		  //gain
		  case 6:
			  currentSettings.settings[CameraSettings::CameraSetting::GAIN] = cbh->getValue();
			  break;
        //white bal
		  case 7:
			  currentSettings.whiteBalance = cbh->getValue();
			  break;
	   }
	   ++i;
	}
}

void CameraTab::sendValues(void) {
   CameraInfo::Camera whichCamera = CameraInfo::Camera::top;
   if (bottomCam->isChecked()) {
		whichCamera = CameraInfo::Camera::bot;
	}
   llog(INFO) << "Sending current settings for Camera: " << CameraInfo::enumCameraToString(whichCamera) << std::endl;

   // Actual shared object to send
   QSharedPointer<CameraSettings> newSettings =  QSharedPointer<CameraSettings>(new CameraSettings);

   // Start with current settings
   *newSettings = currentSettings;

   // Output for testing
   // llog(INFO) << "\t hflip = " << newSettings->hflip << std::endl;
   // llog(INFO) << "\t vflip = " << newSettings->vflip << std::endl;
   // for (int name = 0; name != CameraSettings::CameraSetting::NUM_CAMERA_SETTINGS; ++name) {
   //    llog(INFO) << "\t " << CameraSettings::enumToString((CameraSettings::CameraSetting) name) << " = " 
   //               << newSettings->settings[name] << std::endl;
   // }
   // llog(INFO) << "\t whitebalance = " << newSettings->whiteBalance << std::endl;

   // Send - using a pointer is ok if the emitter and receiver are in the same thread
   Q_EMIT sendCameraSettingsToRobot(whichCamera, newSettings);
}

CameraButtonHandler::CameraButtonHandler(CameraTab *camtab, int control,
                                          QPushButton *qpb, QLineEdit *qle, QSlider *sl) :
      camtab(camtab), 
      control(control), 
      qpb(qpb), 
      qle(qle), 
      sl(sl) {
}

SliderHandler::SliderHandler(CameraTab *camtab, int control, QSlider *sl, QLineEdit *qle) :
	  camtab(camtab), 
     control(control), 
     sl(sl), 
     qle(qle) {
}

CheckboxHandler::CheckboxHandler(CameraTab *camtab, QSlider *sl, QPushButton *db, QLineEdit *qle,
                                 QPushButton *ib, int control):
		camtab(camtab),
      sl(sl),
      db(db),
      qle(qle),
      ib(ib),
      control(control) {
}

void CheckboxHandler::toggleState(bool enable) {
	// Toggle things which this checkbox enables/disables
   if (sl != nullptr) {
		sl->setEnabled(!enable);
   }
   if (db != nullptr) {
		db->setEnabled(!enable);
   }
   if (qle != nullptr) {
		qle->setEnabled(!enable);
   }
   if (ib != nullptr) {
		ib->setEnabled(!enable);
	}

	camtab->updateCSfromVisual();
}

void SliderHandler::setValue(int newValue) {
   qle->setText(QString::number(newValue));
   camtab->updateCSfromVisual();
}

void CameraButtonHandler::setValue(unsigned int val) {
   qle->setText(QString::number(val));
}

unsigned int CameraButtonHandler::getValue() {
   return qle->text().toUInt();
}

int CameraButtonHandler::getControl() const {
   return control;
}

void CameraButtonHandler::directUpdate() {
   int newValue = qle->text().toUInt();
   newValue = std::min(newValue, controlMax[control]);
   newValue = std::max(controlMin[control], newValue);
   sl->setSliderPosition(newValue);

   camtab->updateCSfromVisual();
}

void CameraButtonHandler::clicked(bool click) {
   int diff = qpb ? (qpb->text() == "-" ? -1 : 1) : 0;
   int newval = qle->text().toInt() + diff;
   if (newval >= controlMin[control] && newval <= controlMax[control]){
	   qle->setText(QString::number(newval));
	   sl->setSliderPosition(newval);

      camtab->updateCSfromVisual();
   }
}









//// OLD AUTO TUNER CODE
/*

// extern __u32 controlIds[NUM_CONTROLS];
// extern __s32 (*controlValues)[NUM_CONTROLS];
// __s32 controlValues_lights[NUM_CAMERAS][NUM_CONTROLS] =
// { {1, 1,
//    1,
//    248, 60,
//    130, 0,
//    2,
//    1, 0x00,
//    0,
//    50, 250,
//    2800},
//   {0, 1,
//    1,
//    250, 64,
//    13, 0,
//    -55,
//    0, 0x00,
//    248,
//    60, 180,
//    0}
// };
// __s32 (*controlValues)[NUM_CONTROLS] = controlValues_lights;

struct controlAttributes {
    __u32 control;
    std::string name;
    __s32 min;
    __s32 max;
    __s32 current;
    bool tryIncrease;
};

// For autotuning
static struct controlAttributes ControlValues[] = {
   // Note: Exposure can be set upto 2500 (manually), but for auto-tuning, we limit it to 140
    {V4L2_CID_EXPOSURE,                "Exposure",                  1,  140, 0, false},
    {V4L2_CID_BRIGHTNESS,              "Brightness",                0,  255, 0, false},
    {V4L2_CID_CONTRAST,                "Contrast",                 16,   64, 0, false},
    {V4L2_CID_GAIN,                    "Gain",                     35,  255, 0, false},
    // Note: Saturation ranges from 0 - 255, but anything below 60 is losing its colour
    {V4L2_CID_SATURATION,              "Saturation",               60,  255, 0, false},
    {V4L2_CID_SHARPNESS,               "Sharpness",                 0,    7, 0, false},
    {V4L2_CID_BACKLIGHT_COMPENSATION,  "Backlight Compensation",    0,    4, 0, false},
    {V4L2_CID_DO_WHITE_BALANCE,        "White Balance",          2700, 6500, 0, false},
};

static const int cNUM_CONTROL_ATTRIBUTES = sizeof(ControlValues)/sizeof(struct controlAttributes);

void CameraTab::autoTuner(void) {
   if (autoTuneOn) { // Then turn it off
      qDebug() << "Autotuning off!";
      numLocalMax = 0;
   } else { // Otherwise turn it on
      qDebug() << "Autotuning on!";
      // Reset best entropy value
      bestEntropy = 0.0;

      // Set the values we begin tuning from
      for (int i = 0; i < cNUM_CONTROL_ATTRIBUTES; i++) {
         ControlValues[i].current = getCSParamValue(ControlValues[i].name.c_str());
      }
   }
   autoTuneOn = !autoTuneOn;
   srandom(time(NULL)); // (Re)Seed random number generator
   framesSinceReset = 0;
}

void CameraTab::tuneValue(float entropy) {
   std::vector<struct v4l2_control> vals;
   static unsigned int iterations = 0;
   static int control = -1;
   static unsigned int temperature = 12;
   static unsigned int iterationsSinceChange = 0;

   // wait for new frame with updated image
   if (framesSinceReset > 5) {
      qDebug() << "Pre change: ";
      printCameraControlValues();

      // Get new entropy value
      if (entropy < bestEntropy) {
          // If new entropy value is worse, revert back to previous attribute value for control
          struct v4l2_control val = {ControlValues[prevControl].control, prevControlValue};
          vals.push_back(val);
          // Next time we try this control, try the other direction
          ControlValues[prevControl].tryIncrease = !ControlValues[prevControl].tryIncrease;
          ControlValues[prevControl].current = prevControlValue;
          qDebug() << "Decreased entropy: " << entropy;
          printCameraControlValues();
      } else {
          bestEntropy = entropy;
          iterationsSinceChange = 0;
          qDebug() << "Increased entropy: " << entropy;
          printCameraControlValues();
      }

      // If we have reached one full iteration of the controls
      if ((iterations % cNUM_CONTROL_ATTRIBUTES) == 0) {
         iterationsSinceChange++;
         if (iterationsSinceChange == 2) {
            // If this local max is better than our global max, set values
            if (bestEntropy > globalBestEntropy) {
               qDebug()  << "Better set of settings!";
               globalBestEntropy = bestEntropy;
               globalBestSettings = currentSettings;
            }

            // TODO: turn this back to < 2
            // Restart auto-tuning if we haven't found max of 3 local maxima
            if (numLocalMax < 1) {
               numLocalMax++;
               qDebug() << "Restarting tuning #" << numLocalMax;
               // Turn off auto-tuner and start it again
               autoTuneOn = false;

               // Reset all values randomly
               std::vector<struct v4l2_control> newVals;
               for (int i = 0; i < cNUM_CONTROL_ATTRIBUTES; i++) {
                  int value = ControlValues[i].min + (random() % ((ControlValues[i].max - ControlValues[i].min)/2));
                  ControlValues[i].current = value;
                  struct v4l2_control val = {ControlValues[i].control, value};
                  newVals.push_back(val);
               }

               // Send these values to the robot
               setControls(newVals);

               // Toggle autoTuner
               autoTuneOn = true;
               bestEntropy = 0.0;
               iterationsSinceChange = 0;
               srandom(time(NULL)); // (Re)Seed random number generator
               framesSinceReset = 0;
               return;
            } else {
               llog(INFO) << "autotune - Reached 2 local max" << std::endl;
               llog(INFO) << "Turning autotuning off" << std::endl;
               autoTuneOn = !autoTuneOn;

               // Revert to our best settings
               std::vector<struct v4l2_control> bestVals;
               for (int i = 0; i < cNUM_CONTROL_ATTRIBUTES; i++) {
                  struct v4l2_control val = {ControlValues[i].control, getBestSettingsValue(ControlValues[i].control)};
                  bestVals.push_back(val);
               }
               setControls(bestVals);
               return;
            }
         } // If iterationsSinceChange == 2
      }

      // Select next control to manipulate
      control = (control + 1) % cNUM_CONTROL_ATTRIBUTES;
      // Save current values in case they need to be reset
      prevControl = control;
      prevControlValue = ControlValues[control].current;
      // Value to adjust parameter by
      int degreeOfChange = std::max((ControlValues[control].max - ControlValues[control].min)/temperature, static_cast<unsigned int>(1));
      // If change is too large, pick random value from 1-49
      if (degreeOfChange > 100) {
         degreeOfChange = random() % 50;
      }
      int newValue = prevControlValue + (ControlValues[control].tryIncrease ? degreeOfChange : -degreeOfChange);
      int clippedNewValue = std::min(newValue, ControlValues[control].max);
      clippedNewValue = std::max(clippedNewValue, ControlValues[control].min);
      if (clippedNewValue == prevControlValue)
          ControlValues[control].tryIncrease = !ControlValues[control].tryIncrease;
      struct v4l2_control val = {ControlValues[control].control, clippedNewValue};
      ControlValues[control].current = clippedNewValue;
      vals.push_back(val);
      setControls(vals);

      framesSinceReset = 0;
      iterations++;

   } else {
      framesSinceReset++;
   }

   // Increase the value we alter each param after 2 full iterations
   if (iterations != 0 && (iterations % (2 * cNUM_CONTROL_ATTRIBUTES)) == 0) {
      temperature *= 2;
   }
}

bool CameraTab::outOfBounds(int row, int col) {
   int imageRows, imageCols;
   if (topCam->isChecked()) {
      imageRows = TOP_IMAGE_ROWS;
      imageCols = TOP_IMAGE_COLS;
   } else {
      imageRows = BOT_IMAGE_ROWS;
      imageCols = BOT_IMAGE_COLS;
   }

   if (row < 0 || row > imageRows) {
      return true;
   }

   if (col < 0 || col > imageCols) {
      return true;
   }

   return false;
}

double CameraTab::getEntropyValue() {
   // This is to stop entropy getting calculated too many times
   if (framesSinceReset < 3) {
      return 0;
   }

   unsigned int imageRows, imageCols;
   const uint8_t* frame;
   unsigned int numCorrect = 0;

   // Initialise correct frame and image size depending on selected camera
   if (topCam->isChecked()) {
      frame = topFrame;
      imageRows = TOP_IMAGE_ROWS;
      imageCols = TOP_IMAGE_COLS;
   } else {
      frame = botFrame;
      imageRows = BOT_IMAGE_ROWS;
      imageCols = BOT_IMAGE_COLS;
   }

   for (unsigned int row = 0; row < imageRows; ++row) {
      for (unsigned int col = 0; col < imageCols; ++col) {

         int y = gety(frame, row, col);
         int u = getu(frame, row, col);
         int v = getv(frame, row, col);

         // If white
         if (160 <= y && y <= 255 &&
            120 <= u && u <= 155 &&
            95 <= v && v <= 130) {
            numCorrect++;
         }

         // If green
         if (55 <= y && y <= 85 &&
            46 <= u && u <= 120 &&
            90 <= v && v <= 115) {
            numCorrect++;
         }
      }
   }

   return numCorrect;
}

// Retrieves the best settings
int CameraTab::getBestSettingsValue(__u32 controlName) {
   switch (controlName) {
      case V4L2_CID_EXPOSURE: return globalBestSettings.settings[CameraSettings::CameraSetting::EXPOSURE];
      case V4L2_CID_BRIGHTNESS: return globalBestSettings.settings[CameraSettings::CameraSetting::BRIGHTNESS];
      case V4L2_CID_CONTRAST: return globalBestSettings.settings[CameraSettings::CameraSetting::CONTRAST];
      case V4L2_CID_GAIN: return globalBestSettings.settings[CameraSettings::CameraSetting::GAIN];
      case V4L2_CID_SATURATION: return globalBestSettings.settings[CameraSettings::CameraSetting::SATURATION];
      case V4L2_CID_SHARPNESS: return globalBestSettings.settings[CameraSettings::CameraSetting::SHARPNESS];
      case V4L2_CID_DO_WHITE_BALANCE: return globalBestSettings.whiteBalance;
   }
   return INT_MIN;
}

void CameraTab::printCameraControlValues(void) {
   qDebug() << "Camera control settings";
   for (int i = 0; i < cNUM_CONTROL_ATTRIBUTES; i++) {
      qDebug() << "\t" << ControlValues[i].name.c_str() << "=" << ControlValues[i].current;
   }
}


void CameraTab::setControls(const std::vector<struct v4l2_control> &controls) {
    std::string selectedCam = (topCam->isChecked() ? "top" : "bot");
    CameraSettings *currentSetting;
    std::stringstream ss;
    if(topCam->isChecked()){
    	currentSetting = &topCameraSettings;
    } else {
    	currentSetting = &botCameraSettings;
    }

    for(std::vector<struct v4l2_control>::const_iterator ci = controls.begin();
       ci != controls.end(); ++ci) {
    	switch(ci->id){
    		 	case V4L2_CID_BRIGHTNESS:
    		 		currentSetting->settings[CameraSettings::CameraSetting::BRIGHTNESS] = ci->value;
    		 		break;
    		 	case V4L2_CID_CONTRAST:
    		 		currentSetting->settings[CameraSettings::CameraSetting::CONTRAST] = ci->value;
    		 		break;
    		 	case V4L2_CID_SATURATION:
    		 	   currentSetting->settings[CameraSettings::CameraSetting::SATURATION] = ci->value;
    		 	   break;
    		 	case V4L2_CID_HUE:
    		 		currentSetting->settings[CameraSettings::CameraSetting::HUE] = ci->value;
    		 		break;
    		 	case V4L2_CID_SHARPNESS:
    		 		currentSetting->settings[CameraSettings::CameraSetting::SHARPNESS] = ci->value;
    		 		break;
    		 	case V4L2_CID_EXPOSURE:
    		 		currentSetting->settings[CameraSettings::CameraSetting::EXPOSURE] = ci->value;
    		 		break;
    		 	case V4L2_CID_GAIN:
    		 		currentSetting->settings[CameraSettings::CameraSetting::GAIN] = ci->value;
    		 		break;
    		 	case V4L2_CID_DO_WHITE_BALANCE:
    		 		currentSetting->whiteBalance= ci->value;
    		 		break;
    	}
        ss << "--camera." << selectedCam << "." << getCameraControlName(ci->id)
           << "=" << ci->value << " ";
    }

    Q_EMIT sendStringToRobot(QString(ss.str().c_str()));
}


std::string CameraTab::getCameraControlName(__u32 controlName) {
    switch (controlName) {
        case V4L2_CID_HFLIP: return "hflip";
        case V4L2_CID_VFLIP: return "vflip";
        //TODO: remove the auto adjusters
        case V4L2_CID_EXPOSURE_AUTO: return "autoexp";
        case V4L2_CID_BRIGHTNESS: return "brightness";
        case V4L2_CID_CONTRAST: return "contrast";
        case V4L2_CID_SATURATION: return "saturation";
        case V4L2_CID_HUE: return "hue";
        case V4L2_CID_SHARPNESS: return "sharpness";
        // TODO: remove
        case V4L2_CID_AUTO_WHITE_BALANCE: return "autowb";
        case V4L2_CID_EXPOSURE: return "exposure";
        case V4L2_CID_GAIN: return "gain";
        case V4L2_CID_DO_WHITE_BALANCE: return "whitebalance";
    }
    return "unknownparam";
}


int CameraTab::getCSParamValue(const char* controlName) const {
   if (strcmp(controlName, "Horizontal Flip") == 0) {
      return currentSettings.hflip;
   }
   if (strcmp(controlName, "Vertical Flip") == 0) {
      return currentSettings.vflip;
   }
   if (strcmp(controlName, "Brightness") == 0) {
      return currentSettings.settings[CameraSettings::CameraSetting::BRIGHTNESS];
   }
   if (strcmp(controlName, "Contrast") == 0) {
      return currentSettings.settings[CameraSettings::CameraSetting::CONTRAST];
   }
   if (strcmp(controlName, "Saturation") == 0) {
      return currentSettings.settings[CameraSettings::CameraSetting::SATURATION];
   }
   if (strcmp(controlName, "Hue") == 0) {
      return currentSettings.settings[CameraSettings::CameraSetting::HUE];
   }
   if (strcmp(controlName, "Sharpness") == 0) {
      return currentSettings.settings[CameraSettings::CameraSetting::SHARPNESS];
   }
   if (strcmp(controlName, "Auto Exposure") == 0) {
      return currentSettings.settings[CameraSettings::CameraSetting::AUTO_EXPOSURE];
   }
   if (strcmp(controlName, "Exposure") == 0) {
      return currentSettings.settings[CameraSettings::CameraSetting::EXPOSURE];
   }
   if (strcmp(controlName, "Gain") == 0) {
      return currentSettings.settings[CameraSettings::CameraSetting::GAIN];
   }
   if (strcmp(controlName, "Auto White Balance") == 0) {
      return currentSettings.settings[CameraSettings::CameraSetting::AUTO_WHITE_BALANCE];
   }
   if (strcmp(controlName, "White Balance") == 0) {
      return currentSettings.whiteBalance;
   }
   return 0;
}

void CameraTab::setValueInSetingsDS(const char* controlName, CameraSettings& settings, int value) {
   if (strcmp(controlName, "Horizontal Flip") == 0) {
      settings.hflip = value != 0;
   }
   if (strcmp(controlName, "Vertical Flip") == 0) {
      settings.vflip = value != 0;
   }
   if (strcmp(controlName, "Brightness") == 0) {
      settings.settings[CameraSettings::CameraSetting::BRIGHTNESS] = value;
   }
   if (strcmp(controlName, "Contrast") == 0) {
      settings.settings[CameraSettings::CameraSetting::CONTRAST] = value;
   }
   if (strcmp(controlName, "Saturation") == 0) {
      settings.settings[CameraSettings::CameraSetting::SATURATION] = value;
   }
   if (strcmp(controlName, "Hue") == 0) {
      settings.settings[CameraSettings::CameraSetting::HUE] = value;
   }
   if (strcmp(controlName, "Sharpness") == 0) {
      settings.settings[CameraSettings::CameraSetting::SHARPNESS] = value;
   }
   if (strcmp(controlName, "Auto Exposure") == 0) {
      settings.settings[CameraSettings::CameraSetting::AUTO_EXPOSURE] = value;
   }
   if (strcmp(controlName, "Exposure") == 0) {
      settings.settings[CameraSettings::CameraSetting::EXPOSURE] = value;
   }
   if (strcmp(controlName, "Gain") == 0) {
      settings.settings[CameraSettings::CameraSetting::GAIN] = value;
   }
   if (strcmp(controlName, "Auto White Balance") == 0) {
      settings.settings[CameraSettings::CameraSetting::AUTO_WHITE_BALANCE] = value;
   }
   if (strcmp(controlName, "White Balance") == 0) {
      settings.whiteBalance = value;
   }
}

*/

