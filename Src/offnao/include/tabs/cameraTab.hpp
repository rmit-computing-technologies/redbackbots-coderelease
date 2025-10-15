#pragma once

#include <QCheckBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QMenuBar>
#include <QPushButton>
#include <QRadioButton>
#include <QSlider>

#include <linux/videodev2.h> // __u32 is defined in here

#include "renderers/CameraImageRender.hpp"
#include "tabs/tab.hpp"
#include "tabs/PointCloud.hpp"


#include "types/camera/CameraSettings.hpp"

// Forward Declarations
class CameraButtonHandler;


class CameraTab : public Tab {
   Q_OBJECT
   public:
      CameraTab(QTabWidget *parent, QMenuBar *menuBar);

      // Needs to be public so buttons/sliders can update the current settings variable
      void updateCSfromVisual();

   private:
      void init();
      void resetCameraImages();

      void updateVisualsFromCS();
   
      void setControls(const std::vector<struct v4l2_control> &);
      std::string getCameraControlName(__u32 controlName);

      int getCSParamValue(const char* controlName) const {};
      void setValueInSetingsDS(const char* controlName, CameraSettings& settings, int value) {};   
      int getBestSettingsValue(__u32 controlName) {};
      double getEntropyValue() {};
      bool outOfBounds(int row, int col) {};

      // Image forms that can are displayed
      enum ImageType {
         // CAMERA, 
         RGB, 
         YUV, 
         HSV
      };

      // Constants
      static constexpr int NUM_IMG_TYPES = HSV + 1;

      // Camera Settings tracking
      CameraSettings currentSettings;
      CameraSettings topCameraSettings;
      CameraSettings botCameraSettings;

      // Nao Camera Images
      CameraImage topCameraImage;
      CameraImage botCameraImage;

      // Camera Settings Options
      QRadioButton *topCam;
      QRadioButton *bottomCam;
      std::vector<QCheckBox*> checkInputBoxes;
      std::vector<CameraButtonHandler*> valueInputBoxes;

      // Auto Tune variables
      int numLocalMax;
      CameraSettings globalBestSettings;
      float globalBestEntropy;

      // Frame counter
      int framesSinceReset;
      int prevControl;
      int prevControlValue;

      // Camera image, RGB histogram, YUV histogram and HSV histogram
      CameraImageRender* imageRender;
      QPixmap imagePixmaps[NUM_IMG_TYPES];
      QLabel *imageLabels[NUM_IMG_TYPES];
      PointCloud pointCloud;

      QGridLayout *layout;

      // Track best entropy value
      bool autoTuneOn;
      float bestEntropy;
      bool cameraToggled;

      /* Re-draw the image box from current frame. */
      void redraw();

   public Q_SLOTS:
      void toggleChange(bool active = false);
      void newNaoData(NaoData *naoData);

      // Turn camera auto tuner on/off
      void autoTuner(void) {};

      // Retrieves camera settings on robot
      void fetchValues(void);

      // Send current values to robot
      void sendValues(void);

      // Select a random camera attribute and a random value
      void tuneValue(float entropy) {};

      void printCameraControlValues(void) {};

   Q_SIGNALS:
      /**
       * Methods to send various content to the Nao
       */
      void sendCameraSettingsToRobot(int whichCamera, QSharedPointer<CameraSettings> settings);
      void sendStringToRobot(QString item);
};

class CheckboxHandler : public QObject {
Q_OBJECT
public:
   CheckboxHandler(CameraTab *camtab, QSlider *sl, QPushButton *db, QLineEdit *qle, QPushButton *ib,
                   int control);

public Q_SLOTS:
   void toggleState(bool enable);

private:
   CameraTab *camtab;
   QSlider *sl;
   QPushButton *db;
   QLineEdit *qle;
   QPushButton *ib;
   int control;
};

class SliderHandler : public QObject {
Q_OBJECT
public:
   SliderHandler(CameraTab *camtab, int control, QSlider *sl, QLineEdit *qle);

public Q_SLOTS:
   void setValue(int newValue);

private:
   CameraTab *camtab;
   int control;
   QSlider *sl;
   QLineEdit *qle;
};

/**
 * handles increment and decrement buttons
 */
class CameraButtonHandler : public QObject {
Q_OBJECT
public:
   /**
    * creates a button handler that modifies a textbox and sends a new
    * value to the robot
    *
    * @param camtab the CameraTab i'm called from
    * @param control the index of the control to change
    * @param qpb the button that was pressed, or textbox that was modified
    *            if NULL, uses qle as trigger
    * @param qle the textbox to modify
    * @param sl  slider to change
    */
   CameraButtonHandler(CameraTab *camtab, int control, QPushButton *qpb,
                        QLineEdit *qle, QSlider *sl);
   void setValue(unsigned int);
   unsigned int getValue();
   int getControl() const;

public Q_SLOTS:
   // The +/- buttons are clicked
   void clicked(bool click = false);

   // The value is set directly
   void directUpdate();

private:
   /**
    * the CameraTab i'm called from
    */
   CameraTab *camtab;

   /**
    * the index of the control to change
    */
   int control;

   /**
    * the button that was pressed
    */
   QPushButton *qpb;

   /**
    * the textbox to modify
    */
   QLineEdit *qle;

   /**
    * the slider to modify
    */
   QSlider *sl;
};

