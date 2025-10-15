#pragma once

#include <QCheckBox>
#include <QFrame>
#include <QGridLayout>
#include <QLabel>
#include <QMenuBar>

#include "renderers/OverlayPainter.hpp"
#include "renderers/CameraImageRender.hpp"
#include "tabs/tab.hpp"

#include <string>
#include <vector>

// Forward Declarations
class Blackboard;

/*
 * Tab for displaying output of vision algorithms.
 * Different renders can be enabled/disabled as desired
 */
class VisionTab : public Tab {
   Q_OBJECT
   public:
      VisionTab(QTabWidget *parent, QMenuBar *menuBar);
      virtual ~VisionTab();

   private:
      void init();
      void redraw();

      /* Layouts */
      QFrame* optionsFrame;
      QGridLayout* baseLayout;
      QGridLayout* optionsLayout;

      /* Image rendering */
      CameraImageRender* topImageRender;
      CameraImageRender* botImageRender;

      /* CheckBoxes */
      std::vector<QCheckBox*> checkboxes;

      /* Images from Nao */
      CameraImage topCameraImage;
      CameraImage botCameraImage;

      /* Blackboard ref */
      Blackboard* blackboard;

   public Q_SLOTS:
      void newNaoData(NaoData *naoData);
};

class RenderSwitch : public QCheckBox {
   Q_OBJECT
   public:
      RenderSwitch(std::string label, bool checked = false);
      virtual ~RenderSwitch();

      void addOverlay(OverlayPainter* overlay);

   public Q_SLOTS:
      void toggleState(bool enable);

   private:
      // Overlays controlled by this switch
      std::vector<OverlayPainter*> overlays;

};
