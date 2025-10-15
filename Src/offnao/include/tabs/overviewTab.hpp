#pragma once

#include <QGridLayout>
#include <QLabel>
#include <QMenuBar>

#include "renderers/CameraImageRender.hpp"
#include "tabs/fieldView.hpp"
#include "tabs/tab.hpp"
#include "tabs/variableView.hpp"


// Forward Declarations
class Blackboard;

/*
 * This is the default/initial tab that shows on entering Off-Nao.
 * As the name suggests it gives an overview of most of the important
 * aspects of the nao. Good for general gameplay watching etc.
 *
 * Also will be used for localization debugging unless the localization
 * people need more info.
 */
class OverviewTab : public Tab {
   Q_OBJECT
   public:
      OverviewTab(QTabWidget *parent, QMenuBar *menuBar);
      virtual ~OverviewTab();
      QPixmap *renderPixmap;
      QLabel *renderWidget;

   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      void initOverlays();
      void redraw();

      QString naoName;

      QGridLayout *layout;

      /* These variables are used to present the debug variables from the nao */
      VariableView variableView;
      FieldView fieldView;

      /* Image rendering */
      CameraImageRender* topImageRender;
      CameraImageRender* botImageRender;

      /* Images from Nao */
      CameraImage topCameraImage;
      CameraImage botCameraImage;

      // Blackboard ref
      Blackboard *blackboard;

   public Q_SLOTS:
      void newNaoData(NaoData *naoData);
      void setNao(const QString &naoName);
};
