#pragma once

#include <QImage>
#include <QObject>
#include <QString>
#include <QTabWidget>
#include <QWidget>

#include "naoData.hpp"

class Tab : public QWidget {
   Q_OBJECT

   public:
      Tab() {}
      Tab(QTabWidget *parent) : 
         QWidget(parent), 
         parent(parent)
         {}

   protected:
      QTabWidget *parent;

      /**
       * Returns the RGB value of the pixel at row, col
       * @arg: num_cols is used to vary the image resolution
       */
      virtual QRgb getRGB(unsigned int col, unsigned int row,
                          const uint8_t *yuv, int num_cols);


      virtual void tabSelected();
      virtual void tabDeselected();

   Q_SIGNALS:
      void showMessage(const QString &, int timeout = 0);

   public Q_SLOTS:
      virtual void newNaoData(NaoData *naoData) = 0;
      virtual void readerClosed() {}

   friend class Visualiser;
};
