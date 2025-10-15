
/**
 * @file CameraImageRender.cpp
 * 
 * Renders a camera image as a QLabel.
 * Optionally can render a collection of overlays onto the image
 * 
 * @author RedbackBots
 * 
 */

#include "renderers/CameraImageRender.hpp"

#include "perception/vision/other/YUV.hpp"
#include "utils/classifier.hpp"
#include "renderers/OverlayPainter.hpp"

#include <QImage>

CameraImageRender::CameraImageRender(int cols, int rows, CameraInfo::Camera whichCamera) :
    cols(cols), rows(rows), whichCamera(whichCamera),
    blackboard(nullptr)
{
    cameraImage.setResolution(NO_IMAGE_DIM, NO_IMAGE_DIM);

    imagePixmap = QPixmap(cols, rows);
    imagePixmap.fill(Qt::darkGray);
    this->setPixmap(imagePixmap);
    this->setMinimumSize(cols, rows);
    this->setMaximumSize(cols, rows);
}

CameraImageRender::~CameraImageRender() {
    for (OverlayPainter* overlay : overlays) {
        delete overlay;
    }
}

void CameraImageRender::addOverlay(OverlayPainter* overlay) {
    if (overlay != nullptr) {
        overlays.push_back(overlay);
    }
}

void CameraImageRender::setCameraImage(CameraImage& cameraImage) {
    this->cameraImage = cameraImage;
}

void CameraImageRender::setBlackboard(Blackboard *blackboard) {
    this->blackboard = blackboard;
}

void CameraImageRender::redraw() {
    QImage *qImage = new QImage(cameraImage.width * 2, cameraImage.height, QImage::Format_RGB32);

    // Draw the image pixels
    drawImage(qImage);

    // Scale images up to real size to draw overlays
    QPixmap pixmap = QPixmap::fromImage(
                qImage->scaled(cameraImage.width * 2, cameraImage.height));

    // Draw overlays
    drawOverlays(&pixmap);

    // Rescale the images back to fit the offnao screen
    pixmap = pixmap.scaled(cols, rows);
    this->setPixmap(pixmap);

    // Cleanup
    delete qImage;
}

void CameraImageRender::drawImage(QImage* qImage) {
    // Draw images
    PixelTypes::YUVPixel yuv;
    QRgb rgb;

    // llog(INFO) << "topimage: " << topCameraImage.width << " x " << topCameraImage.height << std::endl;
    for (int row = 0; row < cameraImage.height; ++row) {
        for (int col = 0; col < cameraImage.width * 2; ++col) {
            // Greyscale pixel
            // int pixel = topCameraImage.getY(col, row);
            // topImage->setPixelColor(col, row, QColor(pixel,pixel,pixel));

            // Colour Pixel
            yuv = cameraImage.getYUV(col, row);
            rgb = Classifier::yuv2rgb(yuv.y,yuv.u,yuv.v);
            qImage->setPixelColor(col, row, rgb);
        }
    }

    // Add missing text
    if (cameraImage.height == NO_IMAGE_DIM) {
        std::string text = "No image\n" + CameraInfo::enumCameraToString(whichCamera);

        OverlayPainter olPainter;
        olPainter.begin(qImage);
        olPainter.setPen(QPen(Qt::red));
        olPainter.setFont(QFont("Times", 12, QFont::Bold));
        olPainter.drawText(qImage->rect(), Qt::AlignCenter, text.c_str());

        // Testing OverlayPainter
        // Point p(20,20);
        // olPainter.drawPoint(p, Qt::blue);
        // std::pair<int, int> horizon(10,15);
        // olPainter.drawHorizon(horizon);
        // FieldBoundary fb;
        // fb.boundaryInImage.push_back(Vector2i(0,50));
        // fb.boundaryInImage.push_back(Vector2i(10,55));
        // fb.boundaryInImage.push_back(Vector2i(20,55));
        // fb.boundaryInImage.push_back(Vector2i(30,60));
        // olPainter.drawFieldBoundary(fb);

        olPainter.end();
    }
}

void CameraImageRender::drawOverlays(QPixmap* pixmap) {
    if (blackboard == nullptr) {
        return;
    }

//    std::vector<BallInfo>            balls           = readFrom(vision, balls);
//    std::vector<RobotVisionInfo>     robots          = readFrom(vision, robots);
//    std::vector<FieldFeatureInfo>    fieldFeatures   = readFrom(vision, fieldFeatures);
//    std::vector<RegionI>             regions         = readFrom(vision, regions);

    // Iterate through overlays to draw
    for (OverlayPainter* overlay : overlays) {
        overlay->begin(pixmap);
        overlay->drawOverlay(blackboard);
        overlay->end();
    }

   /** OLD DRAWING */

   // drawOverlaysGeneric (topImage,
   //                      botImage,
   //                      &horizon,
   //                      &balls,
   //                      &robots,
   //                      &fieldBoundaries,
   //                      &fieldFeatures,
   //                      &regions,
   //                      0.5
   //                     );

   /*
   QPainter painter(image);
   const Pose &pose = readFrom(kinematics, pose);
   const std::pair<int, int> horizon = pose.getHorizon();
   painter.setBrush(QBrush(QColor(255, 255, 255)));
   painter.drawLine(0,horizon.first/SALIENCY_DENSITY*2,
         640/SALIENCY_DENSITY*2,
         horizon.second/SALIENCY_DENSITY*2);

   //draw body exclusion points
   painter.setBrush(QBrush(QColor(255, 255, 255)));
   float scale = 2.0/SALIENCY_DENSITY;
   const int16_t *points = pose.getExclusionArray();
   for (int i = 0; i < Pose::EXCLUSION_RESOLUTION; i++) {
       painter.drawEllipse(QPoint(scale*640 * i*1.0/Pose::EXCLUSION_RESOLUTION,
                         scale*points[i]), 2, 2);
   }
   */
}

// void OverviewTab::drawOverlaysGeneric (QPaintDevice *topImage,
//       QPaintDevice                          *botImage,
//       const std::pair<int, int>             *horizon,
//       const std::vector<BallInfo>           *balls,
//       const std::vector<RobotVisionInfo>    *robots,
//       const std::vector<FieldBoundaryInfo>  *fieldBoundaries,
//       const std::vector<FieldFeatureInfo>   *fieldFeatures,
//       const std::vector<RegionI>            *regions,
//       float scale)
// {
//    OverlayPainter topPainter;
//    OverlayPainter botPainter;

//    if (topImage) {
//       topPainter.begin(topImage);
//       topPainter.scale (scale, scale);
//    }

//    if (botImage){
//       botPainter.begin(botImage);
//       botPainter.scale (scale, scale);
//       botPainter.translate(0, -TOP_IMAGE_ROWS);
//    }

//    if (topImage && horizon) {
//       topPainter.drawHorizon(*horizon);
//    }

//    if (balls) {
//       std::vector<BallInfo>::const_iterator i;
//       for (i = balls->begin (); i != balls->end (); ++ i) {
//          if (topImage) topPainter.drawBallOverlay (*i);
//          if (botImage) botPainter.drawBallOverlay (*i);
//       }
//    }

//    if (robots) {
//       std::vector<RobotVisionInfo>::const_iterator i;
//       for (i = robots->begin (); i != robots->end (); ++ i) {
//          if (topImage) topPainter.drawRobotOverlay (*i);
//          if (botImage) botPainter.drawRobotOverlay (*i);
//       }
//    }

//    if (fieldBoundaries) {
//       std::vector<FieldBoundaryInfo>::const_iterator i;
//       for (i = fieldBoundaries->begin (); i != fieldBoundaries->end (); ++ i) {
//          if (topImage) topPainter.drawFieldBoundaryOverlay (*i);
//          if (botImage) botPainter.drawFieldBoundaryOverlay (*i);
//       }
//    }

//    if (fieldFeatures) {
//       std::vector<FieldFeatureInfo>::const_iterator i;
//       for (i = fieldFeatures->begin (); i != fieldFeatures->end (); ++ i) {
//          if (topImage) topPainter.drawFieldFeatureOverlay (*i);
//          if (botImage) botPainter.drawFieldFeatureOverlay (*i);
//       }
//    }

//    if (regions) {
//       std::vector<RegionI>::const_iterator i;
//       for (i = regions->begin (); i != regions->end (); ++ i) {
//           if(i->isTopCamera() && topImage)
//             topPainter.drawRegionOverlay(*i);
//           if(!i->isTopCamera() && botImage)
//             botPainter.drawRegionOverlay(*i);
//       }
//    }

//    if (topImage) topPainter.end();
//    if (botImage) botPainter.end();
// }
