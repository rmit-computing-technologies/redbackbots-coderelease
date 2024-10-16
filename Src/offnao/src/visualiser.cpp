#include "visualiser.hpp"
#include "visualiser.tcc"

#include "mediaPanel.hpp"
#include "readers/record2Reader.hpp"
#include "readers/bbd2Reader.hpp"
#include "progopts.hpp"
#include "tabs/tab.hpp"
// #include "tabs/jointsTab.hpp"
// #include "tabs/temperatureTab.hpp"
// #include "tabs/aroundFeetTab.hpp"
#include "tabs/cameraTab.hpp"
#include "tabs/overviewTab.hpp"
// #include "tabs/visionTab.hpp"
// #include "tabs/sensorTab.hpp"
// #include "tabs/cameraPoseTab.hpp"
// #include "tabs/graphTab.hpp"
// #include "tabs/walkTab.hpp"
// #include "tabs/zmpTab.hpp"
// #include "tabs/LogsTab.hpp"
// #include "tabs/teamTab.hpp"
#include "perception/vision/Vision.hpp"
#include "perception/vision/VisionDebuggerInterface.hpp"
#include "readers/dumpReader.hpp"
#include "readers/networkReader.hpp"

#include <thread/Thread.hpp>
#include <QInputDialog>
#include <QFileDialog>

#include <netdb.h>
#include <arpa/inet.h>

// TODO: REMOVE global namespace uses
using namespace boost;
using std::string;
using namespace std;

Visualiser::Visualiser(QWidget *parent)
   : QMainWindow(parent), ui(new Ui::Visualiser),
   reader(0), naoData(0), cb() {
      // call initLogs so that vision et al don't segfault when they llog
      Thread::name = "Visualiser";

      lastTabIndex = -1;

      srand(time(0));

      vision = new Vision();

      ui->setupUi(this);
      ui->centralWidget->setMinimumSize(1250, 600);

      initMenu();

      // add the media panel to the bottom of the window
      mediaPanel = new MediaPanel(ui->centralWidget);

      // widgets for connecting to the naos
      connectionBar = new QDockWidget;
      cb.setupUi(connectionBar);
      readHostnamesFromJson(QString::fromUtf8(getenv("REDBACKBOTS_CHECKOUT_DIR")) + "/Config/Robots/robotsv2.cfg");
      connect(cb.cbHost, SIGNAL(activated(const QString &)),
              this, SLOT(connectToNao(const QString &)));
      connect(cb.bgMasks, SIGNAL(buttonClicked(QAbstractButton *)),
              this, SLOT(writeToNao(QAbstractButton *)));

      // set up tab holder
      tabs = new QTabWidget(ui->centralWidget);
      tabs->setMinimumSize(600, 450);

      overviewTab =  new OverviewTab(tabs, ui->menuBar, vision);
      // visionTab = new VisionTab(tabs, ui->menuBar, vision);
      cameraTab = new CameraTab(tabs, ui->menuBar, vision);
      // sensorTab = new SensorTab(tabs, ui->menuBar, vision);
      // cameraPoseTab = new CameraPoseTab(tabs, ui->menuBar, vision);
      // graphTab = new GraphTab(tabs, ui->menuBar, vision);
      // jointsTab = new JointsTab(tabs, ui->menuBar, vision);
      // temperatureTab = new TemperatureTab(tabs, ui->menuBar, vision);
      // walkTab = new WalkTab(tabs, ui->menuBar, vision);
      // zmpTab = new ZMPTab(tabs, ui->menuBar, vision);
      // aroundFeetTab = new AroundFeetTab(tabs, ui->menuBar, vision);

      tabVector.push_back(overviewTab);
      // tabVector.push_back(visionTab);
      tabVector.push_back(cameraTab);
      // tabVector.push_back(sensorTab);
      // tabVector.push_back(cameraPoseTab);
      // tabVector.push_back(graphTab);
      // tabVector.push_back(jointsTab);
      // tabVector.push_back(temperatureTab);
      // tabVector.push_back(walkTab);
      // tabVector.push_back(zmpTab);
      // tabVector.push_back(aroundFeetTab);
      // tabVector.push_back(logsTab = new LogsTab(tabs, cb.cbHost));




      /* Set up the tabs */
      Q_FOREACH (Tab *t, tabVector) {
         tabs->addTab(t, QString(t->metaObject()->className()).
                           remove(QRegExp("Tab$", Qt::CaseInsensitive)));
         connect(t, SIGNAL(showMessage(QString, int)),
                 ui->statusBar, SLOT(showMessage(QString, int)));
      }

      // /* Used to redraw tabs when they are in focus */
      connect(tabs, SIGNAL(currentChanged(int)), this,
              SLOT(currentTabChanged(int)));

      ui->rootLayout->addWidget(mediaPanel, 1, 0, 1, 1);
      ui->rootLayout->addWidget(tabs, 0, 0, 1, 1);
   }

Visualiser::~Visualiser() {
   std::cout << "---- about to exit ----" << std::endl << std::flush;
   /*
   delete connectionBar;
   delete ui;
   for (unsigned int i = 0; i > tabVector.size(); i++) {
      delete tabVector[i];
   }
   */
}

void Visualiser::newNaoData(NaoData *naoData) {
   this->naoData = naoData;
}

// sets up the generic menus
void Visualiser::initMenu() {
   fileMenu = new QMenu("&File");
   ui->menuBar->addMenu(fileMenu);

   // set up file menu
   loadAct = new QAction(tr("&Load"), fileMenu);
   saveAsAct = new QAction(tr("&Save As"), fileMenu);

   exitAct = new QAction(tr("&Quit"), fileMenu);


   fileMenu->addAction(tr("Connect to &Nao"), this, SLOT(connectToNao()),
         QKeySequence(tr("Ctrl+N", "File|New")));
   fileMenu->addAction(tr("&Open File"), this, SLOT(openFileDialogue()),
         QKeySequence(tr("Ctrl+O", "File|Open")));
   // This is networkReader specific, but I'm not sure where to put it
   fileMenu->addAction(tr("Send Co&mmand Line String"), this,
                       SLOT(commandLineString()),
                            QKeySequence(tr("F5", "Refresh")));
   fileMenu->addAction(tr("&Save File"), this, SLOT(saveFileDialogue()),
                       QKeySequence(tr("Ctrl+S", "File|Save")));
   fileMenu->addAction(tr("&Save Whiteboard File"), this, SLOT(saveWhiteboardFileDialogue()),
                       QKeySequence(tr("Ctrl+W", "File|SaveWhiteboard")));
   fileMenu->addAction(tr("&Extract Raw Images"), this, SLOT(extractRawImagesDialogue()),
                       QKeySequence(tr("Ctrl+E", "File|Extract")));
   fileMenu->addAction(tr("&Extract Regions of Interest"), this, SLOT(extractROIsDialogue()),
                       QKeySequence(tr("Ctrl+R", "File|ExtractROI")));
   fileMenu->addAction(tr("&Close Current Reader"), this,
                       SLOT(disconnectFromNao()),
         QKeySequence(tr("Ctrl+W", "File|Close")));

   fileMenu->addAction(tr("&Quit"), this,
         SLOT(close()),
         QKeySequence(tr("Ctrl+Q", "Close")));

}


void Visualiser::close() {
   delete connectionBar;

   for (unsigned int i = 0; i > tabVector.size(); i++) {
      delete tabVector[i];
   }
   delete tabs;
   
   //delete vision;
   delete ui;
   exit(0);
}

void Visualiser::openFileDialogue() {
   QString fileName =
     QFileDialog::getOpenFileName(this, "Open File",
                                  getenv("RUNSWIFT_CHECKOUT_DIR"),
                                  "Known file types (*.bbd2 *.yuv *.ofn2);;"
                                  "Dump files (*.yuv);;"
                                  "Record files (*.ofn2);;"
                                  "Blackboard Dump files (*.bbd2)"
                                  );
   openFile(fileName);

}

void Visualiser::openFile (const QString &path)
{
   if (!path.isEmpty()) {
      if(path.endsWith(".bbd2")) {
         reconnect<BBD2Reader, const QString &>(path);
      } else if(path.endsWith(".yuv"))
         reconnect<DumpReader, const QString &>(path);
      else if (path.endsWith(".ofn2")) {
         reconnect<Record2Reader, const QString &>(path);
      } else
         ui->statusBar->showMessage(
           QString("Unknown file extension, modify %1:%2:%3").
                   arg(__FILE__, __PRETTY_FUNCTION__,
                       QString::number(__LINE__)));
   }
}

void Visualiser::saveWhiteboardFileDialogue() {
   QString fileName =
   QFileDialog::getSaveFileName(this, "Save File",
                                getenv("RUNSWIFT_CHECKOUT_DIR"),
                                       "Known file types (*.wb);;"
                                       "Record files (*.wb)"
                                       );
   std::cout << "SAVING WB FILE" << std::endl;
   saveFile(fileName);
}

void Visualiser::saveFileDialogue() {
   QString fileName =
   QFileDialog::getSaveFileName(this, "Save File",
                                getenv("RUNSWIFT_CHECKOUT_DIR"),
                                "Known file types ("/**.yuv */"*.ofn2);;"
                                // "Dump files (*.yuv);;"
                                "Record files (*.ofn2)"
                                );

   saveFile(fileName);
}

void Visualiser::saveFile (const QString &path)
{
   if (!path.isEmpty()) {
      if (path.endsWith(".ofn2")) {
         Record2Reader::write(path, *naoData);
      // else if(path.endsWith(".yuv"))
      //   saveDump(path);
      } else {
         ui->statusBar->showMessage(QString("Unknown file extension, saving as %1.ofn2").arg(path));
         Record2Reader::write(path + ".ofn2", *naoData);
      }
   }
}

void Visualiser::extractRawImagesDialogue() {
  QString dir = QFileDialog::getExistingDirectory(this, tr("Save images to folder"),
                                               getenv("RUNSWIFT_CHECKOUT_DIR"),
                                               QFileDialog::ShowDirsOnly
                                               | QFileDialog::DontResolveSymlinks);
   extractRawImages(dir);
}

void Visualiser::extractRawImages(const QString &path)
{
   if (!path.isEmpty() && naoData) {
      Record2Reader::dumpImages(path, *naoData, false);
   }
}

void Visualiser::extractROIsDialogue() {
  QString dir = QFileDialog::getExistingDirectory(this, tr("Save images to folder"),
                                               getenv("RUNSWIFT_CHECKOUT_DIR"),
                                               QFileDialog::ShowDirsOnly
                                               | QFileDialog::DontResolveSymlinks);
   extractROIs(dir);
}

void Visualiser::extractROIs(const QString &path)
{
   if (!path.isEmpty() && naoData) {
      Record2Reader::dumpImages(path, *naoData, true);
   }
}

/*
 * Not yet used. Probably will be for the network reader.
 */
void Visualiser::startRecording(QAction *action) {
}

void Visualiser::currentTabChanged(int tabIndex) {
   if (lastTabIndex != -1) {
      tabVector[lastTabIndex]->tabDeselected();
   }
   tabVector[tabIndex]->tabSelected();

   Q_EMIT refreshNaoData();
}

OffNaoMask_t Visualiser::transmissionMask(const QAbstractButton *) {
   OffNaoMask_t mask = 0;
   QList<QAbstractButton *> buttons = cb.bgMasks->buttons();
   while (!buttons.isEmpty()) {
      QAbstractButton *button = buttons.takeFirst();
      if (button->isChecked()) {
         if (button == cb.cbBlackboard)
            mask |= BLACKBOARD_MASK;
         else if (button == cb.cbSaliency)
            mask |= SALIENCY_MASK;
         else if (button == cb.cbRaw)
            mask |= RAW_IMAGE_MASK;
         else if (button == cb.cbBatch)
            mask |= USE_BATCHED_MASK;
         else if (button == cb.cbParticles)
            mask |= PARTICLE_FILTER_MASK;
         else if (button == cb.cbRobotFilter)
            mask |= ROBOT_FILTER_MASK;
         else
            ui->statusBar->showMessage("Error in parsing mask checkboxes!");
      }
   }
   return mask;
}

void Visualiser::connectToNao(const QString &naoName, const uint16_t naoPort) {
   QString naoIP;
   string strNao = naoName.toStdString();
	 if (strNao.find('.') != string::npos){
      // If the ip address of the robot has been entered
      naoIP = naoName;
   } else {
      // If the name of the robot has been entered
      // Look up ip address from /etc/hosts
      const hostent* host_info = 0 ;
      host_info = gethostbyname(strNao.c_str());
      if(host_info)
      {
            const in_addr* address = (in_addr*)host_info->h_addr_list[0] ;
            naoIP = inet_ntoa( *address );
      }      
   }
   if(reconnect<NetworkReader, pair<pair<const QString &, int>, OffNaoMask_t> >
      (make_pair(make_pair(naoIP, naoPort), transmissionMask(NULL)))) {
      NetworkReader *reader = dynamic_cast<NetworkReader *>(this->reader);
      Q_ASSERT(reader);
      connect(this, SIGNAL(sendCommandLineString(const QString&)), reader,
              SLOT(sendCommandLineString(QString)));
      connect(cameraTab, SIGNAL(sendCommandToRobot(QString)),
              reader, SLOT(sendCommandLineString(QString)));
      // TODO: ADD BACK WITH TABS
      // connect(cameraPoseTab, SIGNAL(sendCommandToRobot(QString)),
      //         reader, SLOT(sendCommandLineString(QString)));
      // connect(logsTab, SIGNAL(sendCommandToRobot(QString)),
      //         reader, SLOT(sendCommandLineString(QString)));

      this->reader = reader;
      setUpReaderSignals(this->reader);
      this->reader->start();

      overviewTab->setNao(naoName);
   }
}

void Visualiser::writeToNao(QAbstractButton *qab) {
   if (reader) {
      ((NetworkReader*)reader)->write(transmissionMask(qab));
   }
}

void Visualiser::connectToNao() {
   addDockWidget(Qt::TopDockWidgetArea, connectionBar);
   connectionBar->show();
   cb.cbHost->setFocus();
   cb.cbHost->lineEdit()->selectAll();
}

void Visualiser::connectToNao(const QString &naoName) {
   connectToNao(naoName, cb.sbPort->value());
}

// This is networkReader specific, but I'm not sure where to put it
void Visualiser::commandLineString() {
   QString item = QInputDialog::getText(NULL, "Send String",
                                           tr("Command FieldEdge String:"));
   Q_EMIT sendCommandLineString(item);
}

void Visualiser::disconnectFromNao() {
   if (reader && reader->isFinished() == false) {
      connect(reader, SIGNAL(finished()), this, SLOT(disconnectFromNao()));
      reader->finishUp();
      qDebug("Try to destroy reader. Wait for thread to exit.");
   } else if (reader) {
      Q_EMIT readerClosed();
      delete reader;
      reader = 0;
      qDebug("Finished destroying reader");
      ui->statusBar->showMessage(QString("Reader destroyed."));
   }
}

void Visualiser::setUpReaderSignals(Reader *reader) {

   for (unsigned int i = 0; i < tabVector.size(); i++) {
      connect(reader, SIGNAL(newNaoData(NaoData*)), tabVector[i],
            SLOT(newNaoData(NaoData *)));
      connect(this, SIGNAL(readerClosed()), tabVector[i],
            SLOT(readerClosed()));
   }

   connect(reader, SIGNAL(newNaoData(NaoData*)), mediaPanel,
         SLOT(newNaoData(NaoData *)));
   connect(reader, SIGNAL(newNaoData(NaoData*)), this,
           SLOT(newNaoData(NaoData *)));

   connect(mediaPanel->forwardAct, SIGNAL(triggered()), reader,
         SLOT(forwardMediaTrigger()));
   connect(mediaPanel->backAct, SIGNAL(triggered()), reader,
         SLOT(backwardMediaTrigger()));
   connect(mediaPanel->playAct, SIGNAL(triggered()), reader,
         SLOT(playMediaTrigger()));
   connect(mediaPanel->pauseAct, SIGNAL(triggered()), reader,
         SLOT(pauseMediaTrigger()));
   connect(mediaPanel->stopAct, SIGNAL(triggered()), reader,
         SLOT(stopMediaTrigger()));
   connect(mediaPanel->recordAct, SIGNAL(triggered()), reader,
         SLOT(recordMediaTrigger()));
   connect(mediaPanel->frameSlider, SIGNAL(valueChanged(int)),
         reader, SLOT(sliderMoved(int)));

   connect(this, SIGNAL(refreshNaoData()), reader, SLOT(refreshNaoData()));
   connect(mediaPanel->frameSlider, SIGNAL(sliderReleased()), reader,
         SLOT(refreshNaoData()));
   connect(reader, SIGNAL(showMessage(QString, int)), ui->statusBar,
           SLOT(showMessage(QString, int)));
   connect(reader, SIGNAL(openFile()), this, SLOT(openFileDialogue()));
   connect(reader, SIGNAL(disconnectFromNao()), this,
           SLOT(disconnectFromNao()));
   mediaPanel->recordButton->setFocus();
}

void Visualiser::readHostnamesFromJson(const QString& filename) {
   QStringList hostnames;

   // Open the JSON file
   QFile file(filename);
   if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      qDebug() << "Failed to open file:" << filename;
      return;
   }

   // Read JSON data from the file
   QByteArray jsonData = file.readAll();
   file.close();

   // Parse JSON data
   QJsonParseError error;
   QJsonDocument doc = QJsonDocument::fromJson(jsonData, &error);
   if (doc.isNull()) {
      qDebug() << "Failed to parse JSON:" << error.errorString();
      return;
   }

   // Check if the JSON document is an object
   if (!doc.isObject()) {
      qDebug() << "JSON document is not an object";
      return;
   }

   // Get the root object
   QJsonObject root = doc.object();

   // Iterate over the object keys (hostnames)
   for (const QString& hostname : root.keys()) {
      if (hostname != "**WARNING"){
         hostnames.append(hostname);
      }
   }

   // Add robots from config file to widget
   cb.cbHost->clear(); // Clear existing items
   cb.cbHost->addItems(hostnames); // Add new hostnames
}