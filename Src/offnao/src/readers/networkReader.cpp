#include "readers/networkReader.hpp"

#include <QDebug>
#include <QInputDialog>
#include <QMessageBox>
#include <QString>
#include <QStringList>
#include <QInputDialog>

#include <bitset>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <string>
#include <sstream>

#include "progopts.hpp"
#include "blackboard/Blackboard.hpp"
#include "thread/Thread.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/camera/CameraSettings.hpp"
#include "types/geometry/AbsCoord.hpp"
#include "types/serialise/CameraSettingsSerialse.hpp"
#include "communication/Connection.hpp"
#include "utils/Timer.hpp"


NetworkReader::NetworkReader(const QString &robotName, int robotPort, OffNaoMask_t mask) :
   connection_(0), received(), isRecording(false), robotName(robotName), robotPort(robotPort) {
   this->mask = mask;
   this->naoData.setPaused(false);
   isAlive = true;
}

NetworkReader::NetworkReader(std::pair<std::pair<const QString &,int>, OffNaoMask_t> robotNameMask):
   connection_(0), received(), isRecording(false),
   robotName(robotNameMask.first.first), robotPort(robotNameMask.first.second) {
   this->mask = robotNameMask.second;
   this->naoData.setPaused(false);
   isAlive = true;
}

NetworkReader::NetworkReader(std::pair<std::pair<const QString &, int>, OffNaoMask_t> robotNameMask,
                             const NaoData &naoData) :
   Reader(naoData), connection_(0), received(), isRecording(false),
   robotName(robotNameMask.first.first), robotPort(robotNameMask.first.second) {
   this->mask = robotNameMask.second;
   this->naoData.setPaused(false);
   isAlive = true;
}

NetworkReader::~NetworkReader() {
   if (isRecording) disconnect();
   isAlive = false;
}

// Handle completion of a connect operation.
void NetworkReader::handle_connect(const boost::system::error_code& e,
      boost::asio::ip::tcp::resolver::iterator endpoint_iterator) {
   qDebug("Connected!");
   if (Thread::name == NULL) {
      Thread::name = "NetworkReader";
   }

   if (!e) {
      /* Successfully established connection. Start operation to read the list
       * of Blackboards. The connection::async_read() function will
       * automatically decode the data that is read from the underlying socket.
       */
      llog(DEBUG) << "NetworkReader: Connecting via async_read" << std::endl;
      received.blackboard = new Blackboard(config);
      connection_->async_read((ProtobufSerialisable&)*received.blackboard,
            boost::bind(&NetworkReader::handle_read, this,
               boost::asio::placeholders::error));

      write(mask);
      Q_EMIT showMessage(QString("Connected! Now streaming"));
   } else if (endpoint_iterator != boost::asio::ip::tcp::resolver::iterator()) {
      // Try the next endpoint.
      llog(DEBUG) << "NetworkReader: Trying next end-point" << std::endl;
      connection_->socket().close();
      boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator;
      connection_->socket().async_connect(endpoint,
            boost::bind(&NetworkReader::handle_connect, this,
               boost::asio::placeholders::error, ++endpoint_iterator));
   } else {
      /* An error occurred. Log it and return. Since we are not starting a new
       * operation the io_service async_read will run out of work to do and the
       * wirelessClient will exit.
       */
      std::cerr << e.message() << std::endl;
      Q_EMIT showMessage(QString::fromStdString(e.message()));
   }
}

// Handle completion of a read operation.
void NetworkReader::handle_read(const boost::system::error_code& e) {
   if (Thread::name == NULL) {
      Thread::name = "NetworkReader";
   }
   if (!isAlive) {
      return;  // this class has already been destroyed.
   }
   if (!e) {
      static int counter = 0;
      static Timer t;
      Q_EMIT showMessage(QString("average ms per packets: ") +
            QString::number(t.elapsed_ms()/++counter));

      try{
         naoData.appendFrame(received);
         if(!naoData.getIsPaused()) {
            naoData.setCurrentFrame(naoData.getFramesTotal()-1);
         }
         // TODO(jayen): breaks over midnight
         static uint64_t lastnew = 0;
         struct timeval now;
         gettimeofday(&now, NULL);
         uint64_t now2 = now.tv_sec * 1000000ull + now.tv_usec;
         if (now2 >= lastnew + 10000) {
            Q_EMIT newNaoData(&naoData);
            lastnew = now2;
         }
         received.blackboard = new Blackboard(config);
         connection_->async_read((ProtobufSerialisable&)*received.blackboard,
                                 boost::bind(&NetworkReader::handle_read, this,
                                             boost::asio::placeholders::error));
      } catch(boost::system::system_error &se) {
         qDebug() << "Error in receiving wireless data. " <<
               se.what() << endl;
         Q_EMIT showMessage(se.what());
         // disconnect();
      }
   } else {
      qDebug() << "Error in receiving wireless data. " << e.message().c_str() << endl;
      Q_EMIT showMessage(e.message().c_str());
   }
}

void NetworkReader::run()  {
   if (Thread::name == NULL) {
      Thread::name = "NetworkReader";
   }

   int currentFrame = 0;
   Q_EMIT showMessage(
      tr("Started session with nao. Hit record to begin stream...")   
   );

   while (isAlive) {
      currentFrame = naoData.getCurrentFrameIndex();
      if (currentFrame != naoData.getFramesTotal() - 1) {
         msleep(250);
      } else {
         msleep(200);
      }
   }
   Q_EMIT newNaoData(NULL);
}

void NetworkReader::stopMediaTrigger() {
   if (isRecording) {
      isRecording = disconnect();
   }
   naoData.setPaused(true);
   Q_EMIT showMessage(QString("Disconnected. Hit record to continue."));
}

void NetworkReader::recordMediaTrigger() {
   if (!isRecording) {
      isRecording = connect();
   }
   naoData.setPaused(false);
}

bool NetworkReader::disconnect() {
   llog(INFO) << "Try to disconnect!!" << std::endl;
   try {
      if (ioservice)
         ioservice->stop();

      if (cthread) {
         cthread->join();
         delete cthread;
         cthread = NULL;
      }

      if (connection_ && connection_->socket().is_open())
         connection_->socket().close();

      if (connection_) {
         delete connection_;
         connection_ = NULL;
      }
      if (ioservice) {
         delete ioservice;
         ioservice = NULL;
      }
      if (resolver) {
         delete resolver;
         resolver = NULL;
      }
      if (query) {
         delete query;
         query = NULL;
      }
   } catch(boost::system::system_error &se) {
      Q_EMIT showMessage(QString("Could not disconnect to robot!"));
   }

   return false;
}

bool NetworkReader::connect() {
   llog(INFO) << "Try to connect!" << std::endl;
   try {
      ioservice = new boost::asio::io_service();
      connection_ = new Connection(ioservice);
      resolver = new boost::asio::ip::tcp::resolver(*ioservice);
      std::stringstream ss;
      ss << robotPort;

      query = new boost::asio::ip::tcp::resolver::query(robotName.toStdString(), ss.str());
      boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver->resolve(*query);
      boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator;

      connection_->socket().async_connect(endpoint, 
         boost::bind(&NetworkReader::handle_connect, 
                     this,
                     boost::asio::placeholders::error, 
                     ++endpoint_iterator)
      );
      
      cthread = new boost::thread(boost::bind(&boost::asio::io_service::run, ioservice));
      llog(INFO) << "Connected!" << std::endl;
   } catch (boost::system::system_error &se) {
      Q_EMIT showMessage(QString("Could not connect to robot!"));
      llog(ERROR) << "ERROR CODE: " << se.what() << std::endl;
      return false;
   }
   return true;
}

void NetworkReader::write(const msg_t &msg) {
   if (connection_ && connection_->socket().is_open()) {
      ioservice->post(boost::bind(&NetworkReader::do_write, this, msg));
   }
}

void NetworkReader::do_write(msg_t msg) {
   bool write_in_progress = !write_msgs_.empty();
   write_msgs_.push_back(msg);
   if (!write_in_progress) {
      boost::asio::async_write(connection_->socket(),
            boost::asio::buffer(&(write_msgs_.front()),
               sizeof(write_msgs_.front())),
            boost::bind(&NetworkReader::handle_write, this,
               boost::asio::placeholders::error));
   }
}

void NetworkReader::handle_write(const boost::system::error_code &error) {
   if (!error) {
      if (!write_msgs_.empty()) {
         boost::asio::async_write(connection_->socket(),
               boost::asio::buffer(&(write_msgs_.front()),
                  sizeof(write_msgs_.front())),
               boost::bind(&NetworkReader::handle_write, this,
                  boost::asio::placeholders::error));
         write_msgs_.pop_front();
      }
   } else {
      do_close();
   }
}

void NetworkReader::do_close() {
   connection_->socket().close();
}

void NetworkReader::sendCameraSettings(int whichCamera, QSharedPointer<CameraSettings> settings) {
   OffNaoMask_t sendingMask = TO_NAO_MASKS | CAMERA_SETTINGS_MASK;
   llog(INFO) << "OffNao sending with mask " << std::bitset<sizeof(sendingMask) * 8>(sendingMask)
              << " - camera settings: " << CameraInfo::enumCameraToString((CameraInfo::Camera) whichCamera) << std::endl;
   llog(INFO) << "\t hflip = " << settings->hflip << std::endl;
   llog(INFO) << "\t vflip = " << settings->vflip << std::endl;
   for (int name = 0; name != CameraSettings::CameraSetting::NUM_CAMERA_SETTINGS; ++name) {
      llog(INFO) << "\t " << CameraSettings::enumToString((CameraSettings::CameraSetting) name) << " = " 
                 << settings->settings[name] << std::endl;
   }
   llog(INFO) << "\t whitebalance = " << settings->whiteBalance << std::endl;

   // Create data structure to serialise
   CameraSettingsSerialse csSerialise;
   csSerialise.whichCamera = (CameraInfo::Camera) whichCamera;
   csSerialise.settings = *settings;

   if (connection_ && connection_->socket().is_open()) {
      boost::asio::write(connection_->socket(),
                         boost::asio::buffer(&sendingMask, sizeof(sendingMask)));
      connection_->sync_write((ProtobufSerialisable&) csSerialise);
   }
}

void NetworkReader::sendCommandLineString(QString item) {
   OffNaoMask_t sendingMask = TO_NAO_MASKS | STRING_MASK;
   llog(INFO) << "OffNao sending with mask " << std::bitset<sizeof(sendingMask) * 8>(sendingMask)
              << " - string to: " << item.toStdString() << std::endl;

   if (connection_ && connection_->socket().is_open()) {
      boost::asio::write(connection_->socket(),
                         boost::asio::buffer(&sendingMask, sizeof(sendingMask)));
      connection_->sync_write(item.toStdString());
   }
}
