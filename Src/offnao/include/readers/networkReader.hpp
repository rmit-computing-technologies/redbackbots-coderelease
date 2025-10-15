#pragma once

#include "readers/reader.hpp"

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/thread.hpp>
#include <deque>

#include <QSharedPointer>

// base message type
using msg_t = int64_t;
using chat_message_queue = std::deque<msg_t>;

// Forward declarations
class CameraSettings;
class Connection;

/* A reader that connects with the nao and collects data which is
 * then stored in a naoData object.
 *
 */
class NetworkReader : public Reader {
Q_OBJECT

public:
    NetworkReader(const QString &robotName, int robotPort, OffNaoMask_t mask);
    NetworkReader(std::pair<std::pair<const QString &, int>, OffNaoMask_t> robotNameMask);
    NetworkReader(std::pair<std::pair<const QString &, int>,
                           OffNaoMask_t> robotNameMask,
                           const NaoData &naoData);
    virtual ~NetworkReader();

    // main loop that runs when the thread starts
    virtual void run();

    /* Writes a message to the nao */
    void write(const msg_t &msg);

private:
    OffNaoMask_t mask;
    boost::asio::io_service *ioservice;

    void handle_connect(const boost::system::error_code& e,
            boost::asio::ip::tcp::resolver::iterator endpoint_iterator);
    void handle_read(const boost::system::error_code& e);
    Connection *connection_;

    boost::thread *cthread;
    Frame received;

    boost::asio::ip::tcp::resolver *resolver;
    boost::asio::ip::tcp::resolver::query *query;

    bool isRecording;
    QString robotName;
    int robotPort;

    bool disconnect();
    bool connect();


    void do_write(msg_t msg);

    void handle_write(const boost::system::error_code &error);

    void do_close();

    chat_message_queue write_msgs_;

public Q_SLOTS:
    virtual void stopMediaTrigger();
    virtual void recordMediaTrigger();

    /**
     * Methods to send various content to the Nao
     */
    void sendCameraSettings(int whichCamera, QSharedPointer<CameraSettings> settings);
    void sendCommandLineString(QString item);
};
