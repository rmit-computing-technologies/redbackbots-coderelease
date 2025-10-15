#include "communication/Connection.hpp"

Connection::Connection(boost::asio::io_service* io_service) :
      socket_(*io_service), curr_buffer_size_(0), inbound_compressed_data_(), inbound_data_() {}

boost::asio::ip::tcp::socket& Connection::socket() {
   return socket_;
}

void Connection::clear_buffer() {
    clear_buffer(0);
}

void Connection::clear_buffer(int i) {
    if (i < OUTBOUND_BUFFER_SIZE) {
        buffered_outbound_data_[i].clear();
        clear_buffer(i + 1);
        curr_buffer_size_ = std::max(0, curr_buffer_size_ - 1);
    }
}

boost::system::error_code Connection::sync_send(std::string& data) {
   // Compress it
   size_t compressedSize;
   char* compressedBuffer = new char[snappy::MaxCompressedLength(data.size())];
   snappy::RawCompress(
      data.c_str(), data.size(),
      compressedBuffer, &compressedSize
   );
   // llog(DEBUG) << "sync_send: compressed size: " << compressedSize << std::endl;
   
   // Format the header.
   std::ostringstream header_stream;
   header_stream << std::setw(kHeaderLength) << std::hex << compressedSize
                 << std::setw(kHeaderLength) << data.size();
   if (!header_stream || header_stream.str().size() != kHeaderLength * 2) {
      // Something went wrong, inform the caller.
      llog(ERROR) << "Header only contains \"" << header_stream.str() 
                  << "\"" << std::endl;
      boost::system::error_code error(boost::asio::error::invalid_argument);
      return error;
   }
   outbound_header_ = header_stream.str();

   // Write the serialized data to the socket. We use "gather-write" to send
   // both the header and the data in a single write operation.
   std::vector<boost::asio::const_buffer> buffers;
   buffers.push_back(boost::asio::buffer(outbound_header_));
   if (compressedSize) {
      buffers.push_back(boost::asio::buffer(compressedBuffer, compressedSize));
   } else {
      buffers.push_back(boost::asio::buffer(data));
   }
   
   try {
      boost::asio::write(socket_, buffers);
      delete [] compressedBuffer;
   } catch(const std::exception & e) {
      if (strcmp(e.what(), "write: Broken pipe") == 0) {
         std::cout << "[You can safely ignore this, a client disconnected "
                   << "in the middle of sending data - "
                   << "broken_pipe exception caught: "
                   << e.what() << "]" << std::endl;
         return boost::system::errc::make_error_code(
            boost::system::errc::broken_pipe
         );
      }
      throw;
   }
   return boost::system::errc::make_error_code(boost::system::errc::success);
}

Connection::ErrorCategory::ErrorCategory(const char *name) : _name(name) {}
const char* Connection::ErrorCategory::name() const BOOST_SYSTEM_NOEXCEPT {return _name.data();}
std::string Connection::ErrorCategory::message(int ev) const {return _name + ": " + strerror(ev);}

template<>
void Connection::serialise<std::string>(const std::string &ps, std::ostream &os) {
   os << ps;
}

template<>
void Connection::serialise<ProtobufSerialisable>(const ProtobufSerialisable &ps, std::ostream &os) {
   ps.serialise(os);
}

template<>
std::string &Connection::deserialise<std::string>(std::string &ps, std::istream &is) {
   getline(is, ps);
   return ps;
}

template<>
ProtobufSerialisable &Connection::deserialise<ProtobufSerialisable>(ProtobufSerialisable &ps, std::istream &is) {
   ps.deserialise(is);
   return ps;
}
