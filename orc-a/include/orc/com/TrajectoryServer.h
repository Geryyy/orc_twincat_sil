#pragma once

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <ctime>
#include <iostream>
#include <string>
#include "orc/OrcTypes.h"
#include "orc/RobotTraits.h"
#include "orc/com/RobotState.h"
#include "orc/com/flatbuffers/FlatBufferDeserializer.h"
#include "orc/com/flatbuffers/FlatBufferSerializer.h"
#include "orc/robots/Robot.h"
#include "orc/trajectory/Trajectories.h"
#include "orc/util/Logger.h"

using boost::asio::ip::udp;

namespace orc::com {

// Hard wire limit. A single Ethernet frame is 1518 bytes; subtracting the
// MAC (14) + CRC (4) + IP (20) + UDP (8) headers leaves 1472 bytes of UDP
// payload before IP fragmentation kicks in. The TwinCAT real-time stack
// does NOT reassemble fragmented IP datagrams, so any larger payload would
// be silently dropped on the receive side. The pre-FlatBuffers serializer
// (TrajectorySerializer.h) used this limit to drive its automatic
// overlap-splitting algorithm; that algorithm was not ported to the
// FlatBuffers serializer (see TODO in send path below). Until it is,
// callers MUST keep their serialized buffers ≤ MAX_UDP_PAYLOAD.
constexpr size_t MAX_UDP_PAYLOAD = 1472;

template <typename RobotType, uint16_t SERVER_PORT, uint16_t CLIENT_PORT>
class TrajectoryServer {
    static constexpr int DOF = RobotType::DOF;
    using RobotState = orc::com::template RobotState<RobotType>;
    using FlatBufferDeserializer = orc::com::fb::FlatBufferDeserializer<DOF>;

    uint16_t _server_port;
    uint16_t _client_port;
    boost::asio::io_service io_service;
    udp::socket socket_;
    udp::endpoint remote_endpoint_;
    boost::array<uint8_t, MAX_UDP_PAYLOAD> recv_buffer_;
    std::shared_ptr<RobotType> contr_;
    FlatBufferDeserializer deserializer_;

public:
    TrajectoryServer(std::shared_ptr<RobotType> contr, uint16_t server_port = SERVER_PORT,
                     uint16_t client_port = CLIENT_PORT)
        : _server_port(server_port),
          _client_port(client_port),
          socket_(io_service, udp::endpoint(udp::v4(), server_port)),
          remote_endpoint_(boost::asio::ip::address::from_string("127.0.0.1"), client_port),
          contr_(contr) {
        orc::log::write_info("TrajectoryServer started");
        start_receive();
    }

    void run() { io_service.run(); }

    void poll() { io_service.poll(); }

    void send_robot_data(Time time) {
        // TODO: Zero-copy optimization — serialize() returns a std::vector<uint8_t> which is then
        //       copied into boost::asio::buffer. Consider using
        //       FlatBufferBuilder::GetBufferPointer() directly with boost::asio::buffer(ptr, size)
        //       to avoid the intermediate vector allocation. Alternatively, keep a persistent
        //       FlatBufferBuilder as a member and reuse its buffer.
        RobotState rs(*contr_, time);
        auto data = rs.serialize();
        if (data.size() > MAX_UDP_PAYLOAD) {
            // Larger payloads would be IP-fragmented and dropped silently on the
            // TwinCAT side. Surface the regression instead of corrupting the wire.
            orc::log::write_error("RobotState frame size " + std::to_string(data.size()) +
                                  " exceeds Ethernet UDP payload limit " +
                                  std::to_string(MAX_UDP_PAYLOAD) +
                                  " bytes; dropping send to avoid IP fragmentation");
            return;
        }
        remote_endpoint_.port(_client_port);
        socket_.send_to(boost::asio::buffer(data), remote_endpoint_);
    }

private:
    void start_receive() {
        socket_.async_receive_from(
            boost::asio::buffer(recv_buffer_), remote_endpoint_,
            boost::bind(&TrajectoryServer::handle_receive, this, boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }

    void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred) {
        if (!error && bytes_transferred > 0) {
            // Get trajectory type from FlatBuffer
            auto type_result =
                deserializer_.get_trajectory_type(recv_buffer_.data(), bytes_transferred);

            if (type_result.valid) {
                orc::log::write_debug("Received FlatBuffer trajectory type: " +
                                      std::to_string(static_cast<int>(type_result.type)));
                // TODO: Zero-copy optimization — add_trajectory_from_flatbuffer currently
                // deserializes
                //       the FlatBuffer into owned trajectory objects (copying all data). For
                //       trajectories that are consumed immediately or whose lifetime is bounded,
                //       consider keeping the recv_buffer_ alive and wrapping it in a zero-copy
                //       trajectory view. The FlatBuffer data in recv_buffer_ is already in the
                //       right format for direct access.
                contr_->add_trajectory_from_flatbuffer(recv_buffer_.data(), bytes_transferred);
            } else {
                orc::log::write_warning("Received invalid FlatBuffer message");
            }
        }

        start_receive();
    }
};

}  // namespace orc::com
