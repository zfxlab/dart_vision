#include <cassert>
#include <cstring>
#include <cmath>
#include <vector>
#include "dart_serial/packet.hpp"
#include "dart_serial/crc.hpp"
#include "dart_serial/packet_parser.hpp"
using namespace dart_vision::serial;
int main() {
  SendPacket send; send.state=2; send.yaw_rad=-0.125F; send.distance_m=25.2F;
  const auto out=encodeSendPacket(send);
  assert(out.size()==12 && out[0]==0xA5 && out[1]==2 && verifyCRC16(out.data(),out.size()));
  float yaw=0, distance=0; std::memcpy(&yaw,out.data()+2,4); std::memcpy(&distance,out.data()+6,4);
  assert(yaw==send.yaw_rad && distance==send.distance_m);
  ReceivePacket rx; rx.target_id=4; rx.offset_rad=0.03F; rx.yaw_rad=-0.14F;
  std::vector<std::uint8_t> bytes(sizeof(rx)); std::memcpy(bytes.data(),&rx,sizeof(rx));
  appendCRC16(bytes.data(),bytes.size());
  PacketParser parser; parser.append(bytes.data(),4); assert(parser.bufferedSize()==4);
  const auto partial=parser.nextFrame(); assert(partial.frame.empty());
  parser.append(bytes.data()+4,bytes.size()-4);
  const auto decoded=decodeReceivePacket(parser.nextFrame().frame);
  assert(decoded && decoded->target_id==4 && decoded->offset_rad==rx.offset_rad);
  bytes[3]^=1; assert(!decodeReceivePacket(bytes));

  LoggerPacket logger;
  logger.state=2; logger.prepare_state=1; logger.current_dart_id=3;
  logger.string_l_force=0.0507F; logger.string_r_force=0.0516F;
  std::vector<std::uint8_t> logger_bytes(sizeof(logger));
  std::memcpy(logger_bytes.data(),&logger,sizeof(logger));
  appendCRC16(logger_bytes.data(),logger_bytes.size());
  assert(packetTypeFromHeader(0xD5)==PacketType::kLogger);
  assert(packetSizeFromHeader(0xD5)==25 && verifyCRC16(logger_bytes.data(),logger_bytes.size()));

  std::vector<std::uint8_t> receive_bytes(sizeof(rx));
  std::memcpy(receive_bytes.data(),&rx,sizeof(rx));
  appendCRC16(receive_bytes.data(),receive_bytes.size());
  logger_bytes.insert(logger_bytes.end(),receive_bytes.begin(),receive_bytes.end());
  PacketParser mixed_parser;
  mixed_parser.append(logger_bytes.data(),7);
  assert(mixed_parser.nextFrame().status==ParseStatus::kNeedMoreData);
  mixed_parser.append(logger_bytes.data()+7,logger_bytes.size()-7);
  const auto logger_result=mixed_parser.nextFrame();
  assert(logger_result.status==ParseStatus::kFrameReady && logger_result.frame.size()==25);
  const auto decoded_logger=decodeLoggerPacket(logger_result.frame);
  assert(decoded_logger && decoded_logger->current_dart_id==3);
  assert(decoded_logger->string_l_force==logger.string_l_force);
  assert(mixed_parser.nextFrame().status==ParseStatus::kFrameReady);

  const std::vector<std::uint8_t> captured_logger{
    0xD5,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x01,0x00,0x00,0x00,0x00,
    0x01,0x00,0x00,0x00,0x00,0x00,0x17,0xD9,0x4E,0x3D,0xE3,0xEF};
  const auto captured=decodeLoggerPacket(captured_logger);
  assert(captured && captured->current_shot_number==1 && captured->current_dart_id==3);
  assert(captured->door_status==1 && captured->autoaim_allow==1);
}
