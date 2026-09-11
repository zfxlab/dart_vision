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
  const auto partial=parser.nextFrame(); assert(partial.frame.empty()); parser.append(bytes.data()+4,8);
  const auto decoded=decodeReceivePacket(parser.nextFrame().frame);
  assert(decoded && decoded->target_id==4 && decoded->offset_rad==rx.offset_rad);
  bytes[3]^=1; assert(!decodeReceivePacket(bytes));
}
