#include "Commands/Packets/RawADCPacket.h"

RawADCPacket::~RawADCPacket()
{};

RawADCPacket::RawADCPacket():
RnpPacket(0,
          104,
          size())
{};

RawADCPacket::RawADCPacket(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void RawADCPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};