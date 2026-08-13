#include "Commands/packets/HeimdallTelemPacket.h"

HeimdallTelemPacket::~HeimdallTelemPacket()
{};

HeimdallTelemPacket::HeimdallTelemPacket():
RnpPacket(0,
          104,
          size())
{};

HeimdallTelemPacket::HeimdallTelemPacket(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void HeimdallTelemPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};