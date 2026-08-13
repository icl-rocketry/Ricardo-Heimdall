#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

class HeimdallTelemPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &HeimdallTelemPacket::servoVoltage,
                &HeimdallTelemPacket::system_status,
                &HeimdallTelemPacket::system_time
            );

            return ret;
        }
        
    public:
        ~HeimdallTelemPacket();

        HeimdallTelemPacket();
        /**
         * @brief Deserialize Telemetry Packet
         * 
         * @param data 
         */
        HeimdallTelemPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Telemetry Packet
         * 
         * @param buf 
         */
        void serialize(std::vector<uint8_t>& buf) override;

        float servoVoltage;
        uint32_t system_status;
        uint64_t system_time;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};


