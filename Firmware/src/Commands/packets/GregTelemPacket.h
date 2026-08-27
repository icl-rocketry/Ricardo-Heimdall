#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

//shamelessly copied from pickle rick's telemetry packet

class GregTelemPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &GregTelemPacket::FF_angle,
                &GregTelemPacket::fuel_tankP,
                &GregTelemPacket::regAngle,
                &GregTelemPacket::P_angle,
                &GregTelemPacket::Kp,
                &GregTelemPacket::system_status,
                &GregTelemPacket::system_time
            );

            return ret;
        }
        
    public:
        ~GregTelemPacket();

        GregTelemPacket();
        /**
         * @brief Deserialize Telemetry Packet
         * 
         * @param data 
         */
        GregTelemPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Telemetry Packet
         * 
         * @param buf 
         */
        void serialize(std::vector<uint8_t>& buf) override;

        float FF_angle;
        float fuel_tankP;
        uint32_t regAngle;
        float P_angle;
        float Kp;
        uint32_t system_status;
        uint64_t system_time;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};


