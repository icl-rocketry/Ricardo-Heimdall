#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

class ProcessedSensorPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &ProcessedSensorPacket::ch0sens,
                &ProcessedSensorPacket::ch1sens,
                &ProcessedSensorPacket::ch2sens,
                &ProcessedSensorPacket::ch3sens,
                &ProcessedSensorPacket::temp0,
                &ProcessedSensorPacket::temp1,
                &ProcessedSensorPacket::system_status,
                &ProcessedSensorPacket::system_time

            );
            return ret;
        }
        
    public:
        ~ProcessedSensorPacket();

        ProcessedSensorPacket();
        /**
         * @brief Deserialize Telemetry Packet
         * 
         * @param data 
         */
        ProcessedSensorPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Telemetry Packet
         * 
         * @param buf 
         */
        void serialize(std::vector<uint8_t>& buf) override;

        
        //packet header
        //PacketHeader header{static_cast<uint8_t>(packet::TELEMETRY), packet_size()};

        //ADC channels
        float ch0sens;
        float ch1sens;
        float ch2sens;
        float ch3sens;

        float temp0;
        float temp1;


        //system details
        uint32_t system_status;
        uint64_t system_time;


        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};


