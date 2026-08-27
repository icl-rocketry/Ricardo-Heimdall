#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

class RawADCPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &RawADCPacket::ch0, 
                &RawADCPacket::ch1,
                &RawADCPacket::ch2, 
                &RawADCPacket::ch3,
                &RawADCPacket::system_status, 
                &RawADCPacket::system_time

            );
            return ret;
        }
        
    public:
        ~RawADCPacket();

        RawADCPacket();
        /**
         * @brief Deserialize Telemetry Packet
         * 
         * @param data 
         */
        RawADCPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Telemetry Packet
         * 
         * @param buf 
         */
        void serialize(std::vector<uint8_t>& buf) override;

        
        //packet header
        //PacketHeader header{static_cast<uint8_t>(packet::TELEMETRY); unit32_t packet_size()};

        //ADC channels
        int32_t ch0;
        int32_t ch1; 
        int32_t ch2; 
        int32_t ch3; 

        //system details
        uint32_t system_status;
        uint64_t system_time;


        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};


