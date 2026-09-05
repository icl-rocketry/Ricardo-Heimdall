#pragma once

#include <librnp/rnp_serializer.h>
#include <unistd.h>

class TelemetryLogframe{
private:
    static constexpr auto getSerializer()
    {
        auto ret = RnpSerializer(
            &TelemetryLogframe::cmdAngle,
            &TelemetryLogframe::ch0sens,
            &TelemetryLogframe::ch2sens,
            &TelemetryLogframe::temp0,
            &TelemetryLogframe::temp1,
            &TelemetryLogframe::timestamp
        );
        return ret;
    }

public:
    float cmdAngle;
    float ch0sens,ch2sens;
    float temp0,temp1;

    uint64_t timestamp;

    std::string stringify()const{
        return getSerializer().stringify(*this) + "\n";
    };

};
