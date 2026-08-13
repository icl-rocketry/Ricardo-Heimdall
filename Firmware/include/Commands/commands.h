#pragma once
#include <stdint.h>

#include <librnp/rnp_packet.h>
#include <libriccore/commands/commandhandler.h>

#include "Config/forward_decl.h"
#include "Commands/packets/HeimdallTelemPacket.h"

// NB: do not include system.h here - system.h includes this header (via
// commands_config.h), so pulling it in creates an include cycle. Command
// signatures use ForwardDecl_SystemClass; .cpp files include system.h.

namespace Commands{
    
    void FreeRamCommand(ForwardDecl_SystemClass& system, const RnpPacketSerialized& packet);

}