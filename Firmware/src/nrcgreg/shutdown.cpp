#include "shutdown.h"

#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include <librrc/Local/remoteactuatoradapter.h>

#include "system.h"


Shutdown::Shutdown(Greg::DefaultStateInit& DefaultInitParams, RnpNetworkManager &networkmanager):
State(GREG_FLAGS::STATE_SHUTDOWN,DefaultInitParams.gregstatus),
m_regAdapter(DefaultInitParams.regAdapter),
m_regClosedAngle(DefaultInitParams.regClosedAngle),
m_DefaultInitParams(DefaultInitParams),
m_networkmanager(networkmanager)
{};

void Shutdown::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!

    m_regAdapter.arm(0); //Arm the servo
    m_regAdapter.execute(m_regClosedAngle); //Drive the E-Reg to its closed position.
    m_regAdapter.disarm(); //No reason to keep actuator armed

    m_DefaultInitParams.Greg.buckOff(2000);

    static constexpr uint8_t STARK_ADDDRESS = 10;

    // Send shutdown command to the engine
    SimpleCommandPacket stark_shutdown(2, 2);
    stark_shutdown.header.source_service = static_cast<uint8_t>(Services::ID::Heimdall);
    stark_shutdown.header.destination_service = 10;
    stark_shutdown.header.source = 1;
    stark_shutdown.header.destination = STARK_ADDDRESS;
    stark_shutdown.header.uid = 0;
    m_networkmanager.sendPacket(stark_shutdown);

    static constexpr uint8_t OTHER_EREG_ADDRESS = 12;

    // Send shutdown command to the other ereg
    SimpleCommandPacket ereg_shutdown(2, 2);
    ereg_shutdown.header.source_service = static_cast<uint8_t>(Services::ID::Heimdall);
    ereg_shutdown.header.destination_service = 10;
    ereg_shutdown.header.source = 1;
    ereg_shutdown.header.destination = OTHER_EREG_ADDRESS;
    ereg_shutdown.header.uid = 0;
    m_networkmanager.sendPacket(ereg_shutdown);
};

Types::EREGTypes::State_ptr_t Shutdown::update()
{
    return nullptr; //Remain in default state indefinitely. The transition away from default is accessed through the actuator command handler.
};

void Shutdown::exit()
{
    Types::EREGTypes::State_t::exit(); // call parent exit last!
};