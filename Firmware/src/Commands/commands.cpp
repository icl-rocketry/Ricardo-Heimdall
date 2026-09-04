/**
 * @file commands.cpp
 * @author Kiran de Silva (kd619@ic.ac.uk)
 * @brief Implementation of commands for system
 * @version 0.1
 * @date 2023-06-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "commands.h"

#include <librnp/rnp_packet.h>
#include <libriccore/commands/commandhandler.h>

#include "Commands/packets/GregTelemPacket.h"

#include "system.h"
#include "Commands/packets/processedsensorpacket.h"
#include "Commands/packets/rawADCPacket.h"

void Commands::FreeRamCommand(System& sm, const RnpPacketSerialized& packet)
{

	// ESP_LOGI("ch", "%s", "deserialize");

	SimpleCommandPacket commandpacket(packet);

	uint32_t freeram = esp_get_free_heap_size();
	//avliable in all states
	//returning as simple string packet for ease
	//currently only returning free ram
	if (commandpacket.arg == 0){
	MessagePacket_Base<0,static_cast<uint8_t>(decltype(System::commandhandler)::PACKET_TYPES::MESSAGE_RESPONSE)> message("FreeRam: " + std::to_string(esp_get_free_heap_size()));
	// this is not great as it assumes a single command handler with the same service ID
	// would be better if we could pass some context through the function paramters so it has an idea who has called it
	// or make it much clearer that only a single command handler should exist in the system
		message.header.source_service = sm.commandhandler.getServiceID();
		message.header.destination_service = packet.header.source_service;
		message.header.source = packet.header.destination;
		message.header.destination = packet.header.source;
		message.header.uid = packet.header.uid;
		sm.networkmanager.sendPacket(message);
	}
	else if (commandpacket.arg == 1)
	{
		BasicDataPacket<uint32_t,0,105> responsePacket(freeram);
		responsePacket.header.source_service = sm.commandhandler.getServiceID();
		responsePacket.header.destination_service = packet.header.source_service;
		responsePacket.header.source = packet.header.destination;
		responsePacket.header.destination = packet.header.source;
		responsePacket.header.uid = packet.header.uid;
		sm.networkmanager.sendPacket(responsePacket);
	}

}


void Commands::TelemetryCommand(System& sm, const RnpPacketSerialized& packet)
{
	SimpleCommandPacket commandpacket(packet);

	GregTelemPacket gregTelem;

	gregTelem.header.type = 103;
	gregTelem.header.source = sm.networkmanager.getAddress();
	gregTelem.header.source_service = sm.commandhandler.getServiceID();
	gregTelem.header.destination = commandpacket.header.source;
	gregTelem.header.destination_service = commandpacket.header.source_service;
	gregTelem.header.uid = commandpacket.header.uid;
	gregTelem.system_time = millis();

	gregTelem.FF_angle = sm.Heimdall.getFF();
	gregTelem.fuel_tankP = sm.FB_PT.getProcessed();
	gregTelem.n2_tankP = sm.N2_PT.getProcessed();
	gregTelem.regAngle = sm.Heimdall.getRegAngle();
	gregTelem.Kp = sm.Heimdall.getKp();
	gregTelem.tc0= sm.TC0.getTemp();
	gregTelem.tc1 = sm.TC1.getTemp();
	gregTelem.system_status = sm.systemstatus.getStatus();

	sm.networkmanager.sendPacket(gregTelem);
}

void Commands::rawADCCommand(System& sm, const RnpPacketSerialized& packet)
{
	SimpleCommandPacket commandpacket(packet);

	RawADCPacket rawSensors;

	rawSensors.header.type = 104;
	rawSensors.header.source = sm.networkmanager.getAddress();
	rawSensors.header.source_service = sm.commandhandler.getServiceID();
	rawSensors.header.destination = commandpacket.header.source;
	rawSensors.header.destination_service = commandpacket.header.source_service;
	rawSensors.header.uid = commandpacket.header.uid;
	rawSensors.system_time = millis();

	rawSensors.ch0 = sm.ADC0.getOutput(0);
	rawSensors.ch1 = sm.ADC0.getOutput(1);
	rawSensors.ch2 = sm.ADC0.getOutput(2);
	rawSensors.ch3 = sm.ADC0.getOutput(3);

	rawSensors.system_status = sm.systemstatus.getStatus();

	sm.networkmanager.sendPacket(rawSensors);
}