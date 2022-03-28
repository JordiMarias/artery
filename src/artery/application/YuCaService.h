/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef ARTERY_YuCaService_H_
#define ARTERY_YuCaService_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/YuStorage.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/cam.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <vanetza/units/acceleration.hpp>
#include <omnetpp/simtime.h>
#include <iostream>
#include <fstream>
#include <string>


namespace artery
{

struct VelocityXY {
    double x;
    double y;
};

struct AccelerationXY {
    double x;
    double y;
};

struct PositionXYOwn {
    double x;
    double y;
};

class NetworkInterfaceTable;
class Timer;
class VehicleDataProvider;

class YuCaService : public ItsG5BaseService
{
	public:
		YuCaService();
		~YuCaService();
		void initialize() override;
		void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
		void trigger() override;

	private:
		void checkTriggeringConditions(const omnetpp::SimTime&);
		bool checkHeadingDelta() const;
		bool checkPositionDelta() const;
		bool checkSpeedDelta() const;
		void sendCam(const omnetpp::SimTime&);
		omnetpp::SimTime genCamDcc();

		ChannelNumber mPrimaryChannel = channel::CCH;
		const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
		const VehicleDataProvider* mVehicleDataProvider = nullptr;
		const Timer* mTimer = nullptr;
		LocalDynamicMap* mLocalDynamicMap = nullptr;

		omnetpp::SimTime mGenCamMin;
		omnetpp::SimTime mGenCamMax;
		omnetpp::SimTime mGenCam;
		unsigned mGenCamLowDynamicsCounter;
		unsigned mGenCamLowDynamicsLimit;
		Position mLastCamPosition;
		vanetza::units::Velocity mLastCamSpeed;
		vanetza::units::Angle mLastCamHeading;
		vanetza::units::Acceleration mLastCamAcceleration;
		omnetpp::SimTime mLastCamTimestamp;
		omnetpp::SimTime mLastLowCamTimestamp;
		vanetza::units::Angle mHeadingDelta;
		vanetza::units::Length mPositionDelta;
		vanetza::units::Velocity mSpeedDelta;
		bool mDccRestriction;
		bool mFixedRate;
		PositionXYOwn lastCAMPosition;
        VelocityXY lastCAMVelocity;
        AccelerationXY lastCAMAcceleration;

		std::ofstream sended_;
		std::ofstream received_;
		std::ofstream positioned_;

		YuStorage storage_;
};

vanetza::asn1::Cam createCooperativeAwarenessMessage(const VehicleDataProvider&, uint16_t genDeltaTime);
void addLowFrequencyContainer(vanetza::asn1::Cam&, unsigned pathHistoryLength = 0);

} // namespace artery

#endif /* ARTERY_YuCaService_H_ */
