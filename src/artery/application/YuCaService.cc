/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "artery/application/CaObject.h"
#include "artery/application/YuCaService.h"
#include "artery/application/YuStorage.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>


namespace artery
{

using namespace omnetpp;

namespace yu {
	auto microdegree = vanetza::units::degree * boost::units::si::micro;
	auto decidegree = vanetza::units::degree * boost::units::si::deci;
	auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
	auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
static const simsignal_t scSignalCamSent = cComponent::registerSignal("CamSent");
static const auto scLowFrequencyContainerInterval = std::chrono::milliseconds(500);

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}




SpeedValue_t buildSpeedValue(const vanetza::units::Velocity& v)
{
	static const vanetza::units::Velocity lower { 0.0 * boost::units::si::meter_per_second };
	static const vanetza::units::Velocity upper { 163.82 * boost::units::si::meter_per_second };

	SpeedValue_t speed = SpeedValue_unavailable;
	if (v >= upper) {
		speed = 16382; // see CDD A.74 (TS 102 894 v1.2.1)
	} else if (v >= lower) {
		speed = yu::round(v, yu::centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
	}
	return speed;
}

void addLowFrequencyContainer(vanetza::asn1::Cam& message, unsigned pathHistoryLength)
{
	if (pathHistoryLength > 40) {
		EV_WARN << "path history can contain 40 elements at maximum";
		pathHistoryLength = 40;
	}

	LowFrequencyContainer_t*& lfc = message->cam.camParameters.lowFrequencyContainer;
	lfc = vanetza::asn1::allocate<LowFrequencyContainer_t>();
	lfc->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;
	BasicVehicleContainerLowFrequency& bvc = lfc->choice.basicVehicleContainerLowFrequency;
	bvc.vehicleRole = VehicleRole_default;
	bvc.exteriorLights.buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
	assert(nullptr != bvc.exteriorLights.buf);
	bvc.exteriorLights.size = 1;
	bvc.exteriorLights.buf[0] |= 1 << (7 - ExteriorLights_daytimeRunningLightsOn);

	for (unsigned i = 0; i < pathHistoryLength; ++i) {
		PathPoint* pathPoint = vanetza::asn1::allocate<PathPoint>();
		pathPoint->pathDeltaTime = vanetza::asn1::allocate<PathDeltaTime_t>();
		*(pathPoint->pathDeltaTime) = 0;
		pathPoint->pathPosition.deltaLatitude = DeltaLatitude_unavailable;
		pathPoint->pathPosition.deltaLongitude = DeltaLongitude_unavailable;
		pathPoint->pathPosition.deltaAltitude = DeltaAltitude_unavailable;
		ASN_SEQUENCE_ADD(&bvc.pathHistory, pathPoint);
	}

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid Low Frequency CAM: %s", error.c_str());
	}
}


vanetza::asn1::Cam createCooperativeAwarenessMessage(const VehicleDataProvider& vdp, uint16_t genDeltaTime)
{
	vanetza::asn1::Cam message;

	ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 2;
	header.messageID = ItsPduHeader__messageID_cam;
	header.stationID = vdp.station_id();

	CoopAwareness_t& cam = (*message).cam;
	cam.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
	BasicContainer_t& basic = cam.camParameters.basicContainer;
	HighFrequencyContainer_t& hfc = cam.camParameters.highFrequencyContainer;

	basic.stationType = StationType_passengerCar;
	basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
	basic.referencePosition.longitude = yu::round(vdp.longitude(), yu::microdegree) * Longitude_oneMicrodegreeEast;
	basic.referencePosition.latitude = yu::round(vdp.latitude(), yu::microdegree) * Latitude_oneMicrodegreeNorth;
	basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
			SemiAxisLength_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
			SemiAxisLength_unavailable;

	hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
	BasicVehicleContainerHighFrequency& bvc = hfc.choice.basicVehicleContainerHighFrequency;
	bvc.heading.headingValue = yu::round(vdp.heading(), yu::decidegree);
	bvc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
	bvc.speed.speedValue = yu::buildSpeedValue(vdp.speed());
	bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
	bvc.driveDirection = vdp.speed().value() >= 0.0 ?
			DriveDirection_forward : DriveDirection_backward;
	const double lonAccelValue = vdp.acceleration() / vanetza::units::si::meter_per_second_squared;
	// extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well)
	if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0) {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
	} else {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
	}
	bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
	bvc.curvature.curvatureValue = abs(vdp.curvature() / vanetza::units::reciprocal_metre) * 10000.0;
	if (bvc.curvature.curvatureValue >= 1023) {
		bvc.curvature.curvatureValue = 1023;
	}
	bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
	bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;
	bvc.yawRate.yawRateValue = yu::round(vdp.yaw_rate(), yu::degree_per_second) * YawRateValue_degSec_000_01ToLeft * 100.0;
	if (bvc.yawRate.yawRateValue < -32766 || bvc.yawRate.yawRateValue > 32766) {
		bvc.yawRate.yawRateValue = YawRateValue_unavailable;
	}
	bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
	bvc.vehicleLength.vehicleLengthConfidenceIndication =
			VehicleLengthConfidenceIndication_noTrailerPresent;
	bvc.vehicleWidth = VehicleWidth_unavailable;

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid High Frequency CAM: %s", error.c_str());
	}

	return message;
}


}

Define_Module(YuCaService)

YuCaService::YuCaService() :
		mGenCamMin { 100, SIMTIME_MS },
		mGenCamMax { 1000, SIMTIME_MS },
		mGenCam(mGenCamMax),
		mGenCamLowDynamicsCounter(0),
		mGenCamLowDynamicsLimit(3),
		storage_()
{
}

YuCaService::~YuCaService(){
	sended_.close();
	received_.close();
	positioned_.close();
}

void YuCaService::initialize()
{
	ItsG5BaseService::initialize();
	mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
	mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
	mTimer = &getFacilities().get_const<Timer>();
	mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();

	// avoid unreasonable high elapsed time values for newly inserted vehicles
	mLastCamTimestamp = simTime();

	// first generated CAM shall include the low frequency container
	mLastLowCamTimestamp = mLastCamTimestamp - artery::simtime_cast(yu::scLowFrequencyContainerInterval);

	// generation rate boundaries
	mGenCamMin = par("minInterval");
	mGenCamMax = par("maxInterval");
	mGenCam = mGenCamMax;

	// vehicle dynamics thresholds
	mHeadingDelta = vanetza::units::Angle { par("headingDelta").doubleValue() * vanetza::units::degree };
	mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
	mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;

	mDccRestriction = par("withDccRestriction");
	mFixedRate = par("fixedRate");

    lastCAMAcceleration = AccelerationXY{
        .x = 0.0,
        .y = 0.0
    };

    lastCAMVelocity = VelocityXY{
        .x = 0.0,
        .y = 0.0
    };

    lastCAMPosition = PositionXYOwn{
        .x = 0.0,
        .y = 0.0
    };

	// look up primary channel for CA
	mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::CA);
	// Setting the files
	std::string sended_str = std::to_string(mVehicleDataProvider->station_id())+"_sent.csv";
	std::string received_str = std::to_string(mVehicleDataProvider->station_id())+"_received.csv";
	std::string positioned_str = std::to_string(mVehicleDataProvider->station_id())+"_positioned.csv";
	sended_.open(sended_str.c_str(), std::ofstream::out | std::ofstream::app);
	received_.open(received_str.c_str(), std::ofstream::out | std::ofstream::app);
	positioned_.open(positioned_str.c_str(), std::ofstream::out | std::ofstream::app);
	sended_ << "Sent Time,Simulation Time,Origin Latitude,Origin Longitude,X,Y" << std::endl;
	received_ << "Station ID,Sent Time,Received Time,Received Simulation Time,Origin Latitude,Origin Longitude,Destiny Latitude,Destiny Longitude,Destiny X,Destiny Y" << std::endl;
	positioned_ << "Simulation Time,Delta Time,Predicted Position X,Predicted Position Y,Real Postition X,Real Position Y,Real Position Latitude,Real Position Longitude,Predicted Speed X,Predicted Speed Y,Real Speed X,Real Speed Y,Real Acceleration X,Real Acceleration Y,L2 Norm,CAM Sent" << std::endl;
}

void YuCaService::trigger()
{
	Enter_Method("trigger");
	checkTriggeringConditions(simTime());
}

void YuCaService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
	Enter_Method("indicate");

	Asn1PacketVisitor<vanetza::asn1::Cam> visitor;
	const vanetza::asn1::Cam* cam = boost::apply_visitor(visitor, *packet);
	if (cam && cam->validate()) {
		CaObject obj = visitor.shared_wrapper;
		emit(yu::scSignalCamReceived, &obj);
		mLocalDynamicMap->updateAwareness(obj);
		// Dump the results of the received CAM
		Position current_pos = mVehicleDataProvider->position();
		int latitude = yu::round(mVehicleDataProvider->latitude(), yu::microdegree) * Longitude_oneMicrodegreeEast;
		int longitude = yu::round(mVehicleDataProvider->longitude(), yu::microdegree) * Latitude_oneMicrodegreeNorth;
		auto current_time = omnetpp::SimTime();
		uint16_t receivedDeltaTimeMod = countTaiMilliseconds(mTimer->getCurrentTime());
		received_ << (*cam)->header.stationID << ",";
		received_ << (*cam)->cam.generationDeltaTime << ",";
		received_ << receivedDeltaTimeMod << ",";
		received_ << current_time.dbl() << ",";
		received_ << std::to_string((*cam)->cam.camParameters.basicContainer.referencePosition.latitude) << ",";
		received_ << std::to_string((*cam)->cam.camParameters.basicContainer.referencePosition.longitude)  << ",";
		received_ << std::to_string(latitude) << ",";
		received_ << std::to_string(longitude) << ",";
		received_ << std::to_string(mVehicleDataProvider->position().x/boost::units::si::meter) << ",";
		received_ << std::to_string(mVehicleDataProvider->position().y/boost::units::si::meter);
		received_ << std::endl;

		storage_.new_cam((*(*cam)),latitude, longitude, receivedDeltaTimeMod);
		
	}
}

void YuCaService::checkTriggeringConditions(const SimTime& T_now)
{
	// provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
	SimTime& T_GenCam = mGenCam;
	const SimTime& T_GenCamMin = mGenCamMin;
	const SimTime& T_GenCamMax = mGenCamMax;
	const SimTime T_GenCamDcc = mDccRestriction ? genCamDcc() : T_GenCamMin;
	const SimTime T_elapsed = T_now - mLastCamTimestamp;
    double seconds_elapsed = T_elapsed.dbl();
	// T_elapsed.inUnit(SimTimeUnit::SIMTIME_MS).dbl();
	AccelerationXY current_acceleration = AccelerationXY{
        .x = (mVehicleDataProvider->acceleration() / vanetza::units::si::meter_per_second_squared)*std::cos(mLastCamHeading.value()),
        .y = -1*(mVehicleDataProvider->acceleration() / vanetza::units::si::meter_per_second_squared)*std::sin(mLastCamHeading.value())
    };
    PositionXYOwn predicted_position = {
        .x = lastCAMPosition.x+lastCAMVelocity.x*seconds_elapsed+0.5*lastCAMAcceleration.x*(seconds_elapsed*seconds_elapsed),
        .y = lastCAMPosition.y+lastCAMVelocity.y*seconds_elapsed+0.5*lastCAMAcceleration.y*(seconds_elapsed*seconds_elapsed),
    };
    PositionXYOwn current_position{
        .x = mVehicleDataProvider->position().x/boost::units::si::meter,
        .y = mVehicleDataProvider->position().y/boost::units::si::meter
    };
    PositionXYOwn differ_position{
        .x = (predicted_position.x-current_position.x)*(predicted_position.x-current_position.x),
        .y = (predicted_position.y-current_position.y)*(predicted_position.y-current_position.y)
    };
    VelocityXY predicted_velocity{
	.x = lastCAMVelocity.x+lastCAMAcceleration.x*seconds_elapsed,
	.y = lastCAMVelocity.y+lastCAMAcceleration.y*seconds_elapsed
    };
    VelocityXY current_velocity = VelocityXY{
        .x = (mVehicleDataProvider->speed() / vanetza::units::si::meter_per_second)*std::cos(mVehicleDataProvider->heading().value()),
        .y = -1*(mVehicleDataProvider->speed() / vanetza::units::si::meter_per_second)*std::sin(mVehicleDataProvider->heading().value())
    };
    VelocityXY differ_velocity = VelocityXY{
        .x = (predicted_velocity.x-current_velocity.x)*(predicted_velocity.x-current_velocity.x),
        .y = (predicted_velocity.y-current_velocity.y)*(predicted_velocity.y-current_velocity.y)
    };

    
    // double l2_norm = std::sqrt(differ_velocity.x+differ_velocity.y+differ_position.x+differ_position.y);
	double local_paoi = std::sqrt(differ_position.x+differ_position.y);
    int cam_sent = 0;
	uint16_t receivedDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(T_now));
	double penalty = storage_.get_penalties(receivedDeltaTimeMod);
	// double real_penalty = penalty;
	// penalty = penalty/100;
	// if(penalty>6){
	// 	penalty = 6.0;
	// }
	float alpha = 0.9;
	double final_value = alpha*local_paoi+(1-alpha)*penalty;
	if(final_value>6)
	{
		sendCam(T_now);
	 	cam_sent = 1;
	}

	int latitude = yu::round(mVehicleDataProvider->latitude(), yu::microdegree) * Longitude_oneMicrodegreeEast;
	int longitude = yu::round(mVehicleDataProvider->longitude(), yu::microdegree) * Latitude_oneMicrodegreeNorth;
	positioned_ << T_now << ",";
	positioned_ << T_elapsed << ",";
	positioned_ << predicted_position.x << ",";
	positioned_ << predicted_position.y << ",";
	positioned_ << current_position.x << ",";
	positioned_ << current_position.y << ",";
	positioned_ << latitude << ",";
	positioned_ << longitude << ",";
	positioned_ << predicted_velocity.x << ",";
	positioned_ << predicted_velocity.y << ",";
	positioned_ << current_velocity.x << ",";
	positioned_ << current_velocity.y << ",";
	positioned_ << current_acceleration.x << ",";
	positioned_ << current_acceleration.y << ",";
	positioned_ << final_value << ",";
	positioned_ << cam_sent << std::endl;
}

bool YuCaService::checkHeadingDelta() const
{
	return !vanetza::facilities::similar_heading(mLastCamHeading, mVehicleDataProvider->heading(), mHeadingDelta);
}

bool YuCaService::checkPositionDelta() const
{
	return (distance(mLastCamPosition, mVehicleDataProvider->position()) > mPositionDelta);
}

bool YuCaService::checkSpeedDelta() const
{
	return abs(mLastCamSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
}

void YuCaService::sendCam(const SimTime& T_now)
{
	uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
	auto cam = createCooperativeAwarenessMessage(*mVehicleDataProvider, genDeltaTimeMod);

	mLastCamPosition = mVehicleDataProvider->position();
	mLastCamSpeed = mVehicleDataProvider->speed();
	mLastCamHeading = mVehicleDataProvider->heading();
    mLastCamAcceleration = mVehicleDataProvider->acceleration();
    lastCAMPosition = PositionXYOwn{
        .x = mLastCamPosition.x/boost::units::si::meter,
        .y = mLastCamPosition.y/boost::units::si::meter
    };
    lastCAMVelocity = VelocityXY{
        .x = (mLastCamSpeed / vanetza::units::si::meter_per_second)*std::cos(mLastCamHeading.value()),
        .y = -1*(mLastCamSpeed / vanetza::units::si::meter_per_second)*std::sin(mLastCamHeading.value())
    };

    lastCAMAcceleration = AccelerationXY{
        .x = (mLastCamAcceleration / vanetza::units::si::meter_per_second_squared)*std::cos(mLastCamHeading.value()),
        .y = -1*(mLastCamAcceleration / vanetza::units::si::meter_per_second_squared)*std::sin(mLastCamHeading.value())
    };

	mLastCamTimestamp = T_now;
	if (T_now - mLastLowCamTimestamp >= artery::simtime_cast(yu::scLowFrequencyContainerInterval)) {
		yu::addLowFrequencyContainer(cam, par("pathHistoryLength"));
		mLastLowCamTimestamp = T_now;
	}

	//Dump the data of the sent message
	int latitude = yu::round(mVehicleDataProvider->latitude(), yu::microdegree) * Longitude_oneMicrodegreeEast;
	int longitude = yu::round(mVehicleDataProvider->longitude(), yu::microdegree) * Latitude_oneMicrodegreeNorth;
	long timestamp = genDeltaTimeMod;
	sended_ << timestamp << ",";
	sended_ << T_now << ",";
	sended_ << std::to_string(latitude) << ",";
	sended_ << std::to_string(longitude) << ",";
	sended_ << std::to_string(mVehicleDataProvider->position().x/boost::units::si::meter) << ",";
	sended_ << std::to_string(mVehicleDataProvider->position().y/boost::units::si::meter);
	sended_ << std::endl;

	using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::CAM;
	request.gn.its_aid = aid::CA;
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

	CaObject obj(std::move(cam));
	emit(yu::scSignalCamSent, &obj);

	using CamByteBuffer = convertible::byte_buffer_impl<asn1::Cam>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new CamByteBuffer(obj.shared_ptr()) };
	payload->layer(OsiLayer::Application) = std::move(buffer);
	this->request(request, std::move(payload));
}

SimTime YuCaService::genCamDcc()
{
	// network interface may not be ready yet during initialization, so look it up at this later point
	auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for CA's primary channel %i", mPrimaryChannel);
	}

	static const vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
	vanetza::Clock::duration interval = trc->interval(ca_tx);
	SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
	return std::min(mGenCamMax, std::max(mGenCamMin, dcc));
}

} // namespace artery
