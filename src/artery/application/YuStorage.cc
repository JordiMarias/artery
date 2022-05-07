#include "artery/application/YuStorage.h"
#include <GeographicLib/UTMUPS.hpp>
# include <cmath>
#include <random>
#include <limits>


namespace artery {
YuStorage::YuStorage()
{
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> distr(0, 1000); // define the range
    int separate = distr(gen);
//     std::string trace = std::to_string(separate)+"_trace.txt";
//     trace_.open(trace.c_str(), std::ofstream::out | std::ofstream::app);
//     trace_.precision(std::numeric_limits<double>::max_digits10);
}

YuStorage::~YuStorage()
{
    // trace_.close();
}


void YuStorage::new_cam(const CAM_t& received_cam, const int& lat, const int& lon, const uint16_t& received_time)
{
    // trace_ << "New CAM arrived" << std::endl;
    double penalty = 10000;
    if (last_cams_.find(received_cam.header.stationID)!= last_cams_.end())
    {
        // trace_ << "There was a previous cam" << std::endl;
        const CAM_t& last_position = last_cams_[received_cam.header.stationID];
        // Compute the penalty
        penalty = compute_penalty(received_cam, last_position);
        // trace_ << "Penalty computed: "<< penalty << std::endl;
    }
    // trace_ << "Copiing the new cam" << std::endl;
    last_penalty_[received_cam.header.stationID] = penalty;
    CAM_t* solution = (CAM_t*)vanetza::asn1::copy(asn_DEF_CAM, (const void *)(&received_cam));
    last_cams_[received_cam.header.stationID] = *solution;

    double temp_lat = lat;
    temp_lat = temp_lat/10000000;
    double temp_lon = lon;
    temp_lon = temp_lon/10000000;
    double current_x, current_y;
    to_x_and_y(temp_lat, temp_lon, current_x, current_y);
    double temp_lat2 = received_cam.cam.camParameters.basicContainer.referencePosition.latitude;
    temp_lat2 = temp_lat2/10000000;
    double temp_lon2 = received_cam.cam.camParameters.basicContainer.referencePosition.longitude;
    temp_lon2 = temp_lon2/10000000;
    double received_x, received_y;
    to_x_and_y(temp_lat2, temp_lon2, received_x, received_y);
    // trace_ << "Computing and storing the distance" << std::endl;
    double distance = std::sqrt((current_x-received_x)*(current_x-received_x)+
        (current_y-received_y)*(current_y-received_y));
    // trace_ << "    raw Own lat:" << temp_lat << std::endl;
    // trace_ << "    raw Own lon:" << temp_lon << std::endl;
    // trace_ << "    raw Received lat:" << temp_lat2 << std::endl;
    // trace_ << "    raw Received lon:" << temp_lon2 << std::endl;
    // trace_ << "    Own lat:" << current_x << std::endl;
    // trace_ << "    Own lon:" << current_y << std::endl;
    // trace_ << "    Received lat:" << received_x << std::endl;
    // trace_ << "    Received lon:" << received_y << std::endl;
    // trace_ << distance << std::endl;
    distances_[received_cam.header.stationID] = distance;
}

double YuStorage::get_penalties(const uint16_t& current_time)
{
    // trace_ << "Asking for penalties" << std::endl;
    double penalties = 0;
    if(distances_.size() != 0){
        // trace_ << "Other vehicles sensed" << std::endl;
        penalties = 0;
        std::map<unsigned long, double> normalized_weighs;
        double total_weighs = 0;
        for (std::pair<unsigned long, double> element : distances_) {
            normalized_weighs[element.first] = calculate_weight(
                last_cams_[element.first],
                current_time
            );
            total_weighs += normalized_weighs[element.first];
        }
        if (total_weighs>0){
            // trace_ << "New bunch of weights" << std::endl;
            for (std::pair<unsigned long, double> element : distances_) {
                normalized_weighs[element.first] = normalized_weighs[element.first]/total_weighs;
            }
            for (std::pair<unsigned long, double> element : distances_) {
                // trace_ << "Weight: " << std::to_string(normalized_weighs[element.first]) << std::endl;
                penalties += normalized_weighs[element.first]*last_penalty_[element.first];
            }
        }else{
            penalties = 0;
        }
        // trace_ << "Sum of penalties and weights computed: "<< penalties << std::endl;
    }
    // trace_ << "Returning penalties: "<< penalties << std::endl;
    return penalties;
}

double YuStorage::compute_penalty(const CAM_t& received_cam, const CAM_t& last_cam)
{
    // Received cam -> The one that just got received
    // Last CAM -> The cam that was stored
    long time_difference = received_cam.cam.generationDeltaTime-last_cam.cam.generationDeltaTime;
    if (time_difference<0){
        time_difference = (received_cam.cam.generationDeltaTime+65536)-last_cam.cam.generationDeltaTime;
    }
    //long time_of_interest = time_difference/2;
    // Converting 
    double temp_lat = received_cam.cam.camParameters.basicContainer.referencePosition.latitude;
    // trace_ << "    current lat raw:" << temp_lat << std::endl;
    temp_lat = temp_lat/10000000;
    double temp_lon = received_cam.cam.camParameters.basicContainer.referencePosition.longitude;
    // trace_ << "    current lon raw:" << temp_lon << std::endl;
    temp_lon = temp_lon/10000000;
    double last_x, last_y;
    to_x_and_y(temp_lat, temp_lon, last_x, last_y);
    double temp_lat2 = last_cam.cam.camParameters.basicContainer.referencePosition.latitude;
    // trace_ << "    last lat raw:" << temp_lat2 << std::endl;
    temp_lat2 = temp_lat2/10000000;
    double temp_lon2 = last_cam.cam.camParameters.basicContainer.referencePosition.longitude;
    // trace_ << "    last lon raw:" << temp_lat2 << std::endl;
    temp_lon2 = temp_lon2/10000000;
    double current_x, current_y;
    to_x_and_y(temp_lat2, temp_lon2, current_x, current_y);
    // trace_ << "    Computing penalty:" << std::endl;
    // trace_ << "    Time difference: " << time_difference <<std::endl;
    // trace_ << "    current lat:" << temp_lat << std::endl;
    // trace_ << "    current lon:" << temp_lon << std::endl;
    // trace_ << "    last lat:" << temp_lat2 << std::endl;
    // trace_ << "    last lon:" << temp_lon2 << std::endl;
    // trace_ << "    current x:" << current_x << std::endl;
    // trace_ << "    current y:" << current_y << std::endl;
    // trace_ << "    last x:" << last_x << std::endl;
    // trace_ << "    last y:" << last_y << std::endl;
    float pi = 3.1415926;
    double time_passed = time_difference/1000;
    long last_acceleration = received_cam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue;
    double last_heading = received_cam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue/10;
    double last_speed = received_cam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue/100;
    double last_speed_x, last_speed_y, last_accel_x, last_accel_y, predicted_x, predicted_y, predicted_speed_x, predicted_speed_y;
    last_speed_x = last_speed*std::cos(last_heading*(pi/360));
    last_speed_y = last_speed*std::sin(last_heading*(pi/360));
    last_accel_x = last_acceleration*std::cos(last_heading*(pi/360));
    last_accel_y = last_acceleration*std::sin(last_heading*(pi/360));
    predicted_x = last_x+last_speed_x*time_passed+0.5*last_accel_x*time_passed*time_passed;
    predicted_y = last_y+last_speed_y*time_passed+0.5*last_accel_y*time_passed*time_passed;
    predicted_speed_x = last_speed_x*time_passed;
    predicted_speed_y = last_speed_y*time_passed;
    long current_heading = received_cam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue/10;
    double current_speed = received_cam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue/100;
    double current_speed_x = current_speed*std::cos(current_heading*(pi/360));
    double current_speed_y = current_speed*std::sin(current_heading*(pi/360));
    // double penalty = std::sqrt(
    //     (predicted_x-current_x)*(predicted_x-current_x)+
    //     (predicted_y-current_y)*(predicted_y-current_y)+
    //     (predicted_speed_x-current_speed_x)*(predicted_speed_x-current_speed_x)+
    //     (predicted_speed_y-current_speed_y)*(predicted_speed_y-current_speed_y)
    // );
    double penalty = std::sqrt(
        (predicted_x-current_x)*(predicted_x-current_x)+
        (predicted_y-current_y)*(predicted_y-current_y)
    );
    // trace_ << "    predicted x:" << predicted_x << std::endl;
    // trace_ << "    predicted y:" << predicted_y << std::endl;
    return penalty;
}

double YuStorage::calculate_weight(const CAM_t& last_cam, const uint16_t& current_time){
    
    // trace_ << "CALCULATING WEIGHT"<< std::endl;
    double distance = distances_[last_cam.header.stationID];
    double highest_distance = get_highest_distance();
    // trace_ << "    Distance: " << distance << std::endl;
    // trace_ << "    Highest Distance: " << highest_distance << std::endl;

    long time_difference = current_time-last_cam.cam.generationDeltaTime;
    if (time_difference<0){
        time_difference = (current_time+65536)-last_cam.cam.generationDeltaTime;
    }
    double time_difference_seconds = time_difference/1000;
    double meaningful_time = to_time_sigmoid(time_difference_seconds);
    double meaningful_distance = 1;
    if (highest_distance!=0 && highest_distance!=distance){
        meaningful_distance = (highest_distance-distance)/highest_distance;
    }
    // trace_ << "    Meaningful time: " << meaningful_time << std::endl;
    // trace_ << "    Meaningful Distance: " << meaningful_distance << std::endl;

    return meaningful_distance*meaningful_time;
}

double YuStorage::to_time_sigmoid(long time)
{
    return 1/(1+std::exp(-(0.2*((time/1000)-15))));
}

void YuStorage::to_x_and_y(const double& lat, const double& lon, double& x, double& y){
    int zone;
    bool northp;
    double last_x, last_y;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
}

double YuStorage::get_highest_distance()
{
    double highest_distance = 0;
    for (std::pair<unsigned long, double> element : distances_) {
        if(element.second>highest_distance){
            highest_distance = element.second;
        }
    }
    return highest_distance;
}

}