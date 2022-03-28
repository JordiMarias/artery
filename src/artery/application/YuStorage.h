#ifndef YU_STORAGE_H_
#define YU_STORAGE_H_
#include <map>
#include <vanetza/asn1/cam.hpp>
#include <iostream>
#include <fstream>


namespace artery {
class YuStorage{
    public:
        YuStorage();
        ~YuStorage();

        /* Adds a new position and computes the penalty
         *
         */
        void new_cam(const CAM_t& received_cam, const int& lat, const int& lon, const uint16_t& received_time);
        double get_penalties(const uint16_t& current_time);
    private:
        double compute_penalty(const CAM_t& received_cam, const CAM_t& last_cam);
        double calculate_weight(const CAM_t& current_cam, const uint16_t&  current_time);
        void to_x_and_y(const double& lat, const double& lon, double& x, double& y);
        double get_highest_distance();

        // Returns a sigmoid that gets 1 
        double to_time_sigmoid(long time);


        std::map<unsigned long, CAM_t> last_cams_;
        std::map<unsigned long, double> distances_;
        std::map<unsigned long, double> last_penalty_;
        std::ofstream trace_;



};
}
#endif
