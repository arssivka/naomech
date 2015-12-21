//
// Created by arssivka on 11/23/15.
//

#ifndef NAOMECH_LED_H
#define NAOMECH_LED_H


#include <rd/representation/SensorData.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <alproxies/dcmproxy.h>
#include <alproxies/almemoryproxy.h>


namespace rd {
    class LEDs {
    public:
        enum Key {
            CHEST_BOARD_RED,
            CHEST_BOARD_GREEN,
            CHEST_BOARD_BLUE,
            L_EAR_0_DEG,
            L_EAR_108_DEG,
            L_EAR_144_DEG,
            L_EAR_180_DEG,
            L_EAR_216_DEG,
            L_EAR_252_DEG,
            L_EAR_288_DEG,
            L_EAR_324_DEG,
            L_EAR_36_DEG,
            L_EAR_72_DEG,
            R_EAR_0_DEG,
            R_EAR_108_DEG,
            R_EAR_144_DEG,
            R_EAR_180_DEG,
            R_EAR_216_DEG,
            R_EAR_252_DEG,
            R_EAR_288_DEG,
            R_EAR_324_DEG,
            R_EAR_36_DEG,
            R_EAR_72_DEG,
            L_FACE_RED_0_DEG,
            L_FACE_GREEN_0_DEG,
            L_FACE_BLUE_0_DEG,
            L_FACE_RED_135_DEG,
            L_FACE_GREEN_135_DEG,
            L_FACE_BLUE_135_DEG,
            L_FACE_RED_180_DEG,
            L_FACE_GREEN_180_DEG,
            L_FACE_BLUE_180_DEG,
            L_FACE_RED_225_DEG,
            L_FACE_GREEN_225_DEG,
            L_FACE_BLUE_225_DEG,
            L_FACE_RED_270_DEG,
            L_FACE_GREEN_270_DEG,
            L_FACE_BLUE_270_DEG,
            L_FACE_RED_315_DEG,
            L_FACE_GREEN_315_DEG,
            L_FACE_BLUE_315_DEG,
            L_FACE_RED_45_DEG,
            L_FACE_GREEN_45_DEG,
            L_FACE_BLUE_45_DEG,
            L_FACE_RED_90_DEG,
            L_FACE_GREEN_90_DEG,
            L_FACE_BLUE_90_DEG,
            R_FACE_RED_0_DEG,
            R_FACE_GREEN_0_DEG,
            R_FACE_BLUE_0_DEG,
            R_FACE_RED_135_DEG,
            R_FACE_GREEN_135_DEG,
            R_FACE_BLUE_135_DEG,
            R_FACE_RED_180_DEG,
            R_FACE_GREEN_180_DEG,
            R_FACE_BLUE_180_DEG,
            R_FACE_RED_225_DEG,
            R_FACE_GREEN_225_DEG,
            R_FACE_BLUE_225_DEG,
            R_FACE_RED_270_DEG,
            R_FACE_GREEN_270_DEG,
            R_FACE_BLUE_270_DEG,
            R_FACE_RED_315_DEG,
            R_FACE_GREEN_315_DEG,
            R_FACE_BLUE_315_DEG,
            R_FACE_RED_45_DEG,
            R_FACE_GREEN_45_DEG,
            R_FACE_BLUE_45_DEG,
            R_FACE_RED_90_DEG,
            R_FACE_GREEN_90_DEG,
            R_FACE_BLUE_90_DEG,
            L_HEAD_FRONT_0,
            L_HEAD_FRONT_1,
            R_HEAD_FRONT_0,
            R_HEAD_FRONT_1,
            L_HEAD_MIDDLE_0,
            R_HEAD_MIDDLE_0,
            L_HEAD_REAR_0,
            L_HEAD_REAR_1,
            L_HEAD_REAR_2,
            R_HEAD_REAR_0,
            R_HEAD_REAR_1,
            R_HEAD_REAR_2,
            L_FOOT_RED,
            L_FOOT_GREEN,
            L_FOOT_BLUE,
            R_FOOT_RED,
            R_FOOT_GREEN,
            R_FOOT_BLUE,
            LEDS_COUNT
        };

        LEDs(boost::shared_ptr<AL::ALBroker> broker);

        const std::vector<std::string> &getKeys() const;

        bool setBrightness(const std::vector<std::string> &keys,
                           const std::vector<double> &values);

        bool setBrightness(const std::vector<int> &keys,
                           const std::vector<double> &values);

        boost::shared_ptr<SensorData<double> > getBrightness(const std::vector<int> &keys);

        boost::shared_ptr<SensorData<double> > getBrightness(const std::vector<keys> &keys);

        boost::shared_ptr<SensorData<double> > getBrightness();

    private:
        boost::shared_ptr<AL::ALMemoryProxy> mem;
        boost::shared_ptr<AL::DCMProxy> dcm;

        boost::mutex synch;
        AL::ALValue cmd;

        std::vector<std::string> keys;
        std::vector<std::string> leds_list;
        std::map<std::string, std::string> leds_map;
    };
}


#endif //NAOMECH_LED_H
