#pragma once

#include <cstdint>
#include <string>

#include <core/SimulationManager.h>

namespace sf {
class Compass;
class Contact;
class GPS;
class IMU;
class ScalarSensor;
class Thruster;
}

class VehicleAManager final : public sf::SimulationManager {
public:
    static constexpr sf::Scalar kStepsPerSecond = 500.0;
    static constexpr sf::Scalar kFixedTimeStep = 1.0 / kStepsPerSecond;
    static constexpr sf::Scalar kThrusterOffset = 0.395;
    static constexpr sf::Scalar kThrustLimit = 95.0;
    static constexpr sf::Scalar kRotorTimeConstant = 0.25;

    explicit VehicleAManager(const std::string& data_path,
                             bool sensor_noise = false);

    void BuildScenario() override;
    void SetCommands(sf::Scalar port, sf::Scalar starboard);
    void SetGpsHeight(sf::Scalar z_ned);
    void SetScenarioMode(const std::string& mode);
    void SetInitialPose(sf::Scalar north, sf::Scalar east, sf::Scalar yaw);

    std::string ObservationJson() const;
    std::string ContactJson() const;
    sf::Scalar GetPortThrust() const;
    sf::Scalar GetStarboardThrust() const;

private:
    static std::string SampleArray(const sf::ScalarSensor* sensor);

    std::string data_path_;
    bool sensor_noise_;
    sf::Scalar gps_z_ned_;
    std::string scenario_mode_;
    sf::Scalar initial_north_;
    sf::Scalar initial_east_;
    sf::Scalar initial_yaw_;
    sf::Thruster* port_;
    sf::Thruster* starboard_;
    sf::GPS* gps_;
    sf::IMU* imu_;
    sf::Compass* compass_;
    sf::Contact* grounding_contact_;
    sf::Contact* object_contact_;
};
