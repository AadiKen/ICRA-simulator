#include "VehicleAManager.h"

#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>

#include <actuators/ActuatorDynamics.h>
#include <actuators/Thruster.h>
#include <core/FeatherstoneRobot.h>
#include <entities/solids/Polyhedron.h>
#include <entities/statics/Obstacle.h>
#include <entities/statics/Plane.h>
#include <sensors/Contact.h>
#include <sensors/Sample.h>
#include <sensors/scalar/Compass.h>
#include <sensors/scalar/GPS.h>
#include <sensors/scalar/IMU.h>
#include <sensors/ScalarSensor.h>
#include <utils/UnitSystem.h>

VehicleAManager::VehicleAManager(const std::string& data_path,
                                 bool sensor_noise)
    : SimulationManager(kStepsPerSecond, sf::Solver::SI,
                        sf::CollisionFilter::EXCLUSIVE),
      data_path_(data_path),
      sensor_noise_(sensor_noise),
      gps_z_ned_(-0.5),
      scenario_mode_("normal"),
      initial_north_(0.0),
      initial_east_(0.0),
      initial_yaw_(0.0),
      port_(nullptr),
      starboard_(nullptr),
      gps_(nullptr),
      imu_(nullptr),
      compass_(nullptr),
      grounding_contact_(nullptr),
      object_contact_(nullptr) {}

void VehicleAManager::BuildScenario() {
    CreateMaterial("VehicleAFiberglass",
                   sf::UnitSystem::Density(sf::CGS, sf::MKS, 1.5), 0.3);
    SetMaterialsInteraction("VehicleAFiberglass", "VehicleAFiberglass", 0.5,
                            0.2);
    EnableOcean(0.0);

    sf::PhysicsSettings hull_physics;
    hull_physics.mode = sf::PhysicsMode::FLOATING;
    hull_physics.collisions = true;
    hull_physics.buoyancy = true;
    auto* hull = new sf::Polyhedron(
        "VehicleAHull", hull_physics, data_path_ + "/boat_gra.obj", 1.0,
        sf::I4(), data_path_ + "/boat.obj", 1.0, sf::I4(),
        "VehicleAFiberglass", "", 0.09);
    hull->SetArbitraryPhysicalProperties(
        55.0, sf::Vector3(15.0, 20.0, 30.0), sf::I4());

    sf::PhysicsSettings prop_physics;
    prop_physics.mode = sf::PhysicsMode::SUBMERGED;
    prop_physics.collisions = false;
    prop_physics.buoyancy = false;

    auto make_thruster = [&](const std::string& name, bool right_hand) {
        auto propeller = std::make_shared<sf::Polyhedron>(
            name + "Propeller", prop_physics, data_path_ + "/propeller.obj",
            1.0, sf::I4(), "VehicleAFiberglass", "");
        auto rotor = std::make_shared<sf::FirstOrder>(kRotorTimeConstant);
        auto conversion = std::make_shared<sf::InterpolatedThrust>(
            std::vector<sf::Scalar>{-kThrustLimit, 0.0, kThrustLimit},
            std::vector<sf::Scalar>{-kThrustLimit, 0.0, kThrustLimit});
        const bool inverted_setpoint = !right_hand;
        return new sf::Thruster(name, propeller, rotor, conversion, 0.18,
                                right_hand, kThrustLimit, inverted_setpoint,
                                true);
    };

    port_ = make_thruster("PortThruster", true);
    starboard_ = make_thruster("StarboardThruster", false);
    gps_ = new sf::GPS("GPS");
    imu_ = new sf::IMU("IMU");
    compass_ = new sf::Compass("Compass");
    if (sensor_noise_) {
        gps_->setNoise(0.02);
        imu_->setNoise(sf::Vector3(0.001, 0.001, 0.001),
                       sf::Vector3(0.001, 0.001, 0.001), 0.0,
                       sf::Vector3(0.01, 0.01, 0.01));
        compass_->setNoise(0.001);
    }

    auto* vehicle = new sf::FeatherstoneRobot("VehicleA");
    vehicle->DefineLinks(hull);
    vehicle->BuildKinematicStructure();
    constexpr sf::Scalar propeller_z = 0.30;
    vehicle->AddLinkActuator(
        port_, "VehicleAHull",
        sf::Transform(sf::IQ(), sf::Vector3(-0.8, -kThrusterOffset,
                                            propeller_z)));
    vehicle->AddLinkActuator(
        starboard_, "VehicleAHull",
        sf::Transform(sf::IQ(), sf::Vector3(-0.8, kThrusterOffset,
                                            propeller_z)));
    vehicle->AddLinkSensor(
        gps_, "VehicleAHull",
        sf::Transform(sf::IQ(), sf::Vector3(0.0, 0.0, gps_z_ned_)));
    vehicle->AddLinkSensor(imu_, "VehicleAHull", sf::I4());
    vehicle->AddLinkSensor(compass_, "VehicleAHull", sf::I4());
    AddRobot(vehicle,
             sf::Transform(sf::Quaternion(initial_yaw_, 0.0, 0.0),
                           sf::Vector3(initial_north_, initial_east_, 0.0)));

    // Native Bullet contacts are kept separate by counterpart so grounding
    // is never silently folded into generic object collision.
    auto* bottom = new sf::Plane("Seabed", 2000.0, "VehicleAFiberglass");
    const sf::Scalar bottom_z = scenario_mode_ == "grounding" ? 0.0 : 20.0;
    AddStaticEntity(bottom,
                    sf::Transform(sf::IQ(), sf::Vector3(0.0, 0.0, bottom_z)));
    auto* obstacle = new sf::Obstacle(
        "TestObject", sf::Vector3(2.0, 2.0, 2.0), sf::I4(),
        "VehicleAFiberglass");
    const sf::Scalar obstacle_north =
        scenario_mode_ == "object_collision" ? 0.0 : 1000.0;
    AddStaticEntity(
        obstacle,
        sf::Transform(sf::IQ(), sf::Vector3(obstacle_north, 0.0, 0.0)));
    grounding_contact_ = new sf::Contact("HullSeabed", hull, bottom, 1);
    object_contact_ = new sf::Contact("HullObject", hull, obstacle, 1);
    AddContact(grounding_contact_);
    AddContact(object_contact_);
}

void VehicleAManager::SetCommands(sf::Scalar port, sf::Scalar starboard) {
    port_->setSetpoint(port);
    starboard_->setSetpoint(starboard);
}

void VehicleAManager::SetGpsHeight(sf::Scalar z_ned) {
    gps_z_ned_ = z_ned;
}

void VehicleAManager::SetScenarioMode(const std::string& mode) {
    scenario_mode_ = mode;
}

void VehicleAManager::SetInitialPose(sf::Scalar north, sf::Scalar east,
                                     sf::Scalar yaw) {
    initial_north_ = north;
    initial_east_ = east;
    initial_yaw_ = yaw;
}

std::string VehicleAManager::SampleArray(const sf::ScalarSensor* sensor) {
    const auto values = sensor->getLastSample().getData();
    std::ostringstream out;
    out << std::setprecision(17) << '[';
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index != 0) out << ',';
        out << values[index];
    }
    out << ']';
    return out.str();
}

std::string VehicleAManager::ObservationJson() const {
    const auto gps_sample = gps_->getLastSample();
    const auto gps_values = gps_sample.getData();
    const bool fix_valid = gps_sample.getTimestamp() >= 0.0 &&
                           gps_values.size() == 4 &&
                           gps_values[0] <= 90.0 &&
                           gps_values[1] <= 180.0;
    std::ostringstream out;
    out << std::setprecision(17)
        << "{\"gps\":" << SampleArray(gps_)
        << ",\"fix_valid\":" << (fix_valid ? "true" : "false")
        << ",\"imu\":" << SampleArray(imu_)
        << ",\"compass\":" << SampleArray(compass_) << '}';
    return out.str();
}

std::string VehicleAManager::ContactJson() const {
    const bool grounding = grounding_contact_ != nullptr &&
                           !grounding_contact_->getHistory().empty();
    const bool object_collision = object_contact_ != nullptr &&
                                  !object_contact_->getHistory().empty();
    std::ostringstream out;
    out << "{\"grounding\":" << (grounding ? "true" : "false")
        << ",\"object_collision\":"
        << (object_collision ? "true" : "false") << '}';
    return out.str();
}

sf::Scalar VehicleAManager::GetPortThrust() const {
    return port_->getThrust();
}

sf::Scalar VehicleAManager::GetStarboardThrust() const {
    return starboard_->getThrust();
}
