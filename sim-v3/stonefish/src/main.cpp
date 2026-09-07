#include "VehicleAManager.h"

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <core/ConsoleSimulationApp.h>
#include <sensors/Sensor.h>

class BridgeApp final : public sf::ConsoleSimulationApp {
public:
    BridgeApp(const std::string& data_path, VehicleAManager* manager,
              unsigned int max_physics_threads)
        : ConsoleSimulationApp("Stonefish Vehicle A Bridge", data_path,
                               manager),
          manager_(manager) {
        if (max_physics_threads > 0) setMaxPhysicsThreads(max_physics_threads);
    }

protected:
    void LoopInternal() override {
        std::string line;
        if (!std::getline(std::cin, line)) {
            Quit();
            return;
        }

        std::istringstream command(line);
        std::string operation;
        command >> operation;
        if (operation == "RESET") {
            std::uint32_t seed = 0;
            sf::Scalar gps_z = -0.5;
            std::string scenario = "normal";
            if (!(command >> seed)) {
                ReplyError("RESET requires an unsigned seed");
                return;
            }
            if (command >> gps_z) {}
            if (command >> scenario) {}
            if (scenario != "normal" && scenario != "grounding" &&
                scenario != "object_collision") {
                ReplyError("unknown RESET scenario");
                return;
            }
            if (getState() == sf::SimulationState::RUNNING) StopSimulation();
            sf::Sensor::SetRandomSeed(seed);
            manager_->SetGpsHeight(gps_z);
            manager_->SetScenarioMode(scenario);
            manager_->RestartScenario();
            StartSimulation();
            Reply("reset", 0);
        } else if (operation == "STEP") {
            sf::Scalar port = 0.0;
            sf::Scalar starboard = 0.0;
            unsigned int count = 0;
            if (!(command >> port >> starboard >> count) || count == 0) {
                ReplyError("STEP requires port starboard positive_step_count");
                return;
            }
            if (getState() != sf::SimulationState::RUNNING) {
                ReplyError("RESET must be called before STEP");
                return;
            }
            manager_->SetCommands(std::clamp(port, sf::Scalar(-1), sf::Scalar(1)),
                                  std::clamp(starboard, sf::Scalar(-1), sf::Scalar(1)));
            for (unsigned int index = 0; index < count; ++index) {
                StepSimulation();
            }
            Reply("step", count);
        } else if (operation == "QUIT") {
            if (getState() == sf::SimulationState::RUNNING) StopSimulation();
            std::cout << "{\"ok\":true,\"operation\":\"quit\"}" << std::endl;
            Quit();
        } else {
            ReplyError("unknown operation");
        }
    }

private:
    void Reply(const char* operation, unsigned int steps) const {
        std::cout << std::setprecision(17)
                  << "{\"ok\":true,\"operation\":\"" << operation
                  << "\",\"steps\":" << steps
                  << ",\"simulation_time\":"
                  << manager_->getSimulationTime()
                  << ",\"observation\":" << manager_->ObservationJson()
                  << ",\"diagnostics\":{\"port_thrust\":"
                  << manager_->GetPortThrust()
                  << ",\"starboard_thrust\":"
                  << manager_->GetStarboardThrust()
                  << ",\"contacts\":" << manager_->ContactJson()
                  << "}}" << std::endl;
    }

    static void ReplyError(const char* message) {
        std::cout << "{\"ok\":false,\"error\":\"" << message << "\"}"
                  << std::endl;
    }

    VehicleAManager* manager_;
};

int main(int argc, char** argv) {
    if (argc < 2 || argc > 4) {
        std::cerr << "usage: stonefish_vehicle_a_bridge STONEFISH_TEST_DATA [MAX_PHYSICS_THREADS_OR_0] [SENSOR_NOISE_0_OR_1]"
                  << std::endl;
        return 2;
    }
    const unsigned int max_physics_threads =
        argc >= 3 ? static_cast<unsigned int>(std::stoul(argv[2])) : 1U;
    const bool sensor_noise = argc >= 4 && std::string(argv[3]) == "1";
    auto* manager = new VehicleAManager(argv[1], sensor_noise);
    BridgeApp app(argv[1], manager, max_physics_threads);
    app.Run(false, false, VehicleAManager::kFixedTimeStep);
    delete manager;
    return 0;
}
