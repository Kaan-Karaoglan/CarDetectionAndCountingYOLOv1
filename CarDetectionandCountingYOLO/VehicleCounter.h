#pragma once
#ifndef VEHICLE_COUNTER_H
#define VEHICLE_COUNTER_H

#include <string>

class VehicleCounter {
public:
    VehicleCounter(const std::string& inputFile, const std::string& outputFile);
    void run();

private:
    std::string inputFile;
    std::string outputFile;

    void processVideo();
    void writeResults(int totalVehicles, double avgFrameTime);
};

#endif // VEHICLE_COUNTER_H
