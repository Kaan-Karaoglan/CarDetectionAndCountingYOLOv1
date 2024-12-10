#include "Tracker.h"
#include <algorithm>
#include <cmath>

Tracker::Tracker() : nextId(0) {}

double Tracker::calculateIoU(const cv::Rect& box1, const cv::Rect& box2) {
    auto intersection = box1 & box2;
    double intersectionArea = intersection.width * intersection.height;
    double unionArea = box1.width * box1.height + box2.width * box2.height - intersectionArea;
    return intersectionArea / unionArea;
}

void Tracker::predictNextPosition(Vehicle& vehicle) {
    // Basit lineer tahmin
    vehicle.predictedPos = cv::Point2f(
        vehicle.boundingBox.x + vehicle.boundingBox.width / 2.0,
        vehicle.boundingBox.y + vehicle.boundingBox.height / 2.0
    );
}

void Tracker::update(const std::vector<cv::Rect>& detections) {
    for (const auto& detection : detections) {
        bool matched = false;
        for (auto& vehicle : vehicles) {
            double iou = calculateIoU(detection, vehicle.boundingBox);
            double distance = cv::norm(
                cv::Point(detection.x, detection.y) -
                cv::Point(vehicle.boundingBox.x, vehicle.boundingBox.y)
            );

            if (iou > 0.3 || distance < 100) {
                vehicle.boundingBox = detection;
                vehicle.frameCount++;
                predictNextPosition(vehicle);
                matched = true;
                break;
            }
        }

        if (!matched) {
            vehicles.push_back({
                nextId++,
                detection,
                1,
                false,
                0.0,
                cv::Point2f(detection.x + detection.width / 2.0, detection.y + detection.height / 2.0)
                });
        }
    }

    // Dinamik araç temizleme
    vehicles.erase(
        std::remove_if(vehicles.begin(), vehicles.end(),
            [](const Vehicle& v) { return v.frameCount > 100; }),
        vehicles.end()
    );
}

std::vector<Vehicle>& Tracker::getVehicles() {
    return vehicles;
}