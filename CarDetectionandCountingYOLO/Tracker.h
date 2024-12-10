#pragma once
#ifndef TRACKER_H
#define TRACKER_H
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

struct Vehicle {
    int id;
    cv::Rect boundingBox;
    int frameCount;
    bool counted;
    double avgSpeed;              // Ortalama hýz
    cv::Point2f predictedPos;     // Tahminli pozisyon
};

class Tracker {
public:
    Tracker();
    void update(const std::vector<cv::Rect>& detections);
    std::vector<Vehicle>& getVehicles();
private:
    int nextId;
    std::vector<Vehicle> vehicles;
    double calculateIoU(const cv::Rect& box1, const cv::Rect& box2);
    void predictNextPosition(Vehicle& vehicle);
};
#endif