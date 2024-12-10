#include "VehicleCounter.h"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include<opencv2/videoio.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/highgui/highgui_c.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include "Tracker.h"

// Ara� s�n�flar� i�in geni�letilmi� liste
std::vector<int> vehicleClasses = { 2, 7, 3 }; // car, truck, bus

bool isVehicle(int classId) {
    return std::find(vehicleClasses.begin(), vehicleClasses.end(), classId) != vehicleClasses.end();
}

VehicleCounter::VehicleCounter(const std::string& inputFile, const std::string& outputFile)
    : inputFile(inputFile), outputFile(outputFile) {
}

void VehicleCounter::run() {
    processVideo();
}

void VehicleCounter::processVideo() {
    cv::VideoCapture cap(inputFile);
    if (!cap.isOpened()) {
        throw std::runtime_error("Error opening video file: " + inputFile);
    }

    Tracker tracker;

    // Performans parametreleri
    double confidenceThreshold = 0.3;
    double nmsThreshold = 0.3;

    // YOLOv5 modelini y�kleme
    std::string modelPath = "C:/Users/kaank/source/repos/CarDetectionandCountingYOLO/third_party/Yolo5cpp - Kopya/yolov5/yolov5/yolov5s.onnx";
    std::cout << "Model Yolu: " << modelPath << std::endl;
    cv::dnn::Net net = cv::dnn::readNet(modelPath);
    if (net.empty()) {
        std::cerr << "Hata: Model y�klenemedi!" << std::endl;
        throw std::runtime_error("Model y�klenemedi!");
    }
    else {
        std::cout << "Model ba�ar�yla y�klendi: " << modelPath << std::endl;
    }
  
    // S�n�f isimlerini y�kleme
    std::vector<std::string> classNames;
    std::ifstream classNamesFile("C:/Users/kaank/source/repos/CarDetectionandCountingYOLO/third_party/Yolo5cpp - Kopya/yolov5/yolov5/classes.txt");
    std::string className;
    while (std::getline(classNamesFile, className)) {
        classNames.push_back(className);
    }
    if (classNames.empty()) {
        std::cerr << "Hata: S�n�f isimleri y�klenemedi!" << std::endl;
        throw std::runtime_error("S�n�f isimleri y�klenemedi!");
    }
    else {
        std::cout << "S�n�f isimleri ba�ar�yla y�klendi. Toplam: " << classNames.size() << std::endl;
        for (const auto& name : classNames) {
            std::cout << name << std::endl;
        }
    }
  
    
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);


    int totalVehicles = 0;
    double totalFrameTime = 0.0;
    int frameCount = 0;

    cv::Mat frame;
    while (cap.read(frame)) {
        auto start = std::chrono::high_resolution_clock::now();

        // YOLOv5 ile ara� tespiti
        cv::Mat blob = cv::dnn::blobFromImage(frame, 1 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);
        
        std::cout << "Blob Boyutu: "
            << blob.size[0] << "x"
            << blob.size[1] << "x"
            << blob.size[2] << "x"
            << blob.size[3] << std::endl;
        
        net.setInput(blob);
        std::vector<cv::Mat> detections;
        net.forward(detections, net.getUnconnectedOutLayersNames());
        std::cout << "Tespit katman say�s�: " << detections.size() << std::endl;


        // ��kt�y� i�le ve ara� s�n�r kutular�n� al
        std::vector<cv::Rect> validDetections;
        std::vector<int> classIds;
        std::vector<float> confidences;

        for (size_t i = 0; i < detections.size(); ++i) {
            float* data = (float*)detections[i].data;
            for (int j = 0; j < detections[i].rows; ++j, data += detections[i].cols) {
                float confidence = data[4];
                if (confidence > confidenceThreshold) {
                    int classId = -1;
                    float maxClassScore = 0.0f;

                    // S�n�f ve en y�ksek skoru al
                    for (int k = 5; k < detections[i].cols; ++k) {
                        if (data[k] > maxClassScore) {
                            maxClassScore = data[k];
                            classId = k - 5;
                        }
                    }

                    if (isVehicle(classId)) {
                        // Bounding box hesaplamas�
                        int centerX = (int)(data[0] * frame.cols);
                        int centerY = (int)(data[1] * frame.rows);
                        int width = (int)(data[2] * frame.cols);
                        int height = (int)(data[3] * frame.rows);
                        int left = centerX - width / 2;
                        int top = centerY - height / 2;

                        validDetections.push_back(cv::Rect(left, top, width, height));
                        classIds.push_back(classId);
                        confidences.push_back(confidence);
                    }
                }
            }
        }
        std::cout << "Tespit edilen ara� say�s�: " << validDetections.size() << std::endl;

        // NMS (Non-Maximum Suppression) uygulama
        std::vector<int> indices;
        cv::dnn::NMSBoxes(validDetections, confidences, confidenceThreshold, nmsThreshold, indices);

        // Ara�lar� takip et
        std::vector<cv::Rect> filteredDetections;
        for (int idx : indices) {
            filteredDetections.push_back(validDetections[idx]);
        }
        tracker.update(filteredDetections);

        // G�r�nt�y� i�aretle ve ara�lar� say
        for (auto& vehicle : tracker.getVehicles()) {
            if (!vehicle.counted) {
                totalVehicles++;
                vehicle.counted = true;
            }

            // Ara�lar�n s�n�r kutular�n� �iz ve etiketle
            cv::rectangle(frame, vehicle.boundingBox, cv::Scalar(0, 255, 0), 2);
            std::string label = "Vehicle " + std::to_string(vehicle.id);

            cv::putText(frame, label,
                cv::Point(vehicle.boundingBox.x, vehicle.boundingBox.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        auto end = std::chrono::high_resolution_clock::now();
        totalFrameTime += std::chrono::duration<double, std::milli>(end - start).count();
        frameCount++;

        // G�r�nt�y� g�ster
        cv::imshow("Vehicle Detection", frame);
        if (cv::waitKey(1) == 27) { // ESC ile ��k��
            break;
        }
    }

    // Sonu�lar� dosyaya yaz
    writeResults(totalVehicles, totalFrameTime / frameCount);
}

void VehicleCounter::writeResults(int totalVehicles, double avgFrameTime) {
    std::ofstream outFile(outputFile);
    if (!outFile.is_open()) {
        throw std::runtime_error("Error opening output file: " + outputFile);
    }

    outFile << "Total Vehicles: " << totalVehicles << "\n";
    outFile << "Average Frame Time (ms): " << avgFrameTime << "\n";
    outFile.close();
}