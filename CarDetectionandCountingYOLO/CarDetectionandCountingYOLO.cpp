// CarDetectionandCountingYOLO.cpp : Defines the entry point for the application.
//

#include "CarDetectionandCountingYOLO.h"

#include <iostream>
#include <fstream>
#include "VehicleCounter.h"

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " --input <input_video> --output <output_file>\n";
        return 1;
    }

    std::string inputFile, outputFile;

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--input" && i + 1 < argc) {
            inputFile = argv[++i];
        }
        else if (std::string(argv[i]) == "--output" && i + 1 < argc) {
            outputFile = argv[++i];
        }
    }

    if (inputFile.empty() || outputFile.empty()) {
        std::cerr << "Input or output file not specified.\n";
        return 1;
    }

    try {
        VehicleCounter counter(inputFile, outputFile);
        counter.run();
    }
    catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << '\n';
        return 1;
    }

    return 0;
}
