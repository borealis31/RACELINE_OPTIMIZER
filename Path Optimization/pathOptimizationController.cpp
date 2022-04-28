
#include "include/RaceTrack.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

int main() {

    std::vector<int> possibleTracks = {1, 2, 3};
    std::cout << "Select a Track:" << std::endl
              << "---------------" << std::endl;
    std::cout << "1. Oblong Test" << std::endl
              << "2. Kidney Bean Test" << std::endl
              << "3. Montreal" << std::endl;

    int input;

    do {
        std::cin >> input;
    } while (std::find(possibleTracks.begin(), possibleTracks.end(), 1) == possibleTracks.end());
    std::ifstream fileIn;

    switch (input) {
    case 1:
        fileIn.open("raceTrackCSVs/oblongBounds.csv");
        std::cout << "Oblong Testing Track Selected" << std::endl;
        break;
    case 2:
        fileIn.open("raceTrackCSVs/kidneyBeanBounds.csv");
        std::cout << "Kidney Bean Testing Track Selected" << std::endl;
        break;
    case 3:
        fileIn.open("raceTrackCSVs/montrealBounds.csv");
        std::cout << "Montreal Track Selected" << std::endl;
        break;
    default:
        std::cout << "ERROR: No Track File Associated With This TrackID" << std::endl;
        break;
    }

    if (!fileIn.is_open()) {
        std::cout << "ERROR: Track File Not Found" << std::endl;
        system("PAUSE");
        return EXIT_FAILURE;
    }

    std::string csvType;
    fileIn >> csvType;

    if (csvType == "IO") {
        std::cout << "IO-type CSV detected... Initializing Race Track" << std::endl;
        RaceTrackIO currentTrack(fileIn);
        currentTrack.optimizeRaceLineCurvature(500, 1e-12);
        currentTrack.plotRaceLine();
        
        // std::ofstream fileOut;
        // fileOut.open("oblongOut.csv");
        // for (const auto &trackNode : currentTrack.getRaceLine()) {
        //     fileOut << trackNode.x << "," << trackNode.y << "\n";
        // }
        // fileOut.close();
    }

    fileIn.close();
    if (fileIn.is_open()) {
        std::cout << "WARNING: File was not Properly Closed" << std::endl;
    }

    std::cout << "Program Complete" << std::endl;
    system("PAUSE");
    return EXIT_SUCCESS;
}