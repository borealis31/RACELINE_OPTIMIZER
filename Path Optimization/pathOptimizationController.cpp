
#include "core/include/RaceTrack.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

int main() {

    vector<int> possibleTracks = {1, 2, 3};
    cout << "Select a Track:" << endl
         << "---------------" << endl;
    cout << "1. Oblong Test" << endl
         << "2. Kidney Bean Test" << endl
         << "3. Montreal" << endl;

    int input;

    do {
        cin >> input;
    } while (std::find(possibleTracks.begin(), possibleTracks.end(), static_cast<int>(input)) == possibleTracks.end());

    ifstream fileIn;

    switch (input) {
    case 1:
        fileIn.open("raceTrackCSVs/oblongBounds.csv");
        cout << "Oblong Testing Track Selected" << endl;
        break;
    case 2:
        fileIn.open("raceTrackCSVs/kidneyBeanBounds.csv");
        cout << "Kidney Bean Testing Track Selected" << endl;
        break;
    case 3:
        fileIn.open("raceTrackCSVs/montrealBounds.csv");
        cout << "Montreal Track Selected" << endl;
        break;
    default:
        cout << "ERROR: No Track File Associated With This TrackID" << endl;
        break;
    }

    if (!fileIn.is_open()) {
        cout << "ERROR: Track File Not Found" << endl;
        system("PAUSE");
        return EXIT_FAILURE;
    }

    std::string csvType;
    fileIn >> csvType;

    if (csvType == "IO") {
        cout << "IO-type CSV detected... Initializing Race Track" << endl;
        RaceTrackIO currentTrack(fileIn);
        currentTrack.optimizeRaceLineCurvature(500, 1e-12);

        ofstream fileOut;
        fileOut.open("oblongOut.csv");
        for (const auto &trackNode : currentTrack.getRaceLine()) {
            fileOut << trackNode.x << "," << trackNode.y << "\n";
        }
        fileOut.close();
    }

    fileIn.close();
    if (fileIn.is_open()) {
        cout << "WARNING: File was not Properly Closed" << endl;
    }

    cout << "Program Complete" << endl;
    system("PAUSE");
    return EXIT_SUCCESS;
}