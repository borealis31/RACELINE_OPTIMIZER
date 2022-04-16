
#include "include/RaceTrack.hpp"
#include <algorithm>

void RaceTrack::optimizeRaceLineCurvature(const int &maximumIterations, const double &minimumDK) {

    int iterations = 0;
    size_t numberOfNodes = getNumNodes();
    double dK = 1e6;
    double kPrev = 1e6;

    while (dK > minimumDK && iterations < maximumIterations) {
        for (size_t nodeIdx = 0; nodeIdx < numberOfNodes; ++nodeIdx) {
            LaneBoundary localBounds = getLaneBounds(nodeIdx);
            ChordVector localChord = getChordVector(nodeIdx);
            std::vector<Coordinate> relevantNodes = getAmbientPoints(nodeIdx);

            std::vector<double> bisectionCoefficients = computeBisectionCoeffs(mBufferSize, 1 - mBufferSize);

            Coordinate interestPoint = localBounds.interiorPoint;

            double localKShift = 1e3, localKPrev = 1e3;
            int localIterations = 0;
            std::vector<double> kVals;
            double coeff = mBufferSize;
            while (coeff <= 1 - mBufferSize && localIterations <= 100) {
                kVals.emplace_back(curvatureCalculation(relevantNodes, interestPoint + localChord.alongBy(coeff)));
                coeff += 0.01;
            }
            size_t numIncrements = std::distance(kVals.begin(), std::min_element(kVals.begin(), kVals.end()));
            setNode(nodeIdx, interestPoint + localChord.alongBy(mBufferSize + 0.01 * numIncrements));
       }

        double kCur = 0;
        for (size_t nIdx = 0; nIdx < numberOfNodes; ++nIdx) {
            kCur += curvatureCalculation(getAmbientPoints(nIdx), getNode(nIdx)) / 2;
        }

        dK = kPrev - kCur;
        kPrev = kCur;
        ++iterations;
    }
}

RaceTrackIO::RaceTrackIO(std::ifstream &fileIn) {
    std::vector<Coordinate> centerLineOut;
    std::vector<Widths> centerWidthsOut;
    std::vector<ChordVector> chordVectorsOut;
    std::vector<LaneBoundary> laneBoundsOut;

    LaneBoundary laneBound;

    std::vector<double> coordinates(4, -999);
    std::string indivCoordinate;
    std::string coordinateLine;

    while (getline(fileIn, coordinateLine)) {
        if (coordinateLine.empty())
            continue;
        coordinates.clear();
        std::stringstream str(coordinateLine);
        while (getline(str, indivCoordinate, ',')) {
            coordinates.push_back(std::stod(indivCoordinate));
        }

        laneBound.interiorPoint = Coordinate(coordinates[0], coordinates[1]);
        laneBound.exteriorPoint = Coordinate(coordinates[2], coordinates[3]);
        laneBoundsOut.emplace_back(laneBound);

        Coordinate centerPoint;
        centerPoint = (laneBound.interiorPoint + laneBound.exteriorPoint) / 2;
        centerLineOut.push_back(centerPoint);

        centerWidthsOut.push_back(Widths(Coordinate::distance(laneBound.exteriorPoint, centerPoint),
                                         Coordinate::distance(centerPoint, laneBound.interiorPoint)));

        Coordinate chordDirection = Coordinate::normalize(laneBound.exteriorPoint - laneBound.interiorPoint);
        double chordLength = Coordinate::distance(laneBound.exteriorPoint, laneBound.interiorPoint);
        chordVectorsOut.push_back(ChordVector(chordDirection, chordLength));
    }

    if (centerLineOut.front() == centerLineOut.back()) {
        centerLineOut.pop_back();
        centerWidthsOut.pop_back();
        chordVectorsOut.pop_back();
    }

    mCenterLine = centerLineOut;
    mCenterWidths = centerWidthsOut;
    mChordVectors = chordVectorsOut;
    mLaneBounds = laneBoundsOut;
    mRaceLine = centerLineOut;
}

std::vector<double> RaceTrack::computeBisectionCoeffs(const double &coeffOne, const double &coeffTwo) const {
    double offset = (coeffTwo - coeffOne) / 3;
    return {coeffOne, coeffOne + offset, coeffTwo - offset, coeffTwo};
}

double RaceTrack::curvatureCalculation(const std::vector<Coordinate> &ambientPoints, const Coordinate &interestPoint) const {
    Coordinate dRdsM2 = Coordinate::normalize(ambientPoints[1] - ambientPoints[0]);
    Coordinate dRdsM1 = Coordinate::normalize(interestPoint - ambientPoints[1]);
    Coordinate dRdsP1 = Coordinate::normalize(ambientPoints[2] - interestPoint);
    Coordinate dRdsP2 = Coordinate::normalize(ambientPoints[3] - ambientPoints[2]);

    Coordinate dTdsM1 = dRdsM1 - dRdsM2;
    Coordinate dTdsC = dRdsP1 - dRdsM1;
    Coordinate dTdsP1 = dRdsP2 - dRdsP1;

    return 0.5 * Coordinate::length(dTdsM1) + Coordinate::length(dTdsC) + 0.5 * Coordinate::length(dTdsP1);
}

std::vector<RaceTrack::Coordinate> RaceTrack::getAmbientPoints(const size_t &nIdx) const {
    std::vector<RaceTrack::Coordinate> out(4);
    size_t numNodes = getNumNodes();
    if (nIdx == 0) {
        out[0] = getNode(numNodes - 2);
        out[1] = getNode(numNodes - 1);
        out[2] = getNode(nIdx + 1);
        out[3] = getNode(nIdx + 2);
    } else if (nIdx == 1) {
        out[0] = getNode(numNodes - 1);
        out[1] = getNode(nIdx - 1);
        out[2] = getNode(nIdx + 1);
        out[3] = getNode(nIdx + 2);
    } else if (nIdx == numNodes - 2) {
        out[0] = getNode(nIdx - 2);
        out[1] = getNode(nIdx - 1);
        out[2] = getNode(nIdx + 1);
        out[3] = getNode(0);
    } else if (nIdx == numNodes - 1) {
        out[0] = getNode(nIdx - 2);
        out[1] = getNode(nIdx - 1);
        out[2] = getNode(0);
        out[3] = getNode(1);
    } else {
        out[0] = getNode(nIdx - 2);
        out[1] = getNode(nIdx - 1);
        out[2] = getNode(nIdx + 1);
        out[3] = getNode(nIdx + 2);
    }
    return out;
}