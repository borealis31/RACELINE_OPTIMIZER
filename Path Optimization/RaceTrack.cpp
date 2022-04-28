
#include "include/RaceTrack.hpp"
#include "MatlabDataArray.hpp"

void RaceTrack::optimizeRaceLineCurvature(const int &maximumIterations, const double &minimumDK) {

    int iterations = 0;
    size_t numberOfNodes = getNumNodes();
    double dK = 1e6;
    double kPrev = 1e6;

    while (dK > minimumDK && iterations < maximumIterations) {
        for (size_t nodeIdx = 0; nodeIdx < numberOfNodes; ++nodeIdx) {
            const LaneBoundary *localBounds = getLaneBounds(nodeIdx);
            const ChordVector *localChord = getChordVector(nodeIdx);
            std::vector<const Coordinate *> relevantNodes = getAmbientPoints(nodeIdx);

            Coordinate interestPoint = localBounds->interiorPoint;

            double localKShift = 1e3, localKPrev = 1e3;
            int localIterations = 0;
            std::vector<double> kVals;
            double coeff = mBufferSize;
            while (coeff <= 1 - mBufferSize && localIterations <= 100) {
                Coordinate shiftedInterest = interestPoint + localChord->alongBy(coeff);
                kVals.emplace_back(curvatureCalculation(relevantNodes, &shiftedInterest));
                coeff += 0.01;
            }
            size_t numIncrements = std::distance(kVals.begin(), std::min_element(kVals.begin(), kVals.end()));
            setNode(nodeIdx, interestPoint + localChord->alongBy(mBufferSize + 0.01 * numIncrements));
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

double RaceTrack::curvatureCalculation(const std::vector<const Coordinate *> ambientPoints, const Coordinate *interestPoint) const {
    Coordinate dRdsM2 = Coordinate::normalize(*ambientPoints[1] - *ambientPoints[0]);
    Coordinate dRdsM1 = Coordinate::normalize(*interestPoint - *ambientPoints[1]);
    Coordinate dRdsP1 = Coordinate::normalize(*ambientPoints[2] - *interestPoint);
    Coordinate dRdsP2 = Coordinate::normalize(*ambientPoints[3] - *ambientPoints[2]);

    Coordinate dTdsM1 = dRdsM1 - dRdsM2;
    Coordinate dTdsC = dRdsP1 - dRdsM1;
    Coordinate dTdsP1 = dRdsP2 - dRdsP1;

    return 0.5 * Coordinate::length(dTdsM1) + Coordinate::length(dTdsC) + 0.5 * Coordinate::length(dTdsP1);
}

std::vector<const RaceTrack::Coordinate *> RaceTrack::getAmbientPoints(const size_t &nIdx) const {
    std::vector<const RaceTrack::Coordinate *> out(4);
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

void RaceTrack::plotRaceLine() const {
    size_t numNodes = getNumNodes();
    std::vector<double> xCoordRL(numNodes + 1), yCoordRL(numNodes + 1),
        xCoordIB(numNodes + 1), yCoordIB(numNodes + 1),
        xCoordOB(numNodes + 1), yCoordOB(numNodes + 1),
        xCoordCL(numNodes + 1), yCoordCL(numNodes + 1);

    for (size_t nIdx = 0; nIdx <= numNodes; ++nIdx) {
        const Coordinate *curRaceLineNode = getNode(nIdx % numNodes);
        const Coordinate *curCenterNode = getCenterNode(nIdx % numNodes);
        const LaneBoundary *curBound = getLaneBounds(nIdx % numNodes);
        xCoordRL.at(nIdx) = curRaceLineNode->x;
        yCoordRL.at(nIdx) = curRaceLineNode->y;
        xCoordIB.at(nIdx) = curBound->interiorPoint.x;
        yCoordIB.at(nIdx) = curBound->interiorPoint.y;
        xCoordOB.at(nIdx) = curBound->exteriorPoint.x;
        yCoordOB.at(nIdx) = curBound->exteriorPoint.y;
        xCoordCL.at(nIdx) = curCenterNode->x;
        yCoordCL.at(nIdx) = curCenterNode->y;
    }

    matlab::data::ArrayFactory plotFactory;
    std::vector<matlab::data::Array> plotArgs({plotFactory.createArray({xCoordRL.size(), 1}, xCoordRL.begin(), xCoordRL.end()),
                                               plotFactory.createArray({yCoordRL.size(), 1}, yCoordRL.begin(), yCoordRL.end()),
                                               plotFactory.createCharArray(std::string("m-")),
                                               plotFactory.createArray({xCoordIB.size(), 1}, xCoordIB.begin(), xCoordIB.end()),
                                               plotFactory.createArray({yCoordIB.size(), 1}, yCoordIB.begin(), yCoordIB.end()),
                                               plotFactory.createCharArray(std::string("k-")),
                                               plotFactory.createArray({xCoordOB.size(), 1}, xCoordOB.begin(), xCoordOB.end()),
                                               plotFactory.createArray({yCoordOB.size(), 1}, yCoordOB.begin(), yCoordOB.end()),
                                               plotFactory.createCharArray(std::string("k-")),
                                               plotFactory.createArray({xCoordCL.size(), 1}, xCoordCL.begin(), xCoordCL.end()),
                                               plotFactory.createArray({yCoordCL.size(), 1}, yCoordCL.begin(), yCoordCL.end()),
                                               plotFactory.createCharArray(std::string("r:"))});

    mMATLABEngine->feval(u"plot", plotArgs);
    mMATLABEngine->eval(u"axis equal");
    mMATLABEngine->eval(u"axis padded");

    system("PAUSE");
}

RaceTrackIO::RaceTrackIO(std::ifstream &fileIn) {
    std::cout << "Initilializing Visualization Engine" << std::endl;
    mMATLABEngine = matlab::engine::startMATLAB();
    std::cout << "Visualization Engine Initialized" << std::endl;

    std::vector<Coordinate> centerLineOut;
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

        Coordinate chordDirection = Coordinate::normalize(laneBound.exteriorPoint - laneBound.interiorPoint);
        double chordLength = Coordinate::distance(laneBound.exteriorPoint, laneBound.interiorPoint);
        chordVectorsOut.push_back(ChordVector(chordDirection, chordLength));
    }

    if (centerLineOut.front() == centerLineOut.back()) {
        centerLineOut.pop_back();
        chordVectorsOut.pop_back();
        laneBoundsOut.pop_back();
    }

    std::move(centerLineOut.begin(), centerLineOut.end(), std::back_inserter(mCenterLine));
    std::move(chordVectorsOut.begin(), chordVectorsOut.end(), std::back_inserter(mChordVectors));
    std::move(laneBoundsOut.begin(), laneBoundsOut.end(), std::back_inserter(mLaneBounds));
    std::move(centerLineOut.begin(), centerLineOut.end(), std::back_inserter(mRaceLine));
}
