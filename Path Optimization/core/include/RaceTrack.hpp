#pragma once

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

class RaceTrack {
public:
    struct Coordinate {
        double x = 0, y = 0;

        Coordinate(double xIn = 0, double yIn = 0) : x(xIn), y(yIn){};

        bool operator==(const Coordinate &coord) const {
            return x == coord.x && y == coord.y;
        };

        Coordinate operator+(const Coordinate &coord) const {
            Coordinate out(x + coord.x, y + coord.y);
            return out;
        };

        Coordinate operator-(const Coordinate &coord) const {
            Coordinate out(x - coord.x, y - coord.y);
            return out;
        };

        Coordinate operator/(const double &quot) const {
            Coordinate out(x / quot, y / quot);
            return out;
        };

        Coordinate operator*(const double &mult) const {
            Coordinate out(x * mult, y * mult);
            return out;
        };

        Coordinate operator^(const double &powr) const {
            Coordinate out(pow(x, powr), pow(y, powr));
            return out;
        };

        static double Coordinate::sum(const Coordinate &coord) {
            return coord.x + coord.y;
        };

        static double Coordinate::distance(const Coordinate &coordTO, const Coordinate &coordFROM) {
            return sqrt(Coordinate::sum((coordTO - coordFROM) ^ 2));
        };

        static double Coordinate::length(const Coordinate &coordVector) {
            return sqrt(Coordinate::sum(coordVector ^ 2));
        };

        static Coordinate Coordinate::normalize(const Coordinate &coordVector) {
            return coordVector / Coordinate::length(coordVector);
        };
    };

    struct LaneBoundary {
        Coordinate interiorPoint = {0, 0}, exteriorPoint = {0, 0};
        LaneBoundary(Coordinate interiorIn = {0, 0}, Coordinate exteriorIn = {0, 0}) : interiorPoint(interiorIn), exteriorPoint(exteriorIn){};
    };

    struct ChordVector {
        Coordinate chordDirection = {0, 0};
        double chordLength = 0;
        ChordVector(const Coordinate &directionVector = {0, 0}, const double &length = 0.0) : chordDirection(directionVector), chordLength(length){};
        Coordinate ChordVector::alongBy(const double &mult) const {
            return chordDirection * chordLength * mult;
        };
    };

    RaceTrack(){};

    void optimizeRaceLineCurvature(const int &maximumIterations, const double &minimumDK);

    // Debug functions
    std::vector<Coordinate> getRaceLine() const;
    std::vector<Coordinate> getCenterLine() const;

private:
    double curvatureCalculation(const std::vector<const Coordinate *> ambientPoints, const Coordinate *interestPoint) const;
    double mBufferSize = 0.1;

    std::vector<const Coordinate *> getAmbientPoints(const size_t &nIdx) const;
    const Coordinate *getNode(const size_t &nIdx) const;
    size_t getNumNodes() const;
    const ChordVector *getChordVector(const size_t &nIdx) const;
    const LaneBoundary *getLaneBounds(const size_t &nIdx) const;

protected:
    void setNode(const size_t &nIdx, const Coordinate &nodeValue);

    std::vector<Coordinate> mCenterLine = {};
    std::vector<ChordVector> mChordVectors = {};
    std::vector<LaneBoundary> mLaneBounds = {};
    std::vector<Coordinate> mRaceLine = {};
};

class RaceTrackIO : public RaceTrack {
public:
    RaceTrackIO(std::ifstream &fileIn);
};

inline void RaceTrack::setNode(const size_t &nIdx, const RaceTrack::Coordinate &nodeValue) {
    mRaceLine.at(nIdx) = nodeValue;
}

inline const RaceTrack::LaneBoundary* RaceTrack::getLaneBounds(const size_t &nIdx) const {
    return &mLaneBounds.at(nIdx);
}

inline const RaceTrack::Coordinate *RaceTrack::getNode(const size_t &nIdx) const {
    return &mRaceLine.at(nIdx);
}

inline size_t RaceTrack::getNumNodes() const {
    return mCenterLine.size();
}

inline const RaceTrack::ChordVector* RaceTrack::getChordVector(const size_t &nIdx) const {
    return &mChordVectors.at(nIdx);
}

inline std::vector<RaceTrack::Coordinate> RaceTrack::getRaceLine() const {
    return mRaceLine;
}

inline std::vector<RaceTrack::Coordinate> RaceTrack::getCenterLine() const {
    return mCenterLine;
}
