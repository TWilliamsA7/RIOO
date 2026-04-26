#ifndef GAZE_H
#define GAZE_H

#include <Arduino.h>

struct GazePoint {
    float x;
    float y;
    unsigned long timestamp;
};

class GazeTracker {
public:
    static const int WINDOW_SIZE = 20;
    static constexpr float STABILITY_RADIUS = 15.0f;
    static const int OUTLIER_TOLERANCE = 3;

    GazeTracker();

    bool update(float x, float y);  // returns true if stable

private:
    GazePoint buffer[WINDOW_SIZE];
    int front;
    int count;

    void addPoint(float x, float y);
    bool isStable();
};

#endif
