#ifndef GAZE_H
#define GAZE_H

struct GazePoint {
    float x;
    float y;
    unsigned long timestamp;
};

// Sliding window size for finding steady gaze
const int WINDOW_SIZE = 20;

// Allowable tolerance for a point to classify as a steady gaze
const float STABILITY_RADIUS = 15.0;

// Allowable number of misses for tolerance in classifying steady gaze
const int OUTLIER_TOLERANCE = 3;

GazePoint gazeBuffer[WINDOW_SIZE];
int gazeBufferFront = 0, gazeBufferRear = -1;
int gazeBufferCount = 0;

bool isGazeStable(float newX, float newY);


#endif // GAZE_H