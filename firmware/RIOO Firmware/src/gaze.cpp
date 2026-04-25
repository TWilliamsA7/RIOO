#include "gaze.h"
#include "Arduino.h"

bool isGazeBufferFull();
void addGazePoint(float x, float y);

bool isGazeStable(float newX, float newY) {
    addGazePoint(newX, newY);

    if (gazeBufferCount < WINDOW_SIZE) return false;

    // Calculate Centroid
    float avgX = 0, avgY = 0;
    for (const auto& p : gazeBuffer) {
        avgX += p.x;
        avgY += p.y;
    }
    avgX /= WINDOW_SIZE;
    avgY /= WINDOW_SIZE;

    // Check for outliers
    int gazeMisses = 0;

    for (const auto& p : gazeBuffer) {
        float distance = sqrt(pow(p.x - avgX, 2) + pow(p.y - avgY, 2));
        if (distance > STABILITY_RADIUS) {
            gazeMisses++;
        }
        if (gazeMisses > OUTLIER_TOLERANCE) {
            return false;
        }
    }

    return true;
}

bool isGazeBufferFull() {
    return (gazeBufferFront == gazeBufferRear + 1) || (gazeBufferFront == 0 && gazeBufferRear == WINDOW_SIZE - 1);
}

void addGazePoint(float x, float y) {

    GazePoint g;
    g.x = x; g.y = y;
    g.timestamp = millis();

    if (gazeBufferCount == WINDOW_SIZE) {
        gazeBufferFront = (gazeBufferFront + 1) % WINDOW_SIZE;
        gazeBufferCount--;
    }
    gazeBufferRear = (gazeBufferRear + 1) % WINDOW_SIZE;
    gazeBuffer[gazeBufferRear] = g;
    gazeBufferCount++;
}
