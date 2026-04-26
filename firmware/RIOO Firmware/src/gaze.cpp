#include "gaze.h"

// Precompute squared radius (avoid sqrt)
static constexpr float STABILITY_RADIUS_SQ =
    GazeTracker::STABILITY_RADIUS * GazeTracker::STABILITY_RADIUS;

GazeTracker::GazeTracker() {
    front = 0;
    count = 0;
}

void GazeTracker::addPoint(float x, float y) {
    int index = (front + count) % WINDOW_SIZE;

    buffer[index].x = x;
    buffer[index].y = y;
    buffer[index].timestamp = millis();

    if (count < WINDOW_SIZE) {
        count++;
    } else {
        // overwrite oldest
        front = (front + 1) % WINDOW_SIZE;
    }
}

bool GazeTracker::update(float x, float y) {
    addPoint(x, y);

    if (count < WINDOW_SIZE) return false;

    return isStable();
}

bool GazeTracker::isStable() {
    float avgX = 0.0f;
    float avgY = 0.0f;

    // Compute centroid
    for (int i = 0; i < count; i++) {
        int idx = (front + i) % WINDOW_SIZE;
        avgX += buffer[idx].x;
        avgY += buffer[idx].y;
    }

    avgX /= count;
    avgY /= count;

    int misses = 0;

    // Check deviation using squared distance
    for (int i = 0; i < count; i++) {
        int idx = (front + i) % WINDOW_SIZE;

        float dx = buffer[idx].x - avgX;
        float dy = buffer[idx].y - avgY;

        float distSq = dx * dx + dy * dy;

        if (distSq > STABILITY_RADIUS_SQ) {
            misses++;
            if (misses > OUTLIER_TOLERANCE) {
                return false;
            }
        }
    }

    return true;
}
