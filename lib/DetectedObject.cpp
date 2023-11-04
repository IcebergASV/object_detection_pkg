#include "DetectedObject.h"

// Constructor
DetectedObject::DetectedObject(const std::string& class_name, float confidence, int x_min, int x_max, int y_min, int y_max)
    : class_name(class_name), confidence(confidence), x_min(x_min), x_max(x_max), y_min(y_min), y_max(y_max) {
    // Initialize the DetectedObject members in the constructor
}

// Getters
std::string DetectedObject::getClassName() const {
    return class_name;
}

float DetectedObject::getConfidence() const {
    return confidence;
}

int DetectedObject::getXMin() const {
    return x_min;
}

int DetectedObject::getXMax() const {
    return x_max;
}

int DetectedObject::getYMin() const {
    return y_min;
}

int DetectedObject::getYMax() const {
    return y_max;
}

// Setters
void DetectedObject::setClassName(const std::string& class_name) {
    this->class_name = class_name;
}

void DetectedObject::setConfidence(float confidence) {
    this->confidence = confidence;
}

void DetectedObject::setXMin(int x_min) {
    this->x_min = x_min;
}

void DetectedObject::setXMax(int x_max) {
    this->x_max = x_max;
}

void DetectedObject::setYMin(int y_min) {
    this->y_min = y_min;
}

void DetectedObject::setYMax(int y_max) {
    this->y_max = y_max;
}
