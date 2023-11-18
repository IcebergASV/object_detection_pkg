#ifndef DETECTED_OBJECT_H
#define DETECTED_OBJECT_H

#include <string>

class DetectedObject {
public:
    // Constructor
    DetectedObject(const std::string& class_name, float confidence, int x_min, int x_max, int y_min, int y_max);

    // Getters
    std::string getClassName() const;
    float getConfidence() const;
    int getXMin() const;
    int getXMax() const;
    int getYMin() const;
    int getYMax() const;

    // Setters
    void setClassName(const std::string& class_name);
    void setConfidence(float confidence);
    void setXMin(int x_min);
    void setXMax(int x_max);
    void setYMin(int y_min);
    void setYMax(int y_max);

private:
    std::string class_name;
    float confidence;
    int x_min;
    int x_max;
    int y_min;
    int y_max;
};

#endif // DETECTED_OBJECT_H
