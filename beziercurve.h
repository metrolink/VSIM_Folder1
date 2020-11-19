#ifndef BEZIERCURVE_H
#define BEZIERCURVE_H

#include "visualobject.h"
#include <array>

class BezierCurve;
using Bezier = BezierCurve;

class BezierCurve : public VisualObject
{
public:
    std::vector<gsl::Vector3D> b;

public:
    BezierCurve(const std::vector<gsl::Vector3D> &controlPoints);

    gsl::Vector3D evaluateBezier(float t);
    gsl::Vector3D evaluateBezier(float t, unsigned int degree);

    void init() override;
    void draw() override;

    unsigned int getDegree() const { return b.size() - 1; }

    ~BezierCurve() override;
};

#endif // BEZIERCURVE_H
