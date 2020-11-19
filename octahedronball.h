#ifndef OCTAHEDRONBALL_H
#define OCTAHEDRONBALL_H

#include "gsl_math.h"
#include "visualobject.h"
#include "shader.h"

// Forward declarations
class OctahedronBall : public VisualObject
{
private:
    friend class RenderWindow;

    int m_rekursjoner;
    int m_indeks;               // brukes i rekursjon, til å bygge m_vertices
    void lagTriangel(const gsl::vec3& v1, const gsl::vec3& v2, const gsl::vec3& v3);
    void subDivide(const gsl::vec3& a, const gsl::vec3& b, const gsl::vec3& c, int n);
    void oktaederUnitBall();
public:
    OctahedronBall(int n=0);
    OctahedronBall(const OctahedronBall& b) = default;

    void init() override;
    void draw() override;
    void move(float deltaTime) override;
    void update(float time) override;

    gsl::vec3 getAcceleraton();



    ~OctahedronBall() override;
};

#endif // OCTAHEDRONBALL_H
