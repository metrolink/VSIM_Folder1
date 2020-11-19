#ifndef ROLLINGBALL_H
#define ROLLINGBALL_H

#include "octahedronball.h"
#include "visualobject.h"



class RollingBall : public OctahedronBall
{
private:

public:
    using OctahedronBall::OctahedronBall;
    //RollingBall(int re);
    //RollingBall(const RollingBall& b) = default;

    //OctahedronBall* RoundBall;

    void init() override;
    void draw() override;
    void move(float deltaTime) override;
    void update(float time) override;

    gsl::vec3 getAcceleraton();
};

#endif // ROLLINGBALL_H
