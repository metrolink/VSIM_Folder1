#ifndef VISUALOBJECT_H
#define VISUALOBJECT_H

#include <QOpenGLFunctions_4_1_Core>
#include <vector>
#include "vertex.h"
#include "matrix4x4.h"
#include "shader.h"

class RenderWindow;

struct Triangle
{
    Triangle(std::array<unsigned int, 3> indices = {}, std::array<int, 3> neighbours = {})
        : index{indices[0], indices[1], indices[2]}, neighbour{neighbours[0], neighbours[1], neighbours[2]}
    {}

    unsigned int index[3];
    int neighbour[3];

    friend std::ostream& operator<< (std::ostream& out, const Triangle& tri);
    friend std::istream& operator>> (std::istream& in, Triangle& tri);
};


class VisualObject : public QOpenGLFunctions_4_1_Core {
public:
    VisualObject();
    virtual ~VisualObject();
    virtual void init();
    virtual void draw()=0;
    virtual void constructplane();

    virtual void move(float DeltaTime);
    virtual void update(float time);

    gsl::Matrix4x4 mMatrix;
    gsl::vec3 startPos{};
    gsl::vec3 mAcceleration{};
    gsl::vec3 velocity;
    float colliderRadius{1.f};

    std::string mName;

    RenderWindow *mRenderWindow; //Just to be able to call checkForGLerrors()

protected:
    std::vector<Vertex> mVertices;
    std::vector<GLuint> mIndices;

    GLuint mVAO{0};
    GLuint mVBO{0};
    GLuint mEAB{0}; //holds the indices (Element Array Buffer - EAB)

};
#endif // VISUALOBJECT_H

