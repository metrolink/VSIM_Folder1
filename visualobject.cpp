#include "visualobject.h"
#include "shader.h"

VisualObject::VisualObject()
{
}

VisualObject::~VisualObject()
{
   glDeleteVertexArrays( 1, &mVAO );
   glDeleteBuffers( 1, &mVBO );
}

void VisualObject::init()
{
}

void VisualObject::constructplane()
{

}

void VisualObject::move(float deltaTime)
{

}

void VisualObject::update(float time)
{

}

std::ostream &operator<<(std::ostream &out, const Triangle &tri)
{
    return out << "("
        << tri.index[0] << ", "
        << tri.index[1] << ", "
        << tri.index[2] << ", "
        << tri.neighbour[0] << ", "
        << tri.neighbour[1] << ", "
        << tri.neighbour[2] << ")";
}

std::istream &operator>>(std::istream &in, Triangle &tri)
{
    char dummy;
    for (unsigned int i{0}; i < 6; ++i)
        if (i < 3)
            in >> dummy >> *(unsigned int*)(tri.index + i);
        else
            in >> dummy >> *(int*)(tri.index + i);

    return in >> dummy;
}
