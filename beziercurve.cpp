#include "beziercurve.h"

BezierCurve::BezierCurve(const std::vector<gsl::Vector3D> &controlPoints)
    : b{controlPoints}
{
    for (const auto &point : b) {
        mVertices.push_back({point.x, point.y, point.z, 1.f, 0.f, 0.f});
    }
    // mVertices.push_back({b.back().x, b.back().y, b.back().z, 0.f, 0.f, 0.f});
    for (float t{1.f}, step{0.05f}; t > 0.f; t -= step) {
        auto point = evaluateBezier(t);
        mVertices.push_back({point.x, point.y, point.z, 0.f, 1.f, 0.f});
    }
    mVertices.push_back({b.front().x, b.front().y, b.front().z, 0.f, 1.f, 0.f});

    mMatrix.setToIdentity();
}

gsl::Vector3D BezierCurve::evaluateBezier(float t)
{
    std::vector<gsl::Vector3D> c{b};
    for (unsigned int k=1; k<=getDegree(); k++)
    {
        for (int i=0; i < getDegree() - k + 1; i++)
            c[i] = c[i] * (1-t) + c[i+1] * t;
    }
    return c[0];
}

gsl::Vector3D BezierCurve::evaluateBezier(float t, unsigned int degree)
{
    std::vector<gsl::Vector3D> c{b};
    for (unsigned int k=1; k<=degree; k++)
    {
        for (int i=0; i<degree-k+1; i++)
            c[i] = c[i] * (1-t) + c[i+1] * t;
    }
    return c[0];
}

void BezierCurve::init()
{
    initializeOpenGLFunctions();

    //Vertex Array Object - VAO
    glGenVertexArrays( 1, &mVAO );
    glBindVertexArray( mVAO );

    //Vertex Buffer Object to hold vertices - VBO
    glGenBuffers( 1, &mVBO );
    glBindBuffer( GL_ARRAY_BUFFER, mVBO );

    glBufferData( GL_ARRAY_BUFFER, mVertices.size()*sizeof( Vertex ), mVertices.data(), GL_STATIC_DRAW );

    // 1rst attribute buffer : vertices
    glBindBuffer(GL_ARRAY_BUFFER, mVBO);
    glVertexAttribPointer(0, 3, GL_FLOAT,GL_FALSE, sizeof(Vertex), (GLvoid*)0);
    glEnableVertexAttribArray(0);

    // 2nd attribute buffer : colors
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,  sizeof( Vertex ),  (GLvoid*)(3 * sizeof(GLfloat)) );
    glEnableVertexAttribArray(1);

    //enable the matrixUniform
    // mMatrixUniform = glGetUniformLocation( matrixUniform, "matrix" );

    glBindVertexArray(0);
}

void BezierCurve::draw()
{
    glBindVertexArray( mVAO );
    glDrawArrays(GL_LINE_STRIP, 0, mVertices.size());
    // glDrawArrays(GL_FILL, 0, mVertices.size());
}

BezierCurve::~BezierCurve()
{

}

