#include "rollingball.h"

/*RollingBall::RollingBall(int re) : OctahedronBall(re)
{

}*/

void RollingBall::init()
{
   initializeOpenGLFunctions();

   //Vertex Array Object - VAO
   glGenVertexArrays( 1, &mVAO );
   glBindVertexArray( mVAO );

   //Vertex Buffer Object to hold vertices - VBO
   glGenBuffers( 1, &mVBO );
   glBindBuffer( GL_ARRAY_BUFFER, mVBO );

   glBufferData( GL_ARRAY_BUFFER, mVertices.size()*sizeof(Vertex), mVertices.data(), GL_STATIC_DRAW );

    glBindBuffer(GL_ARRAY_BUFFER, mVBO);
   glVertexAttribPointer(0, 3, GL_FLOAT,GL_FALSE,sizeof(Vertex), (GLvoid*)0);
   glEnableVertexAttribArray(0);

   glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,  sizeof(Vertex),  (GLvoid*)(3 * sizeof(GLfloat)) );
   glEnableVertexAttribArray(1);

   glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)(6 * sizeof(GLfloat)));
   glEnableVertexAttribArray(2);

   glBindVertexArray(0);

}

void RollingBall::draw()
{
   glBindVertexArray( mVAO );
   glDrawArrays(GL_TRIANGLES, 0, mVertices.size());//mVertices.size());
}

void RollingBall::move(float deltaTime)
{
    std::cout << "moving";
}

void RollingBall::update(float time)
{
    auto s = startPos + (mAcceleration * std::pow(time, 2)) / 2.f;
    mMatrix.setPosition(s.x, s.y, s.z);
    std::cout << "moving";
}

gsl::vec3 RollingBall::getAcceleraton()
{

}
