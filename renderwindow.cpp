#include "renderwindow.h"
#include <QTimer>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLDebugLogger>
#include <QKeyEvent>
#include <QStatusBar>
#include <QDebug>

#include "shader.h"
#include "mainwindow.h"
#include "matrix4x4.h"
#include "gsl_math.h"

#include "xyz.h"
#include "trianglesurface.h"
#include "octahedronball.h"
#include "beziercurve.h"
#include "rollingball.h"

RenderWindow::RenderWindow(const QSurfaceFormat &format, MainWindow *mainWindow)
    : mContext(nullptr), mInitialized(false), mMainWindow(mainWindow)
{
    //This is sent to QWindow:
    setSurfaceType(QWindow::OpenGLSurface);
    setFormat(format);
    //Make the OpenGL context
    mContext = new QOpenGLContext(this);
    //Give the context the wanted OpenGL format (v4.1 Core)
    mContext->setFormat(requestedFormat());
    if (!mContext->create()) {
        delete mContext;
        mContext = nullptr;
        qDebug() << "Context could not be made - quitting this application";
    }

    //Make the gameloop timer:
    mRenderTimer = new QTimer(this);
}

RenderWindow::~RenderWindow()
{
}

/// Sets up the general OpenGL stuff and the buffers needed to render a triangle
void RenderWindow::init()
{
    //Connect the gameloop timer to the render function:
    connect(mRenderTimer, SIGNAL(timeout()), this, SLOT(render()));

    //********************** General OpenGL stuff **********************

    //The OpenGL context has to be set.
    //The context belongs to the instanse of this class!
    if (!mContext->makeCurrent(this)) {
        qDebug() << "makeCurrent() failed";
        return;
    }

    //just to make sure we don't init several times
    //used in exposeEvent()
    if (!mInitialized)
        mInitialized = true;

    //must call this to use OpenGL functions
    initializeOpenGLFunctions();

    //Start the Qt OpenGL debugger
    //Really helpfull when doing OpenGL
    //Supported on most Windows machines
    //reverts to plain glGetError() on Mac and other unsupported PCs
    // - can be deleted
    startOpenGLDebugger();

    //general OpenGL stuff:
    glEnable(GL_DEPTH_TEST);    //enables depth sorting - must use GL_DEPTH_BUFFER_BIT in glClear
    //glEnable(GL_CULL_FACE);     //draws only front side of models - usually what you want -
    glClearColor(0.4f, 0.4f, 0.4f, 1.0f);    //color used in glClear GL_COLOR_BUFFER_BIT

    //Compile shaders:
    //NB: hardcoded path to files! You have to change this if you change directories for the project.
    //Qt makes a build-folder besides the project folder. That is why we go down one directory
    // (out of the build-folder) and then up into the project folder.
    mShaderProgram[0] = new Shader("../VSIM_Folder1/plainvertex.vert", "../VSIM_Folder1/plainfragment.frag");
    qDebug() << "Plain shader program id: " << mShaderProgram[0]->getProgram();
    mShaderProgram[1]= new Shader("../VSIM_Folder1/texturevertex.vert", "../VSIM_Folder1/texturefragmet.frag");
    qDebug() << "Texture shader program id: " << mShaderProgram[1]->getProgram();

    mShaderProgram[2] = new Shader("../VSIM_Folder1/NPCVertex.vert", "../VSIM_Folder1/NPCfragment.frag");
    qDebug() << "Plain shader program id: " << mShaderProgram[2]->getProgram();

    setupPlainShader(0);
    setupTextureShader(1);

    //**********************  Texture stuff: **********************
    mTexture[0] = new Texture();
    mTexture[1] = new Texture("../VSIM_Folder1/Assets/hund.bmp");

    //Set the textures loaded to a texture unit
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mTexture[0]->id());
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, mTexture[1]->id());

    //********************** Making the objects to be drawn **********************
    VisualObject *temp = new XYZ();
    temp->init();
    mVisualObjects.push_back(temp);

    //testing triangle surface class
//    temp = new TriangleSurface();
//    temp->init();
//    mVisualObjects.push_back(temp);

    //Oppgave 2:
    mSurface = new TriangleSurface("../VSIM_Folder1/SecondTriangle.txt");
    mSurface->init();

    mVisualObjects.push_back(mSurface);

    mBall = new RollingBall(6);
    mBall->init();
    mBall->mMatrix.scale(0.25);
    //mBall->mMatrix.rotateX(90);
    //Her blir ballen sluppet fra 4m over bakken
    mBall->mMatrix.setPosition(0.3,4.0,0.1);
    mBall->startPos = gsl::vec3{0.3,2.0,0.1};
    mBall->mAcceleration = gsl::vec3{0.0,-9.81,0};
    mVisualObjects.push_back(mBall);


    //********************** Set up camera **********************
    mCurrentCamera = new Camera();
    mCurrentCamera->setPosition(gsl::Vector3D(-1.f, -2.0f, 2.f));

    initTerrain();
}

///Called each frame - doing the rendering
void RenderWindow::render()
{

    //input
    handleInput();

    mCurrentCamera->update();

    const float deltaTime = mTimeStart.nsecsElapsed() / 1000000000.f;
    mTimeStart.restart(); //restart FPS clock
    mContext->makeCurrent(this); //must be called every frame (every time mContext->swapBuffers is called)

    //to clear the screen for each redraw
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //******** This should be done with a loop!
    {
        glUseProgram(mShaderProgram[0]->getProgram());
        glUniformMatrix4fv( vMatrixUniform0, 1, GL_TRUE, mCurrentCamera->mViewMatrix.constData());
        glUniformMatrix4fv( pMatrixUniform0, 1, GL_TRUE, mCurrentCamera->mProjectionMatrix.constData());
        glUniformMatrix4fv( mMatrixUniform0, 1, GL_TRUE, mVisualObjects[0]->mMatrix.constData());
        mVisualObjects[0]->draw();


        //Oppgave 2:
        glUseProgram(mShaderProgram[0]->getProgram());
        glUniformMatrix4fv( vMatrixUniform0, 1, GL_TRUE, mCurrentCamera->mViewMatrix.constData());
        glUniformMatrix4fv( pMatrixUniform0, 1, GL_TRUE, mCurrentCamera->mProjectionMatrix.constData());
        glUniformMatrix4fv( mMatrixUniform0, 1, GL_TRUE, mVisualObjects[1]->mMatrix.constData());
        mVisualObjects[1]->draw();

        glUseProgram(mShaderProgram[0]->getProgram());
        moveBall(deltaTime);
        glUniformMatrix4fv( vMatrixUniform0, 1, GL_TRUE, mCurrentCamera->mViewMatrix.constData());
        glUniformMatrix4fv( pMatrixUniform0, 1, GL_TRUE, mCurrentCamera->mProjectionMatrix.constData());
        glUniformMatrix4fv( mMatrixUniform0, 1, GL_TRUE, mVisualObjects[2]->mMatrix.constData());
        mVisualObjects[2]->draw();




    }

    //Calculate framerate before
    // checkForGLerrors() because that takes a long time
    // and before swapBuffers(), else it will show the vsync time
    calculateFramerate();

    //using our expanded OpenGL debugger to check if everything is OK.
    checkForGLerrors();

    //Qt require us to call this swapBuffers() -function.
    // swapInterval is 1 by default which means that swapBuffers() will (hopefully) block
    // and wait for vsync.
    mContext->swapBuffers(this);
}


void RenderWindow::setupPlainShader(int shaderIndex)
{
    mMatrixUniform0 = glGetUniformLocation( mShaderProgram[shaderIndex]->getProgram(), "mMatrix" );
    vMatrixUniform0 = glGetUniformLocation( mShaderProgram[shaderIndex]->getProgram(), "vMatrix" );
    pMatrixUniform0 = glGetUniformLocation( mShaderProgram[shaderIndex]->getProgram(), "pMatrix" );
}

void RenderWindow::setupTextureShader(int shaderIndex)
{
    mMatrixUniform1 = glGetUniformLocation( mShaderProgram[shaderIndex]->getProgram(), "mMatrix" );
    vMatrixUniform1 = glGetUniformLocation( mShaderProgram[shaderIndex]->getProgram(), "vMatrix" );
    pMatrixUniform1 = glGetUniformLocation( mShaderProgram[shaderIndex]->getProgram(), "pMatrix" );
    mTextureUniform = glGetUniformLocation(mShaderProgram[shaderIndex]->getProgram(), "textureSampler");
}

void RenderWindow::moveBall(float deltaTime)
{
    //std::cout << "bruh";
    auto& ball = *mVisualObjects[2];
    ball.velocity += ball.mAcceleration * deltaTime;
    //std::cout << ball.mAcceleration << endl;
    auto pos = ball.mMatrix.getPosition() + ball.velocity * deltaTime;

    auto hitResults = isColliding(&ball, 1.f);
    if (hitResults.first)
    {
        ball.velocity -= gsl::project(ball.velocity, hitResults.second);
        pos = ball.mMatrix.getPosition() + ball.velocity * deltaTime +
                (gsl::project(-ball.mAcceleration, hitResults.second) + ball.mAcceleration) * std::pow(deltaTime, 2) * 0.5f;
    }
    else
    {
        pos = ball.mMatrix.getPosition() + ball.velocity * deltaTime;
    }

    ball.mMatrix.setPosition(pos.x, pos.y, pos.z);
    // std::cout << "velocity: " << ball.velocity << std::endl;
}

std::pair<bool, gsl::vec3> RenderWindow::isColliding(VisualObject* ball, float ballRadius)
{
    auto* tri = getBallToPlaneTriangle(ball->mMatrix.getPosition());
    if (tri != nullptr)
    {
        gsl::vec3 normal = (mTerrainVertices.at(tri->index[1]).get_xyz() - mTerrainVertices.at(tri->index[0]).get_xyz())
                ^ (mTerrainVertices.at(tri->index[2]).get_xyz() - mTerrainVertices.at(tri->index[0]).get_xyz());
        normal.normalize();
        // std::cout << "Normal is: " << normal << std::endl;

        auto toBall = gsl::project(ball->mMatrix.getPosition() - mTerrainVertices.at(tri->index[0]).get_xyz(), normal);

        if (toBall.length() < ballRadius && 0 < toBall * normal)
        {
            // Calculate force
            return {true, normal};
        }
    }
    // ball->mAcceleration = gsl::vec3{0.f, -9.81, 0.f};
    return {false, {0, 0, 0}};
}

Triangle *RenderWindow::getBallToPlaneTriangle(gsl::vec3 ballPos)
{
    if (mTerrainTriangles.empty())
        return nullptr;

    unsigned int index = 0;
    auto* t = &mTerrainTriangles.at(index);
    std::array<gsl::vec3, 3> triangle;
    for (unsigned int i{0}; i < 3; ++i)
    {
        triangle.at(i) = mTerrainVertices.at(t->index[i]).get_xyz();
        triangle.at(i).y = 0.f;
    }

    gsl::vec3 bCoords = gsl::barCoord(gsl::vec3{ballPos.x, 0.f, ballPos.z}, triangle.at(0), triangle.at(1), triangle.at(2));
    unsigned int lastIndex = std::numeric_limits<unsigned int>::max();
    while (!(0 <= bCoords.x && 0 <= bCoords.y && 0 <= bCoords.z))
    {
        unsigned int lowestIndex{0};
        lowestIndex = (bCoords.y < bCoords.x) ? 1 : lowestIndex;
        lowestIndex = (bCoords.z < bCoords.x) ? 2 : lowestIndex;

        if (t->neighbour[lowestIndex] < 0)
            return nullptr;

        unsigned int nextIndex = t->neighbour[lowestIndex];
        if (lastIndex == nextIndex)
            return nullptr;

        lastIndex = index;
        index = nextIndex;

        t = &mTerrainTriangles.at(index);
        for (unsigned int i{0}; i < 3; ++i)
        {
            triangle.at(i) = mTerrainVertices.at(t->index[i]).get_xyz();
            triangle.at(i).y = 0.f;
        }

        bCoords = gsl::barCoord(gsl::vec3{ballPos.x, 0.f, ballPos.z}, triangle.at(0), triangle.at(1), triangle.at(2));
    }

    return t;
}

//This function is called from Qt when window is exposed (shown)
//and when it is resized
//exposeEvent is a overridden function from QWindow that we inherit from
void RenderWindow::exposeEvent(QExposeEvent *)
{
    if (!mInitialized)
        init();

    //This is just to support modern screens with "double" pixels
    const qreal retinaScale = devicePixelRatio();
    glViewport(0, 0, static_cast<GLint>(width() * retinaScale), static_cast<GLint>(height() * retinaScale));

    //If the window actually is exposed to the screen we start the main loop
    //isExposed() is a function in QWindow
    if (isExposed())
    {
        //This timer runs the actual MainLoop
        //16 means 16ms = 60 Frames pr second (should be 16.6666666 to be exact..)
        mRenderTimer->start(16);
        mTimeStart.start();
    }
    mAspectratio = static_cast<float>(width()) / height();
    //    qDebug() << mAspectratio;
    mCurrentCamera->mProjectionMatrix.perspective(45.f, mAspectratio, 1.f, 100.f);
    //    qDebug() << mCamera.mProjectionMatrix;
}

//Simple way to turn on/off wireframe mode
//Not totally accurate, but draws the objects with
//lines instead of filled polygons
void RenderWindow::toggleWireframe()
{
    mWireframe = !mWireframe;
    if (mWireframe)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);    //turn on wireframe mode
        glDisable(GL_CULL_FACE);
    }
    else
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);    //turn off wireframe mode
        glEnable(GL_CULL_FACE);
    }
}

//The way this is set up is that we start the clock before doing the draw call,
//and check the time right after it is finished (done in the render function)
//This will approximate what framerate we COULD have.
//The actual frame rate on your monitor is limited by the vsync and is probably 60Hz
void RenderWindow::calculateFramerate()
{
    long long nsecElapsed = mTimeStart.nsecsElapsed();
    static int frameCount{0};                       //counting actual frames for a quick "timer" for the statusbar

    if (mMainWindow)    //if no mainWindow, something is really wrong...
    {
        ++frameCount;
        if (frameCount > 30) //once pr 30 frames = update the message twice pr second (on a 60Hz monitor)
        {
            //showing some statistics in status bar
            mMainWindow->statusBar()->showMessage(" Time pr FrameDraw: " +
                                                  QString::number(nsecElapsed/1000000., 'g', 4) + " ms  |  " +
                                                  "FPS (approximated): " + QString::number(1E9 / nsecElapsed, 'g', 7));
            frameCount = 0;     //reset to show a new message in 60 frames
        }
    }
}


/// Uses QOpenGLDebugLogger if this is present
/// Reverts to glGetError() if not
void RenderWindow::checkForGLerrors()
{
    if(mOpenGLDebugLogger)
    {
        const QList<QOpenGLDebugMessage> messages = mOpenGLDebugLogger->loggedMessages();
        for (const QOpenGLDebugMessage &message : messages)
            qDebug() << message;
    }
    else
    {
        GLenum err = GL_NO_ERROR;
        while((err = glGetError()) != GL_NO_ERROR)
        {
            qDebug() << "glGetError returns " << err;
        }
    }
}

/// Tries to start the extended OpenGL debugger that comes with Qt
void RenderWindow::startOpenGLDebugger()
{
    QOpenGLContext * temp = this->context();
    if (temp)
    {
        QSurfaceFormat format = temp->format();
        if (! format.testOption(QSurfaceFormat::DebugContext))
            qDebug() << "This system can not use QOpenGLDebugLogger, so we revert to glGetError()";

        if(temp->hasExtension(QByteArrayLiteral("GL_KHR_debug")))
        {
            qDebug() << "System can log OpenGL errors!";
            mOpenGLDebugLogger = new QOpenGLDebugLogger(this);
            if (mOpenGLDebugLogger->initialize()) // initializes in the current context
                qDebug() << "Started OpenGL debug logger!";
        }

        if(mOpenGLDebugLogger)
            mOpenGLDebugLogger->disableMessages(QOpenGLDebugMessage::APISource, QOpenGLDebugMessage::OtherType, QOpenGLDebugMessage::NotificationSeverity);
    }
}

void RenderWindow::setCameraSpeed(float value)
{
    mCameraSpeed += value;

    //Keep within min and max values
    if(mCameraSpeed < 0.01f)
        mCameraSpeed = 0.01f;
    if (mCameraSpeed > 0.3f)
        mCameraSpeed = 0.3f;
}

bool RenderWindow::readTerrainData(std::string file)
{
    std::cout << "read ";
    std::ifstream ifs{file};
    if (ifs)
    {
        unsigned int n{0};
        ifs >> n;
        mTerrainVertices.reserve(n);
        for (unsigned int i{0}; i < n; ++i)
        {
            Vertex v;
            ifs >> v;
            mTerrainVertices.push_back(v);
        }

        ifs >> n;
        mTerrainTriangles.reserve(n);
        for (unsigned int i{0}; i < n; ++i)
        {
            Triangle t;
            ifs >> t;
            mTerrainTriangles.push_back(t);
        }

        return true;
    }
    else{
        std::cout << "no ifs";
        return false;}
}

void RenderWindow::initTerrain()
{
    readTerrainData("../VSIM_Folder1/SecondTriangle.txt");


    std::cout << "Triangle count: " << mTerrainTriangles.size() << std::endl;

    glGenVertexArrays(1, &mTerrainVAO);
    glBindVertexArray(mTerrainVAO);

    GLuint terrainVBO, terrainEBO;
    glGenBuffers(1, &terrainVBO);
    glBindBuffer(GL_ARRAY_BUFFER, terrainVBO);
    glBufferData(GL_ARRAY_BUFFER, mTerrainVertices.size() * sizeof(Vertex), mTerrainVertices.data(), GL_STATIC_DRAW);


    unsigned int *data = new unsigned int[mTerrainTriangles.size() * 3];
    for (unsigned int i{0}; i < mTerrainTriangles.size(); ++i)
        for (unsigned int j{0}; j < 3; ++j)
            data[i * 3 + j] = mTerrainTriangles.at(i).index[j];

    glGenBuffers(1, &terrainEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, terrainEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mTerrainTriangles.size() * 3 * sizeof(unsigned int), data, GL_STATIC_DRAW);

    delete[] data;

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)(3 * sizeof(GLfloat)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)(6 * sizeof(GLfloat)));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);

    // glPointSize(10.f);
}

void RenderWindow::handleInput()
{
    //Camera
    mCurrentCamera->setSpeed(0.f);  //cancel last frame movement
    if(mInput.RMB)
    {
        if(mInput.W)
            mCurrentCamera->setSpeed(-mCameraSpeed);
        if(mInput.S)
            mCurrentCamera->setSpeed(mCameraSpeed);
        if(mInput.D)
            mCurrentCamera->moveRight(mCameraSpeed);
        if(mInput.A)
            mCurrentCamera->moveRight(-mCameraSpeed);
        if(mInput.Q)
            mCurrentCamera->updateHeigth(mCameraSpeed);
        if(mInput.E)
            mCurrentCamera->updateHeigth(-mCameraSpeed);
    }

    //Oppgave 3+4
    if(mInput.UP){
        mPlayer->mMatrix.translateZ(0.3f);
        mBall->mMatrix.translateZ(0.6f);
    }
    if(mInput.DOWN)
    {
        mPlayer->mMatrix.translateZ(-0.3f);
        mBall->mMatrix.translateZ(-0.6f);
    }
    if(mInput.LEFT)
    {
        mPlayer->mMatrix.translateX(-0.3f);
        mBall->mMatrix.translateX(-0.6f);
    }
    if(mInput.RIGHT)
    {
        mPlayer->mMatrix.translateX(0.3f);
        mBall->mMatrix.translateX(0.6f);
    }
}

void RenderWindow::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Escape) //Shuts down whole program
    {
        mMainWindow->close();
    }

    //    You get the keyboard input like this
    if(event->key() == Qt::Key_W)
    {
        mInput.W = true;
    }
    if(event->key() == Qt::Key_S)
    {
        mInput.S = true;
    }
    if(event->key() == Qt::Key_D)
    {
        mInput.D = true;
    }
    if(event->key() == Qt::Key_A)
    {
        mInput.A = true;
    }
    if(event->key() == Qt::Key_Q)
    {
        mInput.Q = true;
    }
    if(event->key() == Qt::Key_E)
    {
        mInput.E = true;
    }
    if(event->key() == Qt::Key_Z)
    {
    }
    if(event->key() == Qt::Key_X)
    {
    }
    if(event->key() == Qt::Key_Up)
    {
        mInput.UP = true;
    }
    if(event->key() == Qt::Key_Down)
    {
        mInput.DOWN = true;
    }
    if(event->key() == Qt::Key_Left)
    {
        mInput.LEFT = true;
    }
    if(event->key() == Qt::Key_Right)
    {
        mInput.RIGHT = true;
    }
    if(event->key() == Qt::Key_U)
    {
    }
    if(event->key() == Qt::Key_O)
    {
    }
}

void RenderWindow::keyReleaseEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_W)
    {
        mInput.W = false;
    }
    if(event->key() == Qt::Key_S)
    {
        mInput.S = false;
    }
    if(event->key() == Qt::Key_D)
    {
        mInput.D = false;
    }
    if(event->key() == Qt::Key_A)
    {
        mInput.A = false;
    }
    if(event->key() == Qt::Key_Q)
    {
        mInput.Q = false;
    }
    if(event->key() == Qt::Key_E)
    {
        mInput.E = false;
    }
    if(event->key() == Qt::Key_Z)
    {
    }
    if(event->key() == Qt::Key_X)
    {
    }
    if(event->key() == Qt::Key_Up)
    {
        mInput.UP = false;
    }
    if(event->key() == Qt::Key_Down)
    {
        mInput.DOWN = false;
    }
    if(event->key() == Qt::Key_Left)
    {
        mInput.LEFT = false;
    }
    if(event->key() == Qt::Key_Right)
    {
        mInput.RIGHT = false;
    }
    if(event->key() == Qt::Key_U)
    {
    }
    if(event->key() == Qt::Key_O)
    {
    }
}

void RenderWindow::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
        mInput.RMB = true;
    if (event->button() == Qt::LeftButton)
        mInput.LMB = true;
    if (event->button() == Qt::MiddleButton)
        mInput.MMB = true;
}

void RenderWindow::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
        mInput.RMB = false;
    if (event->button() == Qt::LeftButton)
        mInput.LMB = false;
    if (event->button() == Qt::MiddleButton)
        mInput.MMB = false;
}

void RenderWindow::wheelEvent(QWheelEvent *event)
{
    QPoint numDegrees = event->angleDelta() / 8;

    //if RMB, change the speed of the camera
    if (mInput.RMB)
    {
        if (numDegrees.y() < 1)
            setCameraSpeed(0.001f);
        if (numDegrees.y() > 1)
            setCameraSpeed(-0.001f);
    }
    event->accept();
}

void RenderWindow::mouseMoveEvent(QMouseEvent *event)
{
    if (mInput.RMB)
    {
        //Using mMouseXYlast as deltaXY so we don't need extra variables
        mMouseXlast = event->pos().x() - mMouseXlast;
        mMouseYlast = event->pos().y() - mMouseYlast;

        if (mMouseXlast != 0)
            mCurrentCamera->yaw(mCameraRotateSpeed * mMouseXlast);
        if (mMouseYlast != 0)
            mCurrentCamera->pitch(mCameraRotateSpeed * mMouseYlast);
    }
    mMouseXlast = event->pos().x();
    mMouseYlast = event->pos().y();
}
