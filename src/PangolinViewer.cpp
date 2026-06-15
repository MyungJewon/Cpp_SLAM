#include "PangolinViewer.h"

#include <pangolin/pangolin.h>

#include <algorithm>
#include <chrono>
#include <thread>

namespace
{
std::array<float, 3> axisEndpoint(const Matrix3x3& rot,
                                  const std::array<float, 3>& pos,
                                  int axis,
                                  float length)
{
    return {
        pos[0] + rot[axis] * length,
        pos[1] + rot[3 + axis] * length,
        pos[2] + rot[6 + axis] * length
    };
}

void drawLine(const std::array<float, 3>& a, const std::array<float, 3>& b)
{
    glVertex3f(a[0], a[1], a[2]);
    glVertex3f(b[0], b[1], b[2]);
}
} // namespace

PangolinViewer::PangolinViewer()
{
    _rotation = {1.0f, 0.0f, 0.0f,
                 0.0f, 1.0f, 0.0f,
                 0.0f, 0.0f, 1.0f};
    _position = {0.0f, 0.0f, 0.0f};
}

PangolinViewer::~PangolinViewer()
{
}

void PangolinViewer::requestStop()
{
    _running = false;
}

bool PangolinViewer::shouldQuit() const
{
    return !_running;
}

void PangolinViewer::update(const std::vector<std::array<float,3>>& mapPoints,
                            const std::vector<std::array<float,3>>& trajectory,
                            const Matrix3x3& rotation,
                            const std::array<float,3>& position,
                            int frameIdx,
                            float icpError)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _mapPoints = mapPoints;
    _trajectory = trajectory;
    _rotation = rotation;
    _position = position;
    _frameIdx = frameIdx;
    _icpError = icpError;
    _dirty = true;
}

void PangolinViewer::runBlocking()
{
    pangolin::CreateWindowAndBind("SLAM Viewer", 1280, 720);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::View& panel = pangolin::CreatePanel("ui")
        .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(200));

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1080, 720, 500, 500, 540, 360, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -5, 3, 0, 0, 0, pangolin::AxisZ)
    );

    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0, -1080.0f / 720.0f)
        .SetHandler(&handler);

    pangolin::Var<int> uiFrame("ui.Frame", 0);
    pangolin::Var<float> uiX("ui.X", 0.0f);
    pangolin::Var<float> uiY("ui.Y", 0.0f);
    pangolin::Var<float> uiZ("ui.Z", 0.0f);
    pangolin::Var<float> uiIcpError("ui.ICP Error", 0.0f);
    pangolin::Var<int> uiMapPoints("ui.Map Points", 0);

    (void)panel;

    std::vector<std::array<float,3>> mapPoints;
    std::vector<std::array<float,3>> trajectory;
    Matrix3x3 rot = {1.0f, 0.0f, 0.0f,
                     0.0f, 1.0f, 0.0f,
                     0.0f, 0.0f, 1.0f};
    std::array<float,3> pos = {0.0f, 0.0f, 0.0f};
    int frameIdx = 0;
    float icpError = 0.0f;

    while (_running && !pangolin::ShouldQuit())
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            mapPoints = _mapPoints;
            trajectory = _trajectory;
            rot = _rotation;
            pos = _position;
            frameIdx = _frameIdx;
            icpError = _icpError;
            _dirty = false;
        }

        uiFrame = frameIdx;
        uiX = pos[0];
        uiY = pos[1];
        uiZ = pos[2];
        uiIcpError = icpError;
        uiMapPoints = static_cast<int>(mapPoints.size());

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();
        Twc.m[0]=rot[0]; Twc.m[1]=rot[3]; Twc.m[2]=rot[6];  Twc.m[3]=0;
        Twc.m[4]=rot[1]; Twc.m[5]=rot[4]; Twc.m[6]=rot[7];  Twc.m[7]=0;
        Twc.m[8]=rot[2]; Twc.m[9]=rot[5]; Twc.m[10]=rot[8]; Twc.m[11]=0;
        Twc.m[12]=pos[0]; Twc.m[13]=pos[1]; Twc.m[14]=pos[2]; Twc.m[15]=1;
        s_cam.Follow(Twc);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.05f, 0.05f, 0.05f, 1.0f);

        d_cam.Activate(s_cam);

        glPointSize(2.0f);
        glColor3f(0.4f, 0.5f, 0.6f);
        glBegin(GL_POINTS);
        for (const auto& p : mapPoints)
            glVertex3f(p[0], p[1], p[2]);
        glEnd();

        if (trajectory.size() >= 2)
        {
            glLineWidth(3.0f);
            glBegin(GL_LINE_STRIP);
            const float denom = static_cast<float>(std::max<size_t>(trajectory.size() - 1, 1));
            for (size_t i = 0; i < trajectory.size(); ++i)
            {
                const float ratio = static_cast<float>(i) / denom;
                glColor3f(ratio, 1.0f - ratio, 0.0f);
                const auto& p = trajectory[i];
                glVertex3f(p[0], p[1], p[2]);
            }
            glEnd();
        }

        const float axisLen = 0.3f;
        const auto xEnd = axisEndpoint(rot, pos, 0, axisLen);
        const auto yEnd = axisEndpoint(rot, pos, 1, axisLen);
        const auto zEnd = axisEndpoint(rot, pos, 2, axisLen);

        glLineWidth(4.0f);
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);
        drawLine(pos, xEnd);
        glColor3f(0.0f, 1.0f, 0.0f);
        drawLine(pos, yEnd);
        glColor3f(0.0f, 0.0f, 1.0f);
        drawLine(pos, zEnd);
        glEnd();

        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    _running = false;
}
