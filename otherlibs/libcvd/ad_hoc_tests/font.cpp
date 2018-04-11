#include <cvd/glwindow.h>
#include <cvd/gl_helpers.h>

using namespace std;

int main(int argc, char* argv[])
{

    CVD::GLWindow window(CVD::ImageRef(400,400), CVD::glGetFont());
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glColor3f(1, 1, 1);
    glLineWidth(1);

    CVD::GLWindow::EventSummary summary;
    double scale = 0.1;
    const double zoom = 1.02;
    std::string text = argc > 2 ? argv[2] : ("ABCDEFGHIJKLM\nNOPQRSTUVWXYZ\n"
                                             "abcdefghijklmnopqrstuvwxyz\n"
                                             "0123456789!@#$%^&*`|\n()_+-={}[]:\";'\\/?><,.~");

    CVD::glSetFont("mono");
    std::pair<double,double> size = CVD::glGetExtends(text, 1.5, 0.1);
    scale = 2 / (size.first + 1);

    bool outline = true;
    bool fill = true;
    while (!summary.should_quit())
    {
        glClear(GL_COLOR_BUFFER_BIT);
        glLoadIdentity();
        glTranslated(-0.95, 0, 0);
        glScaled(scale,scale,1);

        glPushMatrix();
        glTranslated(0, 0.85 / scale , 0);
        CVD::glSetFont("sans");
        if( fill )
            size = CVD::glDrawText(text,CVD::FILL,1.5, 0.1);
        if( outline )
            size = CVD::glDrawText(text, CVD::OUTLINE, 1.5, 0.1);
        glTranslated(0, 1.5, 0);
        glBegin(GL_LINE_STRIP);
        glVertex2d(-0.2,0);
        glVertex2d(size.first+0.2, 0);
        glVertex2d(size.first+0.2, -size.second - 0.5);
        glVertex2d(-0.2, -size.second - 0.5);
        glVertex2d(-0.2,0);
        glEnd();
        glPopMatrix();

        glPushMatrix();
        glTranslated(0, 0.2/scale , 0);
        CVD::glSetFont("serif");
        if( fill )
            size = CVD::glDrawText(text,CVD::FILL,1.5, 0.1);
        if( outline )
            size = CVD::glDrawText(text, CVD::OUTLINE, 1.5, 0.1);
        glTranslated(0, 1.5, 0);
        glBegin(GL_LINE_STRIP);
        glVertex2d(-0.2,0);
        glVertex2d(size.first+0.2, 0);
        glVertex2d(size.first+0.2, -size.second - 0.5);
        glVertex2d(-0.2, -size.second - 0.5);
        glVertex2d(-0.2,0);
        glEnd();
        glPopMatrix();

        glPushMatrix();
        glTranslated(0, -0.45 / scale , 0);
        CVD::glSetFont("mono");
        if( fill )
            size = CVD::glDrawText(text,CVD::FILL,1.5, 0.1);
        if( outline )
            size = CVD::glDrawText(text, CVD::OUTLINE, 1.5, 0.1);
        glTranslated(0, 1.5, 0);
        glBegin(GL_LINE_STRIP);
        glVertex2d(-0.2,0);
        glVertex2d(size.first+0.2, 0);
        glVertex2d(size.first+0.2, -size.second - 0.5);
        glVertex2d(-0.2, -size.second - 0.5);
        glVertex2d(-0.2,0);
        glEnd();
        glPopMatrix();

        window.swap_buffers();
        summary.clear();
        window.get_events(summary);
        if (summary.key_down.count('='))
            scale *= zoom;
        else if (summary.key_down.count('-'))
            scale *= 1.0/zoom;
        if(summary.key_down.count('o'))
            outline = !outline;
        if(summary.key_down.count('f'))
            fill = !fill;
    };
}
