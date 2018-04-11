#include <cvd/gl_helpers.h>

#include <algorithm>
#include <cassert>
#include <cmath> 
#include <map>

using namespace std;

namespace CVD {

namespace Internal {

struct Point {
    float x,y;
};

struct Font {
    typedef unsigned short Index;

    struct Char {
        Index vertexOffset;
        Index triangleOffset;
        Index outlineOffset;
        GLsizei numTriangles;
        GLsizei numOutlines;
        float advance;
    };

    Point * vertices;
    Index * triangles;
    Index * outlines;
    Char * characters;
    string glyphs;

    const Char * findChar( const char c ) const {
        size_t ind = glyphs.find(c);
        if(ind == string::npos)
            return NULL;
        return characters + ind;
    }

    float getAdvance( const char c ) const {
        const Char * ch = findChar(c);
        if(!ch)
            return 0;
        return ch->advance;
    }

    void fill( const char c ) const {
        const Char * ch = findChar(c);
        if(!ch || !ch->numTriangles)
            return;
        glVertexPointer(2, GL_FLOAT, 0, vertices + ch->vertexOffset);
        glDrawElements(GL_TRIANGLES, ch->numTriangles, GL_UNSIGNED_SHORT, triangles + ch->triangleOffset);
    }

    void outline( const char c ) const {
        const Char * ch = findChar(c);
        if(!ch || !ch->numOutlines)
            return;
        glVertexPointer(2, GL_FLOAT, 0, vertices + ch->vertexOffset);
        glDrawElements(GL_LINES, ch->numOutlines, GL_UNSIGNED_SHORT, outlines + ch->outlineOffset);
    }

    void draw( const char c ) const {
        const Char * ch = findChar(c);
        if(!ch || !ch->numTriangles || !ch->numOutlines)
            return;
        glVertexPointer(2, GL_FLOAT, 0, vertices + ch->vertexOffset);
        glDrawElements(GL_TRIANGLES, ch->numTriangles, GL_UNSIGNED_SHORT, triangles + ch->triangleOffset);
        glDrawElements(GL_LINES, ch->numOutlines, GL_UNSIGNED_SHORT, outlines + ch->outlineOffset);
    }
};

// the fonts defined in these headers are derived from Bitstream Vera fonts. See http://www.gnome.org/fonts/ for license and details
#include "sans.h"
#include "mono.h"
#include "serif.h"


struct FontData {

    typedef map<string,Font *> FontMap;

    FontData() {
        fonts["sans"] = &sans_font;
        fonts["mono"] = &mono_font;
        fonts["serif"] = &serif_font;
        glSetFont("sans");
    }
    inline Font * currentFont(){
        return fonts[currentFontName];
    }

    string currentFontName;
    FontMap fonts;
};

static struct FontData data;

} // namespace Internal

void glSetFont( const std::string & fontname ){
    if(Internal::data.fonts.count(fontname) > 0)
        Internal::data.currentFontName = fontname;
}

const std::string & glGetFont(){
    return Internal::data.currentFontName;
}

std::pair<double,double> glDrawText(const std::string& text, enum TEXT_STYLE style, double spacing, double kerning){
    glPushMatrix();
    if(style == NICE) {
        glPushAttrib( GL_COLOR_BUFFER_BIT | GL_LINE_BIT );
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(1);
    }
    glEnableClientState(GL_VERTEX_ARRAY);

    // figure out which operation to do on the Char (yes, this is a pointer to member function :)
    void (Internal::Font::* operation)(const char c) const;
    switch(style){
        case FILL: operation = &Internal::Font::fill;
            break;
        case OUTLINE: operation = &Internal::Font::outline;
            break;
        case NICE: operation = &Internal::Font::draw;
            break;
        default: assert(false);
    }

    int lines = 0;
    double max_total = 0;
    double total=0;
    const Internal::Font * font = Internal::data.currentFont();
    const Internal::Font::Char * space = font->findChar(' ');
    const double tab_width = 8 * ((space)?(space->advance):1);
    for (size_t i=0; i<text.length(); ++i) {
        char c = text[i];
        if (c == '\n') {
            glTranslated(-total,-spacing, 0);
            max_total = std::max(max_total, total);
            total = 0;
            ++lines;
            continue;
        }
        if(c == '\t'){
            const float advance = tab_width - std::fmod(total, tab_width);
            total += advance;
            glTranslated(advance, 0, 0);
            continue;
        }
        const Internal::Font::Char * ch = font->findChar(c);
        if(!ch){
            c = toupper(c);
            ch = font->findChar(c);
            if(!ch) {
                c = '?';
                ch = font->findChar(c);
            }
        }
        if(!ch)
            continue;
        (font->*operation)(c);

        double w = ch->advance + kerning;
        glTranslated(w, 0, 0);
        total += w;
    }

    glDisableClientState(GL_VERTEX_ARRAY);
    if(style == NICE){
        glPopAttrib();
    }
    glPopMatrix();

    max_total = std::max(total, max_total);
    return std::make_pair(max_total, (lines+1)*spacing);
}

std::pair<double, double> glGetExtends(const std::string & text, double spacing, double kerning)
{
    int lines = 0;
    double max_total = 0;
    double total=0;
    const Internal::Font * font = Internal::data.currentFont();
    for (size_t i=0; i<text.length(); ++i) {
        char c = text[i];
        if (c == '\n') {
            max_total = std::max(max_total, total);
            total = 0;
            ++lines;
            continue;
        }
        const Internal::Font::Char * ch = font->findChar(c);
        if(!ch){
            c = toupper(c);
            ch = font->findChar(c);
            if(!ch) {
                c = '?';
                ch = font->findChar(c);
            }
        }
        if(!ch)
            continue;
        total += ch->advance + kerning;
    }
    max_total = std::max(total, max_total);
    return std::make_pair(max_total, (lines+1)*spacing);
}

} // namespace CVD
