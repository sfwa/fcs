/*
*   imageio.cpp Copyright (C) 2009-2010 Paolo Medici
*
*   This library is free software; you can redistribute it and/or modify it under the terms of the
*    GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of
*    the License, or (at your option) any later version.
*
*   This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
*    implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
*    License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License along with this library; if not,
*   write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

// Build: g++ -O3 -o debayer -stdlib=libc++ debayer.cpp

#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>

using namespace std;

float rgb2L(float r, float g, float b) {
    float x, y, z;

    if (r > 0.04045f) {
        r = powf((r + 0.055f) * (1.0f / 1.055f), 2.4f);
    } else {
        r = r / 12.92f;
    }

    if (g > 0.04045f) {
        g = powf((g + 0.055f) * (1.0f / 1.055f), 2.4f);
    } else {
        g = g / 12.92f;
    }

    if (b > 0.04045f) {
        b = powf((b + 0.055f) * (1.0f / 1.055f), 2.4f);
    } else {
        b = b / 12.92f;
    }

    // Observer. = 2°, Illuminant = D65
    y = r * 0.2126f + g * 0.7152f + b * 0.0722f;

    if (y > 0.008856f) {
        y = powf(y, 1.0f / 3.0f);
    } else {
        y = (7.787f * y) + (16.0f / 116.0f);
    }

    return (116.0f * y) - 16.0f;
}

float rgb2h(float r, float g, float b) {
    float minv, maxv, deltav, h, s, v, dr, dg, db;

    minv = min(min(r, g), b);    // Min. value of RGB
    maxv = max(max(r, g), b);    // Max. value of RGB
    deltav = maxv - minv;   // Delta RGB value

    v = maxv;

    if (deltav == 0.0f) {   // This is a gray, no chroma...
        h = 0.0f;           // HSV results from 0 to 1
        s = 0.0f;
    } else {                // Chromatic data...
        s = deltav / maxv;

        dr = (((maxv - r) * (1.0f / 6.0f)) + (deltav * 0.5f)) / deltav;
        dg = (((maxv - g) * (1.0f / 6.0f)) + (deltav * 0.5f)) / deltav;
        db = (((maxv - b) * (1.0f / 6.0f)) + (deltav * 0.5f)) / deltav;

        if (r == maxv) {
            h = db - dg;
        } else if (g == maxv) {
            h = (1.0f / 3.0f) + dr - db;
        } else if (b == maxv) {
            h = (2.0f / 3.0f) + dg - dr;
        }

        if (h < 0.0f) {
            h += 1.0f;
        }
        if (h > 1.0f) {
            h -= 1.0f;
        }
    }

    return h;
}

float hue_dist(float h, float ref) {
    return 1.0f - 2.0f * min(0.5f, fabs(fmod(1.0f + h - ref, 1.0f) - 1.0f));
}

// color format
enum ColorFormat {
    AUTO,   // reserved
    GREY8,  // 1 byte per pixel
    GREY16, // 2 bytes per pixel
    RGB24,  // 3 bytes per pixel
    RGB48   // 6 bytes per pixel
};

// Image class
class Image {

    // The image format
    ColorFormat m_format;

    // The Image Geomtry
    unsigned int m_width, m_height;

    // Image stride
    long m_stride;

    // The inner buffer
    unsigned int *m_buffer;

private:
    static void pnm_skip_comments(std::istream& i) {
        while (isspace(i.peek())) {
            while (isspace(i.peek())) {
                i.get();
            }

            if (i.peek() == '#') {
                while (i.peek()!='\r' && i.peek()!='\n') {
                    i.get();
                }
            }
        }
    }

    static char pnm_read_header(std::istream& iss, unsigned int& width, unsigned int& height, unsigned int& max_val) {
        char h, t;

        // check pnm header
        h = iss.get();
        t = iss.get();
        if (!((h == 'P') && ((t == '5') || (t == '6')))) {
            return '\0';
        }

        pnm_skip_comments(iss);
        iss >> width;
        pnm_skip_comments(iss);
        iss >> height;
        pnm_skip_comments(iss);
        iss >> max_val;
        iss.get(); // TODO: use a getline fn

        return t;
    }

    // private costructor to avoid copy
    Image(const Image & prv) { }

public:
    Image() : m_width(0), m_height(0), m_buffer(0) { }

    ~Image() {
        delete [] m_buffer;
    }

    // destroy image and realloc buffer
    bool alloc(unsigned int width, unsigned int height, ColorFormat format) {
        delete [] m_buffer;
        // buffer size
        m_width = width; m_height = height; m_format = format;
        m_stride = m_width * channels_per_pixel();
        m_buffer = new unsigned int [m_stride * m_height];
        return true;
    }

    inline unsigned int channels_per_pixel() const {
        if (m_format == RGB48) {
            return 3;
        } else if (m_format == RGB24) {
            return 3;
        } else if (m_format == GREY16) {
            return 1;
        } else if (m_format == GREY8) {
            return 1;
        } else {
            return 0;
        }
    }

    // return the bytes per pixel
    inline unsigned int bytes_per_pixel() const {
        if (m_format == RGB48) {
            return 6;
        } else if (m_format == RGB24) {
            return 3;
        } else if (m_format == GREY16) {
            return 2;
        } else if (m_format == GREY8) {
            return 1;
        } else {
            return 0;
        }
    }

    // return the image size in bytes
    inline unsigned int memsize() const {
        return m_width * m_height * bytes_per_pixel();
    }

    // return the image size in pixels
    inline unsigned int size() const {
        return m_width * m_height;
    }

    // image width
    inline unsigned int width() const {
        return m_width;
    }

    // image height
    inline unsigned int height() const {
        return m_height;
    }

    // color format
    inline ColorFormat format() const {
        return m_format;
    }

    // data access
    inline const unsigned int *data() const {
        return m_buffer;
    }
    inline unsigned int *data() {
        return m_buffer;
    }

    // data access
    inline unsigned int operator [] (int i)  const {
        return m_buffer[i];
    }

    inline unsigned int & operator [] (int i) {
        return m_buffer[i];
    }

    // data access (column, row)
    inline unsigned int operator () (int i, int j)  const {
        return m_buffer[i + j * m_stride ];
    }

    inline unsigned int & operator () (int i, int j) {
        return m_buffer[i + j * m_stride ];
    }

    // Load a PGM/PPM image, reserve memory with new and return geometry
    // @param [in] file filename
    // @return true if image is loaded correctly
    bool load(std::istream& istr, ColorFormat format = AUTO) {
        unsigned int width, height, max_val, size, i;
        char header = pnm_read_header(istr, width, height, max_val);
        unsigned char *buffer;

        if (header != '5' && header != '6') {
            std::cerr << "Unsupported format " << header <<
                         "; only 5 or 6 are allowed." << std::endl;
            return false;
        }

        if (format == AUTO) {
            if (max_val == 255) {
                if (header == '5') {
                    format = GREY8;
                } else if (header == '6') {
                    format = RGB24;
                }
            } else if (max_val == 65535) {
                if (header == '5') {
                    format = GREY16;
                } else if (header == '6') {
                    format = RGB48;
                }
            }
        }

        if (format == AUTO) {
            std::cerr << "Unsupported maximum channel value " << max_val <<
                         "; only 255 or 65535 are allowed." << std::endl;
            return false;
        }

        alloc(width, height, format);
        buffer = new unsigned char[memsize()];
        istr.read(reinterpret_cast<char *>(buffer), memsize());

        if (format == GREY16 || format == RGB48) {
            for (i = 0; i < m_stride * m_height; i++) {
                m_buffer[i] = (buffer[(i << 1)] << 8) + buffer[(i << 1) + 1];
            }
        } else {
            for (i = 0; i < m_stride * m_height; i++) {
                m_buffer[i] = buffer[i];
            }
        }

        delete [] buffer;

        return true;
    }

    // Write a PGM/PPM file
    // @param filename a file
    // @return true if file is created

    bool save(std::ostream& out) const {
        unsigned char *buffer;
        unsigned int i;

        buffer = new unsigned char[memsize()];

        if (m_format == GREY16 || m_format == RGB48) {
            for (i = 0; i < m_stride * m_height; i++) {
                buffer[(i << 1)] = m_buffer[i] >> 8;
                buffer[(i << 1) + 1] = m_buffer[i] & 0xff;
            }
        } else {
            for (i = 0; i < m_stride * m_height; i++) {
                buffer[i] = m_buffer[i];
            }
        }

        out << "P" << (channels_per_pixel() == 1 ? '5' : '6') << ' ' <<
               m_width << ' ' << m_height << ' ' <<
               (bytes_per_pixel() > channels_per_pixel() ? "65535" : "255") << "\n";
        out.write(reinterpret_cast<const char *>(buffer), memsize());

        delete [] buffer;

        return true;
    }
};


////////////////////////////// Example of uses //////////////////////////

///////////////////////////////////////////////////// PM DEBAYER //////////////////////////////////

struct pBGGR;
struct pGRBG;
struct pGBRG;
struct pRGGB;

struct pBGGR {
    static unsigned int R(const unsigned int *src, unsigned int w) {
        return src[w+1];
    }
    static unsigned int G1(const unsigned int *src, unsigned int w) {
        return src[1];
    }
    static unsigned int G2(const unsigned int *src, unsigned int w) {
        return src[w];
    }
    static unsigned int B(const unsigned int *src, unsigned int w) {
        return src[0];
    }
    typedef pGBRG horz;
    typedef pGRBG vert;
};

struct pGBRG {
    static unsigned int R(const unsigned int *src, unsigned int w) {
        return src[w];
    }
    static unsigned int G1(const unsigned int *src, unsigned int w) {
        return src[0];
    }
    static unsigned int G2(const unsigned int *src, unsigned int w) {
        return src[w+1];
    }
    static unsigned int B(const unsigned int *src, unsigned int w) {
        return src[1];
    }
    typedef pBGGR horz;
    typedef pRGGB vert;
};

struct pGRBG {
    static unsigned int R(const unsigned int *src, unsigned int w) {
        return src[1];
    }
    static unsigned int G1(const unsigned int *src, unsigned int w) {
        return src[0];
    }
    static unsigned int G2(const unsigned int *src, unsigned int w) {
        return src[w+1];
    }
    static unsigned int B(const unsigned int *src, unsigned int w) {
        return src[w];
    }
    typedef pRGGB horz;
    typedef pBGGR vert;
};

struct pRGGB {
    static unsigned int R(const unsigned int *src, unsigned int w) {
        return src[0];
    }
    static unsigned int G1(const unsigned int *src, unsigned int w) {
        return src[1];
    }
    static unsigned int G2(const unsigned int *src, unsigned int w) {
        return src[w];
    }
    static unsigned int B(const unsigned int *src, unsigned int w) {
        return src[w+1];
    }
    typedef pGRBG horz;
    typedef pGBRG vert;
};

// PM Bayer Simple! (Because Simply is the best)
template<class T>
void bayer_simple(const unsigned int *src, unsigned int *dst, unsigned int w, unsigned int h) {
    unsigned int i,j;
    T a;
    typename T::horz b;
    typename T::vert c;
    typename T::horz::vert d;

    for (i = 0; i < h - 2; i += 2) {
        for (j = 0; j < w - 2; j += 2) {
            dst[0] = a.R(src, w);
            dst[1] = (a.G1(src, w) + a.G2(src, w)) >> 1;
            dst[2] = a.B(src, w);
            dst +=3;
            src++;

            dst[0] = b.R(src, w);
            dst[1] = (b.G1(src, w) + b.G2(src, w)) >> 1;
            dst[2] = b.B(src, w);
            dst +=3;
            src++;
        }

        // FIX: ultima colonna
        dst[3] = dst[0] = a.R(src, w);
        dst[4] = dst[1] = (a.G1(src, w) + a.G2(src, w)) >> 1;
        dst[5] = dst[2] = a.B(src, w);
        dst += 6;
        src += 2;

        for (j = 0; j < w - 2; j += 2) {
            dst[0] = c.R(src, w);
            dst[1] = (c.G1(src, w) + c.G2(src, w)) >> 1;
            dst[2] = c.B(src, w);
            dst += 3;
            src++;

            dst[0] = d.R(src, w);
            dst[1] = (d.G1(src, w) + d.G2(src, w)) >> 1;
            dst[2] = d.B(src, w);
            dst += 3;
            src++;
        }

        // FIX: ultima colonna
        dst[3] = dst[0] = c.R(src, w);
        dst[4] = dst[1] = (c.G1(src, w) + c.G2(src, w)) >> 1;
        dst[5] = dst[2] = c.B(src, w);
        dst += 6;
        src += 2;
    }

    // FIX: ultima riga!
    for (j = 0; j < w - 2; j += 2) {
        dst[w] = dst[0] = a.R(src, w);
        dst[w+1] = dst[1] = (a.G1(src, w) + a.G2(src, w)) >> 1;
        dst[w+2] = dst[2] = a.B(src, w);
        dst += 3;
        src++;

        dst[w] = dst[0] = b.R(src, w);
        dst[w+1] = dst[1] = (b.G1(src, w) + b.G2(src, w)) >> 1;
        dst[w+2] = dst[2] = b.B(src, w);
        dst += 3;
        src++;
    }

    // FIx: ultima riga, ultima colonna:
    dst[w+3] = dst[w] = dst[3] = dst[0] = a.R(src, w);
    dst[w+4] = dst[w+1] = dst[4] = dst[1] = (a.G1(src, w) + a.G2(src, w)) >> 1;
    dst[w+5] = dst[w+2] = dst[5] = dst[2] = a.B(src, w);
}

int16_t kernel[144] = {
    -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3,
    -3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -3,
    -3, -1, -1, -1, -1,  1,  1, -1, -1, -1, -1, -3,
    -3, -1, -1, -1,  1,  1,  1,  1, -1, -1, -1, -3,
    -3, -1, -1,  1,  1,  1,  1,  1,  1, -1, -1, -3,
    -3, -1,  1,  1,  1,  1,  1,  1,  1,  1, -1, -3,
    -3, -1,  1,  1,  1,  1,  1,  1,  1,  1, -1, -3,
    -3, -1, -1,  1,  1,  1,  1,  1,  1, -1, -1, -3,
    -3, -1, -1, -1,  1,  1,  1,  1, -1, -1, -1, -3,
    -3, -1, -1, -1, -1,  1,  1, -1, -1, -1, -1, -3,
    -3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -3,
    -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3
};

int16_t l_kernel[144] = {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1,  1,  1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1,  1,  1,  1,  1, -1, -1, -1, -1,
    -1, -1, -1,  1,  1,  3,  3,  1,  1, -1, -1, -1,
    -1, -1,  1,  1,  3,  3,  3,  3,  1,  1, -1, -1,
    -1, -1,  1,  1,  3,  3,  3,  3,  1,  1, -1, -1,
    -1, -1, -1,  1,  1,  3,  3,  1,  1, -1, -1, -1,
    -1, -1, -1, -1,  1,  1,  1,  1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1,  1,  1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
};

int32_t apply_kernel(int16_t *kernel, uint16_t *src, size_t kernel_dim, size_t src_width) {
    int32_t result = 0;
    size_t i, j;

    for (i = 0; i < kernel_dim; i++) {
        for (j = 0; j < kernel_dim; j++) {
            result += (int32_t)kernel[i * kernel_dim + j] * (int32_t)src[i * src_width + j];
        }
    }

    return result;
}

int main(int argc, char *argv[]) {
    if (argc == 2) {
        Image img;

        if (!img.load(std::cin)) {
            std::cerr << "Couldn't read image from stdin." << std::endl;
        } else {
            Image imgRGB;

            imgRGB.alloc(img.width(), img.height(), RGB48);

            if (!strcmp(argv[1], "RGGB")) {
                bayer_simple<pRGGB>(img.data(), imgRGB.data(), img.width(), img.height());
            } else if (!strcmp(argv[1], "BGGR")) {
                bayer_simple<pBGGR>(img.data(), imgRGB.data(), img.width(), img.height());
            } else if (!strcmp(argv[1], "GRBG")) {
                bayer_simple<pGRBG>(img.data(), imgRGB.data(), img.width(), img.height());
            } else if (!strcmp(argv[1], "GBRG")) {
                bayer_simple<pGBRG>(img.data(), imgRGB.data(), img.width(), img.height());
            } else {
                std::cerr <<
                    "Invalid Bayer pattern; use RGGB, BGGR, GRBG or GBRG." <<
                    std::endl;
            }

            imgRGB.save(std::cout);

            uint16_t *hue = new uint16_t[imgRGB.width() * imgRGB.height()];
            uint16_t *blue = new uint16_t[imgRGB.width() * imgRGB.height()];
            uint16_t *yellowgreen = new uint16_t[imgRGB.width() * imgRGB.height()];
            uint16_t *L = new uint16_t[imgRGB.width() * imgRGB.height()];
            uint16_t L_offset, max_L, blue_offset, max_blue, yellowgreen_offset, max_yellowgreen;
            float r, g, b, s = 1.0f / 65535.0f, hval,
                  L_scale, blue_scale, yellowgreen_scale;
            uint32_t *L_hist = new uint32_t[65536], accum, pm999, pm900;
            uint32_t *blue_hist = new uint32_t[65536];
            uint32_t *yellowgreen_hist = new uint32_t[65536];
            int32_t result;
            size_t i, j, d = imgRGB.channels_per_pixel(), w = imgRGB.width(),
                   h = imgRGB.height(), l = w * h;

            memset(L_hist, 0, 65536 * 4);
            memset(blue_hist, 0, 65536 * 4);
            memset(yellowgreen_hist, 0, 65536 * 4);

            for (i = 0; i < l; i++) {
                r = (float)imgRGB[i * d] * s;
                g = (float)imgRGB[i * d + 1] * s;
                b = (float)imgRGB[i * d + 2] * s;
                hval = min(max(0.0f, rgb2h(r, g, b)), 1.0f);

                hue[i] = hval * 65535.0f;
                L[i] = min(max(0.0f, rgb2L(r, g, b) * 0.01f), 1.0f) * 65535.0f;
                blue[i] = hue_dist(hval, 0.66f) * 65535.0f;
                yellowgreen[i] = hue_dist(hval, 0.0875f) * 65535.0f;

                L_hist[L[i]]++;
            }

            // Find the 90th percentile L value
            for (i = 65535, accum = 0, pm999 = 0, pm900 = 0; i > 0 && (!pm999 || !pm900); i--) {
                accum += L_hist[i];
                if (pm999 == 0 && accum > l / 1000) {
                    pm999 = i;
                }
                if (pm900 == 0 && accum > l / 10) {
                    pm900 = i;
                }
            }

            L_offset = pm900;
            max_L = pm999;
            if (L_offset < max_L) {
                L_scale = 65535.0f / (max_L - L_offset);
            } else {
                L_scale = 1.0f;
                L_offset = 0;
            }

            for (i = 0; i < l; i++) {
                L[i] = L[i] > L_offset ? L[i] - L_offset : 0;
                L[i] = min((float)L[i] * L_scale * (float)L[i] * L_scale, 65535.0f * 65535.0f) * (1.0f / 65535.0f);

                if (L[i] < 32768) {
                    L[i] = 0;
                }

                blue[i] = (uint16_t)(((uint32_t)L[i] * (uint32_t)blue[i]) >> 16u);
                yellowgreen[i] = (uint16_t)(((uint32_t)L[i] * (uint32_t)yellowgreen[i]) >> 16u);

                blue_hist[blue[i]]++;
                yellowgreen_hist[yellowgreen[i]]++;
            }

            for (i = 0; i < h - 12; i++) {
                for (j = 0; j < w - 12; j++) {
                    result = apply_kernel(l_kernel, &L[i * w + j], 12u, w);
                    if (result > 25 * 65535) {
                        std::cerr << "('L', " << (j + 6) << ", " << (i + 6) << ", " << result << ")" << std::endl;
                    }
                }
            }

            // Find the 90th percentile blue value
            for (i = 65535, accum = 0, pm999 = 0, pm900 = 0; i > 0 && (!pm999 || !pm900); i--) {
                accum += blue_hist[i];
                if (pm999 == 0 && accum > l / 10000) {
                    pm999 = i;
                }
                if (pm900 == 0 && accum > l / 10) {
                    pm900 = i;
                }
            }

            blue_offset = pm900;
            max_blue = pm999;
            if (blue_offset < max_blue) {
                blue_scale = 65535.0f / (max_blue - blue_offset);
            } else {
                blue_scale = 1.0f;
                blue_offset = 0;
            }

            // Multiply blue/yellowgreen maps by luminance value, and then zero any
            // with a hue histogram frequency of zero
            for (i = 0; i < l; i++) {
                blue[i] = blue[i] > blue_offset ? blue[i] - blue_offset : 0;
                blue[i] = min(blue[i] * blue_scale, 65535.0f);
            }

            // Apply the kernel and locate the best blobs
            for (i = 0; i < h - 12; i++) {
                for (j = 0; j < w - 12; j++) {
                    result = apply_kernel(kernel, &blue[i * w + j], 12u, w);
                    if (result > 5 * 65535) {
                        std::cerr << "('b', " << (j + 6) << ", " << (i + 6) << ", " << result << ")" << std::endl;
                    }
                }
            }

            // Find the 90th percentile yellow-green value
            for (i = 65535, accum = 0, pm999 = 0, pm900 = 0; i > 0 && (!pm999 || !pm900); i--) {
                accum += yellowgreen_hist[i];
                if (pm999 == 0 && accum > l / 10000) {
                    pm999 = i;
                }
                if (pm900 == 0 && accum > l / 100) {
                    pm900 = i;
                }
            }

            yellowgreen_offset = pm900;
            max_yellowgreen = pm999;
            if (yellowgreen_offset < max_yellowgreen) {
                yellowgreen_scale = 65535.0f / (max_yellowgreen - yellowgreen_offset);
            } else {
                yellowgreen_scale = 1.0f;
                yellowgreen_offset = 0;
            }

            for (i = 0; i < l; i++) {
                yellowgreen[i] = yellowgreen[i] > yellowgreen_offset ? yellowgreen[i] - yellowgreen_offset : 0;
                yellowgreen[i] = min(yellowgreen[i] * yellowgreen_scale, 65535.0f);
            }

            for (i = 0; i < h - 12; i++) {
                for (j = 0; j < w - 12; j++) {
                    result = apply_kernel(kernel, &yellowgreen[i * w + j], 12u, w);
                    if (result > 5 * 65535) {
                        std::cerr << "('y', " << (j + 6) << ", " << (i + 6) << ", " << result << ")" << std::endl;
                    }
                }
            }

//            // Debug
//            Image imgH, img_blue, img_yellowgreen, imgL;
//            imgH.alloc(imgRGB.width(), imgRGB.height(), GREY16);
//            img_blue.alloc(imgRGB.width(), imgRGB.height(), GREY16);
//            img_yellowgreen.alloc(imgRGB.width(), imgRGB.height(), GREY16);
//            imgL.alloc(imgRGB.width(), imgRGB.height(), GREY16);
//
//            for (i = 0; i < l; i++) {
//                imgH[i] = hue[i];
//                img_blue[i] = blue[i];
//                img_yellowgreen[i] = yellowgreen[i];
//                imgL[i] = L[i];
//            }
//
//            std::ofstream Hf, bluef, yellowgreenf, Lf;
//
//            Hf.open("/Users/bendyer/Desktop/debayer-H.pgm", ios::out | ios::binary);
//            imgH.save(Hf);
//            Hf.close();
//
//            bluef.open("/Users/bendyer/Desktop/debayer-blue.pgm", ios::out | ios::binary);
//            img_blue.save(bluef);
//            bluef.close();
//
//            yellowgreenf.open("/Users/bendyer/Desktop/debayer-yellowgreen.pgm", ios::out | ios::binary);
//            img_yellowgreen.save(yellowgreenf);
//            yellowgreenf.close();
//
//            Lf.open("/Users/bendyer/Desktop/debayer-L.pgm", ios::out | ios::binary);
//            imgL.save(Lf);
//            Lf.close();
        }
    } else {
        std::cerr << "debayer <Bayer pattern>" << std::endl;
    }

    return 0;
}
