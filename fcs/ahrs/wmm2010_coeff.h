/*
This file is in the public domain. The data is copied from WMM.COF in the NOAA
WMM_Linux 2010 legacy distribution. The licence information attached to the
newer version of that code is:

    The WMM source code is in the public domain and not licensed or under
    copyright.

    The information and software may be used freely by the public. As required
    by 17 U.S.C. 403, third parties producing copyrighted works consisting
    predominantly of the material produced by U.S. government agencies must
    provide notice with such work(s) identifying the U.S. Government material
    incorporated and stating that such material is not subject to copyright
    protection.

    Software and Model Support
    National Geophysical Data Center
    NOAA EGC/2
    325 Broadway
    Boulder, CO 80303 USA
    Attn: Manoj Nair or Stefan Maus
    Phone:  (303) 497-4642 or -6522
    Email:  Manoj.C.Nair@Noaa.gov or Stefan.Maus@noaa.gov
    Web: http://www.ngdc.noaa.gov/geomag/WMM/

However, in relation to this code following disclaimer applies:

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

struct wmm_record_t {
    uint32_t n;
    uint32_t m;
    double gnm;
    double hnm;
    double dgnm;
    double dhnm;
};

static const double wmm_epoch = 2010.0;
static const struct wmm_record_t wmm_coeffs[] = {
    /* N   M        GNM        HNM        DGNM        DHNM */
    {  1,  0,  -29496.6,       0.0,       11.6,        0.0},
    {  1,  1,   -1586.3,    4944.4,       16.5,      -25.9},
    {  2,  0,   -2396.6,       0.0,      -12.1,        0.0},
    {  2,  1,    3026.1,   -2707.7,       -4.4,      -22.5},
    {  2,  2,    1668.6,    -576.1,        1.9,      -11.8},
    {  3,  0,    1340.1,       0.0,        0.4,        0.0},
    {  3,  1,   -2326.2,    -160.2,       -4.1,        7.3},
    {  3,  2,    1231.9,     251.9,       -2.9,       -3.9},
    {  3,  3,     634.0,    -536.6,       -7.7,       -2.6},
    {  4,  0,     912.6,       0.0,       -1.8,        0.0},
    {  4,  1,     808.9,     286.4,        2.3,        1.1},
    {  4,  2,     166.7,    -211.2,       -8.7,        2.7},
    {  4,  3,    -357.1,     164.3,        4.6,        3.9},
    {  4,  4,      89.4,    -309.1,       -2.1,       -0.8},
    {  5,  0,    -230.9,       0.0,       -1.0,        0.0},
    {  5,  1,     357.2,      44.6,        0.6,        0.4},
    {  5,  2,     200.3,     188.9,       -1.8,        1.8},
    {  5,  3,    -141.1,    -118.2,       -1.0,        1.2},
    {  5,  4,    -163.0,       0.0,        0.9,        4.0},
    {  5,  5,      -7.8,     100.9,        1.0,       -0.6},
    {  6,  0,      72.8,       0.0,       -0.2,        0.0},
    {  6,  1,      68.6,     -20.8,       -0.2,       -0.2},
    {  6,  2,      76.0,      44.1,       -0.1,       -2.1},
    {  6,  3,    -141.4,      61.5,        2.0,       -0.4},
    {  6,  4,     -22.8,     -66.3,       -1.7,       -0.6},
    {  6,  5,      13.2,       3.1,       -0.3,        0.5},
    {  6,  6,     -77.9,      55.0,        1.7,        0.9},
    {  7,  0,      80.5,       0.0,        0.1,        0.0},
    {  7,  1,     -75.1,     -57.9,       -0.1,        0.7},
    {  7,  2,      -4.7,     -21.1,       -0.6,        0.3},
    {  7,  3,      45.3,       6.5,        1.3,       -0.1},
    {  7,  4,      13.9,      24.9,        0.4,       -0.1},
    {  7,  5,      10.4,       7.0,        0.3,       -0.8},
    {  7,  6,       1.7,     -27.7,       -0.7,       -0.3},
    {  7,  7,       4.9,      -3.3,        0.6,        0.3},
    {  8,  0,      24.4,       0.0,       -0.1,        0.0},
    {  8,  1,       8.1,      11.0,        0.1,       -0.1},
    {  8,  2,     -14.5,     -20.0,       -0.6,        0.2},
    {  8,  3,      -5.6,      11.9,        0.2,        0.4},
    {  8,  4,     -19.3,     -17.4,       -0.2,        0.4},
    {  8,  5,      11.5,      16.7,        0.3,        0.1},
    {  8,  6,      10.9,       7.0,        0.3,       -0.1},
    {  8,  7,     -14.1,     -10.8,       -0.6,        0.4},
    {  8,  8,      -3.7,       1.7,        0.2,        0.3},
    {  9,  0,       5.4,       0.0,        0.0,        0.0},
    {  9,  1,       9.4,     -20.5,       -0.1,        0.0},
    {  9,  2,       3.4,      11.5,        0.0,       -0.2},
    {  9,  3,      -5.2,      12.8,        0.3,        0.0},
    {  9,  4,       3.1,      -7.2,       -0.4,       -0.1},
    {  9,  5,     -12.4,      -7.4,       -0.3,        0.1},
    {  9,  6,      -0.7,       8.0,        0.1,        0.0},
    {  9,  7,       8.4,       2.1,       -0.1,       -0.2},
    {  9,  8,      -8.5,      -6.1,       -0.4,        0.3},
    {  9,  9,     -10.1,       7.0,       -0.2,        0.2},
    { 10,  0,      -2.0,       0.0,        0.0,        0.0},
    { 10,  1,      -6.3,       2.8,        0.0,        0.1},
    { 10,  2,       0.9,      -0.1,       -0.1,       -0.1},
    { 10,  3,      -1.1,       4.7,        0.2,        0.0},
    { 10,  4,      -0.2,       4.4,        0.0,       -0.1},
    { 10,  5,       2.5,      -7.2,       -0.1,       -0.1},
    { 10,  6,      -0.3,      -1.0,       -0.2,        0.0},
    { 10,  7,       2.2,      -3.9,        0.0,       -0.1},
    { 10,  8,       3.1,      -2.0,       -0.1,       -0.2},
    { 10,  9,      -1.0,      -2.0,       -0.2,        0.0},
    { 10, 10,      -2.8,      -8.3,       -0.2,       -0.1},
    { 11,  0,       3.0,       0.0,        0.0,        0.0},
    { 11,  1,      -1.5,       0.2,        0.0,        0.0},
    { 11,  2,      -2.1,       1.7,        0.0,        0.1},
    { 11,  3,       1.7,      -0.6,        0.1,        0.0},
    { 11,  4,      -0.5,      -1.8,        0.0,        0.1},
    { 11,  5,       0.5,       0.9,        0.0,        0.0},
    { 11,  6,      -0.8,      -0.4,        0.0,        0.1},
    { 11,  7,       0.4,      -2.5,        0.0,        0.0},
    { 11,  8,       1.8,      -1.3,        0.0,       -0.1},
    { 11,  9,       0.1,      -2.1,        0.0,       -0.1},
    { 11, 10,       0.7,      -1.9,       -0.1,        0.0},
    { 11, 11,       3.8,      -1.8,        0.0,       -0.1},
    { 12,  0,      -2.2,       0.0,        0.0,        0.0},
    { 12,  1,      -0.2,      -0.9,        0.0,        0.0},
    { 12,  2,       0.3,       0.3,        0.1,        0.0},
    { 12,  3,       1.0,       2.1,        0.1,        0.0},
    { 12,  4,      -0.6,      -2.5,       -0.1,        0.0},
    { 12,  5,       0.9,       0.5,        0.0,        0.0},
    { 12,  6,      -0.1,       0.6,        0.0,        0.1},
    { 12,  7,       0.5,       0.0,        0.0,        0.0},
    { 12,  8,      -0.4,       0.1,        0.0,        0.0},
    { 12,  9,      -0.4,       0.3,        0.0,        0.0},
    { 12, 10,       0.2,      -0.9,        0.0,        0.0},
    { 12, 11,      -0.8,      -0.2,       -0.1,        0.0},
    { 12, 12,       0.0,       0.9,        0.1,        0.0}
};
