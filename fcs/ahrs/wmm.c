/*
The code in this file is in the public domain. Most is derived from geomag.c
in the NOAA's WMM_Linux 2010 legacy distribution. The licence information
attached to the newer version of that code is:

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

While there is no license attached to the previous version of that code (on
which this file is actually based), it's safe to assume that the same
conditions apply.

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

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "../config/config.h"
#include "../util/util.h"
#include "../util/3dmath.h"

bool fcs_wmm_calculate_field(double lat, double lon, double alt, double year,
double out_field[3]) {
    out_field[0] = 1.0;
    out_field[1] = 0.0;
    out_field[2] = 0.0;
    return true;
}
