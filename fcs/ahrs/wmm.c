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

#include "../util/util.h"
#include "../util/3dmath.h"
#include "wmm.h"
#include "wmm2010_coeff.h"

static void E0000(int IENTRY, double alt, double glat, double glon,
double time, double *dec, double *dip, double *ti, double *gv);

static bool wmm_inited;

void fcs_wmm_init(void) {
    if (!wmm_inited) {
        /* Don't ask. */
        E0000(0, 0.0, 0.0, 0.0, 0.0, NULL, NULL, NULL, NULL);
        wmm_inited = true;
    }
}

bool fcs_wmm_calculate_field(double lat, double lon, double alt, double year,
double out_field[3]) {
    assert(wmm_inited);
    assert(out_field);
    _nassert((size_t)out_field % 8 == 0);

    double dec, dip, ti, gv;
    E0000(1, alt * 0.001, lat * (180.0/M_PI), lon * (180.0/M_PI), year, &dec,
          &dip, &ti, &gv);

    /* COMPUTE X, Y, Z, AND H COMPONENTS OF THE MAGNETIC FIELD */
    double rdec, rdip, c_rdip, x, y, z;
    rdec = dec  * (M_PI/180.0);
    rdip = dip  * (M_PI/180.0);
    c_rdip  = cos(rdip);
    x = ti * (cos(rdec) * c_rdip);
    y = ti * (c_rdip    * sin(rdec));
    z = ti * (sin(rdip));

    if (isnan(x) || isnan(y) || isnan(z)) {
        return false;
    } else {
        out_field[0] = x;
        out_field[1] = y;
        out_field[2] = z;
        return true;
    }
}

/*
As far as I'm concerned the below is basically just magic. It's exactly the
same as the version in geomag.c, but with the interactive and coefficient file
access bits removed since we've got everything in memory.
*/
#define MAXDEG 12

static void E0000(int IENTRY, double alt, double glat, double glon,
double time, double *dec, double *dip, double *ti, double *gv) {
    static int maxord,i,n,m,j,D1,D2,D3,D4;
    static double c[13][13],cd[13][13],tc[13][13],dp[13][13],snorm[169],
        sp[13],cp[13],fn[13],fm[13],pp[13],k[13][13],pi,dtr,a,b,re,
        a2,b2,c2,a4,b4,c4,flnmj,otime,oalt,
        olat,olon,dt,rlon,rlat,srlon,srlat,crlon,crlat,srlat2,
        crlat2,q,q1,q2,ct,st,r2,r,d,ca,sa,aor,ar,br,bt,bp,bpp,
        par,temp1,temp2,parp,bx,by,bz,bh;
    static double *p = snorm;

    switch(IENTRY){
        case 0: goto GEOMAG;
        case 1: goto GEOMG1;
    }

GEOMAG:
    /* INITIALIZE CONSTANTS */
    maxord = MAXDEG;
    sp[0] = 0.0;
    cp[0] = *p = pp[0] = 1.0;
    dp[0][0] = 0.0;
    a = 6378.137;
    b = 6356.7523142;
    re = 6371.2;
    a2 = a*a;
    b2 = b*b;
    c2 = a2-b2;
    a4 = a2*a2;
    b4 = b2*b2;
    c4 = a4 - b4;

    /* READ WORLD MAGNETIC MODEL SPHERICAL HARMONIC COEFFICIENTS */
    c[0][0] = 0.0;
    cd[0][0] = 0.0;

    for (i = 0; i < 90; i++) {
        n = (int)wmm_coeffs[i].n;
        m = (int)wmm_coeffs[i].m;
        if (m <= n)
        {
            c[m][n] = wmm_coeffs[i].gnm;
            cd[m][n] = wmm_coeffs[i].dgnm;
            if (m != 0) {
                c[n][m-1] = wmm_coeffs[i].hnm;
                cd[n][m-1] = wmm_coeffs[i].dhnm;
            }
        }
    }

    /* CONVERT SCHMIDT NORMALIZED GAUSS COEFFICIENTS TO UNNORMALIZED */
    *snorm = 1.0;
    fm[0] = 0.0;
    for (n=1; n<=maxord; n++) {
        *(snorm+n) = *(snorm+n-1)*(double)(2*n-1)/(double)n;
        j = 2;
        for (m=0,D1=1,D2=(n-m+D1)/D1; D2>0; D2--,m+=D1) {
            k[m][n] = (double)(((n-1)*(n-1))-(m*m))/(double)((2*n-1)*(2*n-3));
            if (m > 0) {
                flnmj = (double)((n-m+1)*j)/(double)(n+m);
                *(snorm+n+m*13) = *(snorm+n+(m-1)*13)*sqrt(flnmj);
                j = 1;
                c[n][m-1] = *(snorm+n+m*13)*c[n][m-1];
                cd[n][m-1] = *(snorm+n+m*13)*cd[n][m-1];
            }
            c[m][n] = *(snorm+n+m*13)*c[m][n];
            cd[m][n] = *(snorm+n+m*13)*cd[m][n];
        }
        fn[n] = (double)(n+1);
        fm[n] = (double)n;
    }
    k[1][1] = 0.0;

    otime = oalt = olat = olon = -1000.0;
    return;

/*************************************************************************/

GEOMG1:

    dt = time - wmm_epoch;

    pi = 3.14159265359;
    dtr = pi/180.0;
    rlon = glon*dtr;
    rlat = glat*dtr;
    srlon = sin(rlon);
    srlat = sin(rlat);
    crlon = cos(rlon);
    crlat = cos(rlat);
    srlat2 = srlat*srlat;
    crlat2 = crlat*crlat;
    sp[1] = srlon;
    cp[1] = crlon;

    /* CONVERT FROM GEODETIC COORDS. TO SPHERICAL COORDS. */
    if (alt != oalt || glat != olat) {
        q = sqrt(a2-c2*srlat2);
        q1 = alt*q;
        q2 = ((q1+a2)/(q1+b2))*((q1+a2)/(q1+b2));
        ct = srlat/sqrt(q2*crlat2+srlat2);
        st = sqrt(1.0-(ct*ct));
        r2 = (alt*alt)+2.0*q1+(a4-c4*srlat2)/(q*q);
        r = sqrt(r2);
        d = sqrt(a2*crlat2+b2*srlat2);
        ca = (alt+d)/r;
        sa = c2*crlat*srlat/(r*d);
    }
    if (glon != olon) {
        for (m=2; m<=maxord; m++) {
            sp[m] = sp[1]*cp[m-1]+cp[1]*sp[m-1];
            cp[m] = cp[1]*cp[m-1]-sp[1]*sp[m-1];
        }
    }
    aor = re/r;
    ar = aor*aor;
    br = bt = bp = bpp = 0.0;
    for (n=1; n<=maxord; n++) {
        ar = ar*aor;
        for (m=0,D3=1,D4=(n+m+D3)/D3; D4>0; D4--,m+=D3) {
/*
   COMPUTE UNNORMALIZED ASSOCIATED LEGENDRE POLYNOMIALS
   AND DERIVATIVES VIA RECURSION RELATIONS
*/
            if (alt != oalt || glat != olat) {
                if (n == m) {
                    *(p+n+m*13) = st**(p+n-1+(m-1)*13);
                    dp[m][n] = st*dp[m-1][n-1]+ct**(p+n-1+(m-1)*13);
                    goto S50;
                }
                if (n == 1 && m == 0) {
                    *(p+n+m*13) = ct**(p+n-1+m*13);
                    dp[m][n] = ct*dp[m][n-1]-st**(p+n-1+m*13);
                    goto S50;
                }
                if (n > 1 && n != m) {
                    if (m > n-2) {
                        *(p+n-2+m*13) = 0.0;
                    }
                    if (m > n-2) {
                        dp[m][n-2] = 0.0;
                    }
                    *(p+n+m*13) = ct**(p+n-1+m*13)-k[m][n]**(p+n-2+m*13);
                    dp[m][n] = ct*dp[m][n-1] -
                               st**(p+n-1+m*13)-k[m][n]*dp[m][n-2];
                }
            }
S50:
/*
    TIME ADJUST THE GAUSS COEFFICIENTS
*/
            if (time != otime) {
                tc[m][n] = c[m][n]+dt*cd[m][n];
                if (m != 0) {
                    tc[n][m-1] = c[n][m-1]+dt*cd[n][m-1];
                }
            }
/*
    ACCUMULATE TERMS OF THE SPHERICAL HARMONIC EXPANSIONS
*/
            par = ar**(p+n+m*13);
            if (m == 0) {
                temp1 = tc[m][n]*cp[m];
                temp2 = tc[m][n]*sp[m];
            } else {
                temp1 = tc[m][n]*cp[m]+tc[n][m-1]*sp[m];
                temp2 = tc[m][n]*sp[m]-tc[n][m-1]*cp[m];
            }
            bt = bt-ar*temp1*dp[m][n];
            bp += (fm[m]*temp2*par);
            br += (fn[n]*temp1*par);
/*
    SPECIAL CASE:  NORTH/SOUTH GEOGRAPHIC POLES
*/
            if (st == 0.0 && m == 1) {
                if (n == 1) {
                    pp[n] = pp[n-1];
                } else {
                    pp[n] = ct*pp[n-1]-k[m][n]*pp[n-2];
                }
                parp = ar*pp[n];
                bpp += (fm[m]*temp2*parp);
            }
        }
    }
    if (st == 0.0) {
        bp = bpp;
    } else {
        bp /= st;
    }
/*
    ROTATE MAGNETIC VECTOR COMPONENTS FROM SPHERICAL TO
    GEODETIC COORDINATES
*/
    bx = -bt*ca-br*sa;
    by = bp;
    bz = bt*sa-br*ca;
/*
    COMPUTE DECLINATION (DEC), INCLINATION (DIP) AND
    TOTAL INTENSITY (TI)
*/
    bh = sqrt((bx*bx)+(by*by));
    *ti = sqrt((bh*bh)+(bz*bz));
    *dec = atan2(by,bx)/dtr;
    *dip = atan2(bz,bh)/dtr;
/*
    COMPUTE MAGNETIC GRID VARIATION IF THE CURRENT
    GEODETIC POSITION IS IN THE ARCTIC OR ANTARCTIC
    (I.E. GLAT > +55 DEGREES OR GLAT < -55 DEGREES)

    OTHERWISE, SET MAGNETIC GRID VARIATION TO -999.0
*/
    *gv = -999.0;
    if (fabs(glat) >= 55.) {
        if (glat > 0.0 && glon >= 0.0) {
            *gv = *dec-glon;
        }
        if (glat > 0.0 && glon < 0.0) {
            *gv = *dec+fabs(glon);
        }
        if (glat < 0.0 && glon >= 0.0) {
            *gv = *dec+glon;
        }
        if (glat < 0.0 && glon < 0.0) {
            *gv = *dec-fabs(glon);
        }
        if (*gv > +180.0) {
            *gv -= 360.0;
        }
        if (*gv < -180.0) {
            *gv += 360.0;
        }
    }
    otime = time;
    oalt = alt;
    olat = glat;
    olon = glon;
    return;
}
