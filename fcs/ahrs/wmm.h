/*
Copyright (C) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef _FCS_WMM_H
#define _FCS_WMM_H

/*
Prepare derived values from the current WMM coefficients. Must be called
prior to `fcs_wmm_calculate_field`.
*/
void fcs_wmm_init(void);

/*
Calculate the magnetic field at the given point in space and time. The result
is copied to `out_field` as NED field components in nT.

The lat and lon values are in radians; alt is in metres above the WGS84
ellipsoid. The year value is a fractional Gregorian year, e.g. 1st July 2014
would be 2014.5.

If the conversion was successful, true is returned; otherwise, false.
*/
bool fcs_wmm_calculate_field(double lat, double lon, double alt, double year,
double out_field[3]);

#endif
