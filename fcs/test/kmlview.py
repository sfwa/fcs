#!/usr/bin/python

import argparse
import plog
import math
from lxml import etree
from pykml.parser import Schema
from pykml.factory import KML_ElementMaker as KML
from pykml.factory import GX_ElementMaker as GX

if __name__ == '__main__':
    parser = argparse.ArgumentParser('Convert telemetry log to KML path')
    parser.add_argument('infile', help='file to convert')
    parser.add_argument('outfile', help='KML output file name')

    args = parser.parse_args()

    data = []
    with open(args.infile, 'r') as infile:
        for rec in plog.iterlogs(infile):
            for param in rec:
                pt = param.parameter_type
                if isinstance(param, plog.DataParameter):
                    pv = param.values
                else:
                    pv = param.value

                if pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_POSITION_LLA:
                    data.append(
                        ','.join((
                            # KML is lon, lat
                            str(math.degrees(pv[1] * math.pi / 2**31)),
                            str(math.degrees(pv[0] * math.pi / 2**31)),
                            str(45.0 + pv[2] * 1e-2)
                        ))
                    )

    doc = KML.kml(
        KML.Placemark(
            KML.name('flight path'),
            KML.LineString(
                GX.altitudeMode('absolute'),
                KML.coordinates(
                    '\n'.join([v for i, v in enumerate(data) if i % 100 == 0])
                )
            )
        )
    )

    # Write KML to disk.
    with open(args.outfile, 'w') as outfile:
        outfile.write(etree.tostring(doc, pretty_print=True))
