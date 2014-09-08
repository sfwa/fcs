#!/usr/bin/python

import argparse
import plog
import math
import colorsys
import datetime

if __name__ == '__main__':
    parser = argparse.ArgumentParser('Convert telemetry log to KML path')
    parser.add_argument('infile', help='file to convert')
    parser.add_argument('outfile', help='KML output file name')

    args = parser.parse_args()

    path_id = []
    path_data = []
    path_airspeed = []
    path_throttle = []
    path_left_elevon = []
    path_right_elevon = []
    path_obj_val = []
    ref_path = []
    ref_path_airspeed = []
    last_ref_point = None
    last_path = 0
    last_auto = 0
    with open(args.infile, 'r') as infile:
        for t, rec in enumerate(plog.iterlogs(infile)):
            if t % 1000:
                continue

            path = None
            point = None
            attitude = None
            airspeed = None
            ref_point = None
            ref_attitude = None
            ref_airspeed = None
            velocity = None
            wind = None
            control_pos = None
            auto = None
            control_cycles = None
            control_obj_val = None
            control_errors = None
            control_resets = None
            mode = None
            time_when = '<when>' + datetime.datetime.fromtimestamp(float(t) / 1000.0).isoformat() + '</when>'

            for param in rec:
                pt = param.parameter_type
                if isinstance(param, plog.DataParameter):
                    pv = param.values
                else:
                    pv = param.value

                if pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_POSITION_LLA:
                    point =  '<gx:coord>{0} {1} {2:.2f}</gx:coord>'.format(
                        # KML is lon, lat
                        math.degrees(pv[1] * math.pi / 2**31),
                        math.degrees(pv[0] * math.pi / 2**31),
                        pv[2] * 1e-2
                    )
                elif pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_VELOCITY_NED:
                    velocity = map(lambda x: float(x) * 1e-2, pv)
                elif pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_ATTITUDE_Q:
                    att_q = map(lambda x: float(x) / 32767.0, pv)
                    attitude = '<gx:angles>{0:.1f} {1:.1f} {2:.1f}</gx:angles>'.format(*plog.q_to_euler(att_q))
                elif pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED:
                    wind = map(lambda x: float(x) * 1e-2, pv)
                elif pt == plog.ParameterType.FCS_PARAMETER_CONTROL_POS:
                    control_pos = map(lambda x: float(x) / 65535.0, pv)
                elif pt == plog.ParameterType.FCS_PARAMETER_KEY_VALUE:
                    ref_waypoint = plog.extract_waypoint(pv)
                    ref_point = '<gx:coord>{0} {1} {2:.2f}</gx:coord>'.format(
                        math.degrees(ref_waypoint['lon']),
                        math.degrees(ref_waypoint['lat']),
                        ref_waypoint['alt']
                    )
                    ref_attitude = '<gx:angles>{0:.1f} {1:.1f} {2:.1f}</gx:angles>'.format(
                        math.degrees(ref_waypoint['yaw']),
                        math.degrees(ref_waypoint['pitch']),
                        math.degrees(ref_waypoint['roll'])
                    )
                    ref_airspeed = '<gx:value>{0:.2f}</gx:value>'.format(ref_waypoint['airspeed'])
                elif pt == plog.ParameterType.FCS_PARAMETER_CONTROL_MODE:
                    if param.device_id == 1:
                        auto = pv[0]
                elif pt == plog.ParameterType.FCS_PARAMETER_NAV_PATH_ID:
                    path = pv[0]
                elif pt == plog.ParameterType.FCS_PARAMETER_CONTROL_STATUS:
                    control_cycles = pv[0]
                    control_obj_val = pv[1]
                    control_errors = pv[2]
                    control_resets = pv[3]
                elif pt == plog.ParameterType.FCS_PARAMETER_AHRS_MODE:
                    mode = chr(pv[0])

            if ref_point == last_ref_point or not point or not ref_point \
                    or not path or control_obj_val is None \
                    or control_pos is None or mode not in ('A', 'R'):
                continue

            airspeed = math.sqrt((velocity[0] - wind[0])**2 +
                                 (velocity[1] - wind[1])**2 +
                                 (velocity[2] - wind[2])**2)
            airspeed = '<gx:value>{0:.2f}</gx:value>'.format(airspeed)

            ref_path.append(time_when + ref_point + ref_attitude)
            ref_path_airspeed.append(ref_airspeed)

            if auto is None:
                auto = last_auto

            if not auto:
                path = 0

            path_id.append('<gx:value>{0}</gx:value>'.format(path))
            path_throttle.append('<gx:value>{0:.3f}</gx:value>'.format(control_pos[0]))
            path_left_elevon.append('<gx:value>{0:.3f}</gx:value>'.format(control_pos[1]))
            path_right_elevon.append('<gx:value>{0:.3f}</gx:value>'.format(control_pos[2]))
            path_airspeed.append(airspeed)
            path_data.append(time_when + point + attitude)
            path_obj_val.append('<gx:value>{0:.1f}</gx:value>'.format(control_obj_val))

            last_ref_point = ref_point
            last_auto = auto

    # Write KML to disk.
    with open(args.outfile, 'w') as outfile:
        outfile.write('''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"
     xmlns:gx="http://www.google.com/kml/ext/2.2">
    <Document>
        <name>{infile}</name>

        <Schema name="PathData" id="PathData">
            <SimpleField type="string" name="path_id">
                <displayName>Path ID</displayName>
            </SimpleField>
            <SimpleField type="float" name="airspeed">
                <displayName>Airspeed</displayName>
            </SimpleField>
            <SimpleField type="float" name="throttle">
                <displayName>Throttle Position</displayName>
            </SimpleField>
            <SimpleField type="float" name="left_elevon">
                <displayName>Left Elevon Position</displayName>
            </SimpleField>
            <SimpleField type="float" name="right_elevon">
                <displayName>Right Elevon Position</displayName>
            </SimpleField>
            <SimpleField type="float" name="obj_val">
                <displayName>Control Objective Function Value</displayName>
            </SimpleField>
        </Schema>

        <Placemark>
            <name>Reference trajectory</name>
            <Style>
                <LineStyle>
                    <color>99ffffff</color>
                </LineStyle>
            </Style>
            <gx:Track>
                <altitudeMode>relativeToGround</altitudeMode>
                {ref_path}
                <ExtendedData>
                    <SchemaData schemaUrl="#schema">
                        <gx:SimpleArrayData name="airspeed">
                            {ref_airspeed}
                        </gx:SimpleArrayData>
                    </SchemaData>
                </ExtendedData>
            </gx:Track>
        </Placemark>

        <Placemark>
            <name>Flight path</name>
            <Style>
                <LineStyle>
                    <color>ff0000ff</color>
                </LineStyle>
            </Style>
            <gx:Track>
            <altitudeMode>relativeToGround</altitudeMode>
                {path}
                <ExtendedData>
                    <SchemaData schemaUrl="#schema">
                        <gx:SimpleArrayData name="airspeed">
                            {path_airspeed}
                        </gx:SimpleArrayData>
                        <gx:SimpleArrayData name="path_id">
                            {path_id}
                        </gx:SimpleArrayData>
                        <gx:SimpleArrayData name="throttle">
                            {path_throttle}
                        </gx:SimpleArrayData>
                        <gx:SimpleArrayData name="left_elevon">
                            {path_left_elevon}
                        </gx:SimpleArrayData>
                        <gx:SimpleArrayData name="right_elevon">
                            {path_right_elevon}
                        </gx:SimpleArrayData>
                        <gx:SimpleArrayData name="obj_val">
                            {path_obj_val}
                        </gx:SimpleArrayData>
                    </SchemaData>
                </ExtendedData>
            </gx:Track>
        </Placemark>
    </Document>
</kml>'''.format(
    infile=args.infile,
    ref_path="".join(ref_path),
    ref_airspeed="".join(ref_path_airspeed),
    path="".join(path_data),
    path_airspeed="".join(path_airspeed),
    path_id="".join(path_id),
    path_throttle="".join(path_throttle),
    path_left_elevon="".join(path_left_elevon),
    path_right_elevon="".join(path_right_elevon),
    path_obj_val="".join(path_obj_val)
))
