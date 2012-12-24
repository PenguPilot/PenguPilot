#!/usr/bin/env python
"""
  ___________________________________________________
 |  _____                       _____ _ _       _    |
 | |  __ \                     |  __ (_) |     | |   |
 | | |__) |__ _ __   __ _ _   _| |__) || | ___ | |_  |
 | |  ___/ _ \ '_ \ / _` | | | |  ___/ | |/ _ \| __| |
 | | |  |  __/ | | | (_| | |_| | |   | | | (_) | |_  |
 | |_|   \___|_| |_|\__, |\__,_|_|   |_|_|\___/ \__| |
 |                   __/ |                           |
 |  GNU/Linux based |___/  Multi-Rotor UAV Autopilot |
 |___________________________________________________|
  
 File Purpose

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """

# ----------------------------------------
# Copyright (C) 2008-2011  Frank Paehlke
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ----------------------------------------

import codecs
import math
import sys
import time
from os.path import basename
from xml.dom.minidom import parse, getDOMImplementation
import dateutil.parser

# Python 3.x compatibility
if sys.version_info[0] >= 3:
    basestring = str

# ----------------------------------------
# Utility functions
# ----------------------------------------

class Util:
    """
    Several static utility functions for geodesic calculations and map projections.

    These functions do not aim at maximum accuracy - the Geoid is modeled
    as a perfect sphere with radius 6370 km.
    """

    r_earth = 6.370e6 # earth radius [m]

    @staticmethod
    def haversin (x):
        """
        haversin(x) = sin^2 (x/2)
        """
        sinx2 = math.sin(x/2.0)
        return sinx2*sinx2

    @staticmethod
    def inv_haversin (x):
        """
        inverse of haversin(x)
        """
        return 2.0 * math.asin(math.sqrt(x))

    @classmethod
    def angular_distance (cls, lat1, lon1, lat2, lon2):
        """
        angular distance [radians] between two geographical coordinates
        """
        _lat1 = math.radians(lat1)
        _lat2 = math.radians(lat2)
        londiff = math.radians(lon2-lon1)
        return cls.inv_haversin (
            cls.haversin(_lat2 - _lat1) +
            math.cos(_lat1) * math.cos(_lat2) * cls.haversin(londiff))

    @classmethod
    def geo_distance (cls, lat1, lon1, lat2, lon2):
        """
        distance [m] between two geographical coordinates
        """
        return cls.r_earth * cls.angular_distance(lat1, lon1, lat2, lon2)

    @classmethod
    def distance (cls, p1, p2):
        """
        distance between two objects of class "Point"
        """
        return cls.r_earth * cls.angular_distance(p1.lat, p1.lon, p2.lat, p2.lon)

    @classmethod
    def transMercator (cls, lat, lon, centerlon=0.0):
        """
        Transversal Mercator projection - returns (x,y) tuple (unit:  m)
        """
        _lat = math.radians(lat)
        _lon = math.radians(lon-centerlon)
        x = cls.r_earth * math.atanh(math.sin(_lon) * math.cos(_lat))
        y = cls.r_earth * math.atan(math.tan(_lat) / math.cos(_lon))
        return x,y

# ----------------------------------------
# Representation of a GPX document
# ----------------------------------------
class Document:
    """
    Representation of a GPX document
    """

    def __init__ (self, tracks=[], waypoints=[], name="(unnamed)"):
        self.tracks = tracks
        self.waypoints = waypoints
        self.name = name

    def __str__ (self):
        return '<Document "%s" (%d tracks, %d waypoints)>' % (self.name, len(self.tracks), len(self.waypoints))

    def toGPX (self, domImpl=getDOMImplementation()):
        """
        convert to GPX DOM
        """
        doc = domImpl.createDocument ("http://www.topografix.com/GPX/1/1","gpx",None)
        gpx = doc.documentElement
        gpx.setAttribute ("xmlns","http://www.topografix.com/GPX/1/1")
        gpx.setAttribute ("creator", "")
        gpx.setAttribute ("version", "1.1")
        gpx.setAttribute ("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
        gpx.setAttribute ("xsi:schemaLocation", "http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd")
        for trk in self.tracks:
            gpx.appendChild (trk.toGPX(doc))
        for wpt in self.waypoints:
            gpx.appendChild (wpt.toGPX(doc))
        return doc

    def writeGPX (self, file, encoding='utf-8'):
        """
        write GPX file
        """
        if isinstance (file, basestring):
            self.writeGPX (codecs.open (file, 'w', encoding), encoding)
            return
        gpx = self.toGPX (getDOMImplementation())
        gpx.writexml (codecs.getwriter (encoding)(file), addindent=' ', newl='\n', encoding=encoding)

    @staticmethod
    def fromGPX (dom, name='(unnamed)'):
        """
        parse GPX DOM
        """
        tracks = []
        for trk in dom.getElementsByTagName("trk"):
            tracks.append (Track.fromGPX (trk))
        waypoints = []
        for wpt in dom.getElementsByTagName("wpt"):
            waypoints.append (Waypoint.fromGPX (wpt))
        return Document (tracks, waypoints, name)

    @staticmethod
    def readGPX (file, name='(unnamed)'):
        """
        read GPX file
        """
        dom = parse(file)
        return Document.fromGPX (dom, name)

    # create KML document
    def toKML (self, domImpl):
        """
        convert to KML DOM
        """
        doc = domImpl.createDocument ("http://earth.google.com/kml/2.0","kml",None)
        kml = doc.documentElement
        kml.setAttribute ("xmlns","http://earth.google.com/kml/2.0")

        # create Document tag
        document = doc.createElement("Document")
        kml.appendChild(document)
        document.setAttribute ("xmlns","http://earth.google.com/kml/2.0")
        e = doc.createElement("name")
        e.appendChild (doc.createTextNode(self.name))
        document.appendChild (e)
        e = doc.createElement ("visibility")
        e.appendChild (doc.createTextNode("1"))
        document.appendChild (e)

        # create Tracks folder
        trkfolder = doc.createElement ("Folder")
        document.appendChild (trkfolder)
        e = doc.createElement ("name")
        e.appendChild (doc.createTextNode("Tracks"))
        trkfolder.appendChild (e)

        # append tracks
        for trk in self.tracks:
            trkfolder.appendChild (trk.toKML(doc))

        # create Waypoints folder
        wptfolder = doc.createElement ("Folder")
        document.appendChild (wptfolder)
        e = doc.createElement ("name")
        e.appendChild (doc.createTextNode("Waypoints"))
        wptfolder.appendChild (e)

        # append waypoints
        for wpt in self.waypoints:
            wptfolder.appendChild (wpt.toKML(doc))

        return doc

    def writeKML (self, file, encoding='utf-8'):
        """
        write KML file
        """
        if isinstance (file, basestring):
            self.writeKML (codecs.open (file, 'w', encoding), encoding)
            return
        kml = self.toKML (getDOMImplementation())
        kml.writexml (codecs.getwriter (encoding)(file), addindent=' ', newl='\n', encoding=encoding)

    @staticmethod
    def fromKML (dom):
        """
        parse KML DOM
        """
        doc = dom.getElementsByTagName('Document')[0]
        name = '(unnamed)'
        for e in doc.getElementsByTagName('name'):
            name = e.childNodes[0].data
        tracks = []
        waypoints = []
        for placemark in doc.getElementsByTagName("Placemark"):
            if len (placemark.getElementsByTagName ("LineString")) > 0:
                tracks.append (Track.fromKML (placemark))
            elif len (placemark.getElementsByTagName ("Point")) > 0:
                waypoints.append (Waypoint.fromKML (placemark))
        return Document (tracks, waypoints, name)

    @staticmethod
    def readKML (file):
        """
        read KML file
        """
        dom = parse(file)
        return Document.fromKML (dom)

    def toOVL (self, ostream):
        """
        write OVL to output stream
        """
        groupnr = 0
        symbolnr = 0
        for track in self.tracks:
            groupnr = groupnr+1
            segnr = 0
            bmin, bmax = 100.0, -100.0
            lmin, lmax = 200.0, -200.0
            for trkseg in track.segments:
                symbolnr = symbolnr+1
                ostream.write (
                    "[Symbol %d]\r\n"
                    "Typ=3\r\n"
                    "Group=%d\r\n"
                    "Col=1\r\n"
                    "Zoom=1\r\n"
                    "Size=103\r\n"
                    "Art=1\r\n"
                    % (symbolnr, groupnr))
                punkte = trkseg.points
                ostream.write ("Punkte=%d\r\n" % len(punkte))
                punktnr = 0
                for trkpt in punkte:
                    b = float(trkpt.lat)
                    l = float(trkpt.lon)
                    bmin, bmax = min (bmin, b), max (bmax, b)
                    lmin, lmax = min (lmin, l), max (lmax, l)
                    ostream.write ("XKoord%d=%.6f\r\n" % (punktnr, l))
                    ostream.write ("YKoord%d=%.6f\r\n" % (punktnr, b))
                    punktnr = punktnr+1
        groupnr = groupnr+1
        for waypoint in self.waypoints:
            symbolnr = symbolnr+1
            ostream.write (
                "[Symbol %d]\r\n"
                "Typ=6\r\n"
                "Group=%d\r\n"
                "Width=10\r\n"
                "Height=10\r\n"
                "Dir=100\r\n"
                "Col=1\r\n"
                "Zoom=1\r\n"
                "Size=103\r\n"
                "Area=2\r\n"
                "XKoord=%.6f\r\n"
                "YKoord=%.6f\r\n"
                % (symbolnr, groupnr, waypoint.lon, waypoint.lat))
        ostream.write (
            "[Overlay]\r\n"
            "Symbols=%d\r\n"
            "[MapLage]\r\n"
            "MapName=Top. Karte 1:50000 Bw\r\n"
            "DimmFc=100\r\n"
            "ZoomFc=100\r\n"
            "CenterLat=%.6f\r\n"
            "CenterLong=%.6f\r\n"
            "RefOn=0\r\n"
            % (symbolnr, (bmin+bmax)/2, (lmin+lmax)/2))

    def writeOVL (self, file, encoding='iso-8859-1'):
        """
        write OVL file
        """
        if isinstance (file, basestring):
            self.toOVL (codecs.open (file, 'w', encoding), encoding)
        else:
            self.toOVL (codecs.getwriter (encoding)(file))

    @staticmethod
    def fromOVL (istream, name="(unnamed)"):
        """
        parse OVL from input stream
        """
        tracks = {} # Map Group -> Track

        mode = "start"
        for line in istream:
            if line.startswith("["):
                if mode == "symbol":
                    # Group schon vorhanden?
                    if symbolGroup in tracks:
                        track = tracks[symbolGroup]
                    else:
                        track = Track ("%s/%d" % (name, symbolGroup))
                        tracks[symbolGroup] = track
                    # Symbol schreiben
                    trkseg = TrackSegment()
                    for lon, lat in koord:
                        trkpt = TrackPoint (lat, lon)
                        trkseg.appendPoint (trkpt)
                    track.appendSegment (trkseg)
                if line.startswith("[Symbol"):
                    symbolTyp = None
                    symbolGroup = None
                    koord = []
                    mode = "symbol"
                else:
                    mode = "other"
            elif line.startswith ("Group"):
                symbolGroup = int(line.strip().split("=")[1])
            elif line.startswith ("XKoord"):
                x = float(line.strip().split("=")[1])
            elif line.startswith ("YKoord"):
                y = float(line.strip().split("=")[1])
                koord.append ((x, y))

        groups = sorted(tracks.keys())
        return Document([tracks[group] for group in groups], name=name)

    @staticmethod
    def readOVL (file, name='(unnamed)', encoding='iso-8859-1'):
        """
        read OVL file
        """
        if isinstance (file, basestring):
            return Document.fromOVL (codecs.open (file, 'r', encoding), basename(file))
        else:
            return Document.fromOVL (codecs.getreader (encoding)(file), basename(file.name))

# ----------------------------------------
# Representation of a GPX track
# ----------------------------------------
class Track:
    """
    Representation of a GPX track

    Constructor:
    Track(name=None, description=None, segments=[])    
    """

    def __init__ (self, name=None, description=None, segments=[]):
        self.name = name
        self.description = description
        self.segments = segments

    def __str__ (self):
        return '<Track "%s" (%d segments)>' % (self.name, len(self.segments))

    def appendSegment (self, trkseg):
        self.segments.append(trkseg)

    def appendTrack (self, trk):
        self.segments += trk.segments

    def length (self):
        """
        total length of all segments [m]
        """
        l = 0.0
        for s in self.segments:
            l += s.length()
        return l

    def boundingBox (self):
        """
        return minimum/maximum coordinates of this track
        as tuple (minLat, minLon, maxLat, maxLon)
        """
        minLat = minLon = maxLat = maxLon = None
        first = True
        for segment in self.segments:
            try:
                s_minLat, s_minLon, s_maxLat, s_maxLon = segment.boundingBox()
            except ValueError: # segment is empty
                continue
            if first:
                minLat, minLon, maxLat, maxLon = s_minLat, s_minLon, s_maxLat, s_maxLon
                first = False
            else:
                minLat = min(minLat, s_minLat)
                minLon = min(minLon, s_minLon)
                maxLat = max(maxLat, s_maxLat)
                maxLon = max(maxLon, s_maxLon)
        if first:
            raise ValueError("empty track has no bounding box")
        return minLat, minLon, maxLat, maxLon

    def transMercator (self, centerlon=0.0):
        """
        Transversal Mercator projection - returns list of lists of (x,y) tuples (unit: m)

        Each sub-list corresponds to a track segment.
        """
        return [ s.transMercator(centerlon) for s in self.segments ]

    def toGPX (self, doc):
        """
        convert to GPX Element <trk>
        """
        res = doc.createElement("trk")
        if self.name != None:
            e = doc.createElement("name")
            e.appendChild (doc.createTextNode(self.name))
            res.appendChild (e)
        if self.description != None:
            e = doc.createElement("desc")
            e.appendChild (doc.createTextNode(self.description))
            res.appendChild (e)
        for p in self.segments:
            res.appendChild (p.toGPX (doc))
        return res

    @staticmethod
    def fromGPX (trk):
        "parse GPX Element <trk>"
        name = None
        for e in trk.getElementsByTagName("name"):
            name = e.childNodes[0].data
        description = None
        for e in trk.getElementsByTagName("desc"):
            description = e.childNodes[0].data
        res = Track (name, description, [])
        for trkseg in trk.getElementsByTagName("trkseg"):
            res.appendSegment (TrackSegment.fromGPX (trkseg))
        return res

    def toKML (self, doc):
        """
        convert to KML Element <Placemark>
        """
        res = doc.createElement("Placemark")
        if self.name != None:
            e = doc.createElement("name")
            e.appendChild (doc.createTextNode(self.name))
            res.appendChild (e)
        if self.description != None:
            e = doc.createElement("description")
            e.appendChild (doc.createTextNode(self.description))
            res.appendChild (e)
        for p in self.segments:
            res.appendChild (p.toKML (doc))
        return res

    @staticmethod
    def fromKML (placemark):
        """
        parse KML Element <Placemark>
        """
        name = "(unnamed)"
        for e in placemark.getElementsByTagName ("name"):
            name = e.childNodes[0].data.strip()
        track = Track(name)
        for e in placemark.getElementsByTagName ("LineString"):
            track.appendSegment (TrackSegment.fromKML (e))
        return track

# ----------------------------------------
# Representation of a GPX track segment
# ----------------------------------------
class TrackSegment:
    """
    Representation of a GPX track segment

    Constructor:
    TrackSegment(points=[])
    """

    def __init__ (self, points=[]):
        self.points = points

    def __str__ (self):
        return '<TrackSegment (%d points)>' % len(self.points)

    def __repr__ (self):
        return 'TrackSegment(' + repr(self.points) + ')'

    def __getitem__ (self, i):
        return self.points[i]

    def appendPoint (self, point):
        self.points.append(point)

    def appendSegment (self, trkseg):
        self.points += trkseg.points

    def __iadd__ (self, trkseg):
        self.points += trkseg.points

    def __add_ (self, trkseg):
        return TrackSegment(self.points + trkseg.points)

    def length (self):
        """
        length of track segment [m]
        """
        l = 0.0
        for i in range(len(self.points)-1):
            l += Util.distance (self.points[i], self.points[i+1])
        return l

    def boundingBox (self):
        """
        return minimum/maximum coordinates of this track segment
        as tuple (minLat, minLon, maxLat, maxLon)
        """
        if len(self.points) == 0:
            raise ValueError("empty track segment has no bounding box")
        minLat = maxLat = self.points[0].lat
        minLon = maxLon = self.points[0].lon
        for p in self.points[1:]:
            minLat = min(minLat, p.lat)
            minLon = min(minLon, p.lon)
            maxLat = max(maxLat, p.lat)
            maxLon = max(maxLon, p.lon)
        return minLat, minLon, maxLat, maxLon

    def transMercator (self, centerlon=0.0):
        """
        Transversal Mercator projection - returns list of (x,y) tuples (unit: m)
        """
        return [ p.transMercator(centerlon) for p in self.points ]

    def toGPX (self, doc):
        """
        convert to GPX Element <trkseg>
        """
        res = doc.createElement("trkseg")
        for p in self.points:
            res.appendChild (p.toGPX (doc))
        return res

    @staticmethod
    def fromGPX (trkseg):
        """
        parse GPX Element <trkseg>
        """
        res = TrackSegment()
        for trkpt in trkseg.getElementsByTagName("trkpt"):
            res.appendPoint (TrackPoint.fromGPX (trkpt))
        return res

    def toKML (self, doc):
        """
        convert to KML Element <LineString>
        """
        res = doc.createElement ("LineString")
        coord = doc.createElement ("coordinates")
        coordStr = " ".join([p.toKML(doc) for p in self.points])
        coord.appendChild (doc.createTextNode (coordStr))
        res.appendChild (coord)
        return res

    @staticmethod
    def fromKML (linestring):
        """
        parse KML Element <LineString>
        """
        trkseg = TrackSegment()
        coords = linestring.getElementsByTagName("coordinates")[0].childNodes[0].data
        for c in coords.split ():
            lon, lat, ele = c.split (",")
            lon = float(lon)
            lat = float(lat)
            if ele == "0":
                ele = None
            else:
                ele = float(ele)
            trkseg.appendPoint (TrackPoint (lon, lat, ele))
        return trkseg

    def simplify (self, delta=1.0):
        """
        Reduce number of track points while preserving the shape of the track segment as much as possible.

        The parameter delta gives the minimum distance [m] that a track point should have
        from the straight line connecting its predecessor and successor.
        """
        n = len(self.points)
        if n <= 2:
            # there's nothing to simplify
            return
        idx = [0] + self._simplify (0, n-1, delta) + [n-1]
        self.points = [ self.points[i] for i in idx ]

    def _simplify (self, start, end, delta):
        # Reduce number of intermediate points between points[start] and points[end]
        if end-start <= 1:
            return [] # there are no intermediate points
        # calculate the point with maximum distance
        # from the straight line between points[start] and points[end]
        max_dist =  0.0
        max_index = None
        for k in range(start+1, end):
            d = self.points[k].distance (self.points[start], self.points[end])
            if d > max_dist:
                max_dist = d
                max_index = k
        if max_dist < delta:
            return [] # discard all intermediate points
        # recursion
        return (
            self._simplify(start, max_index, delta)
            + [max_index]
            + self._simplify(max_index, end, delta))

# ----------------------------------------
# Common base class for track points and waypoints
# ----------------------------------------
class Point:
    """
    Common base class for track points (class TrackPoint) and waypoints (class Waypoint)

    Constructor: Point(lat, lon, ele=None, t=None)
    """

    def __init__ (self, lat, lon, ele=None, t=None):
        self.lat = lat
        self.lon = lon
        self.ele = ele
        self.t = t

    def __str__ (self):
        s = [ self.__class__.__name__, '(', str(self.lat), ',', str(self.lon) ]
        if self.ele != None:
            s += [ ',', str(self.ele) ]
        if self.t != None:
            s += [ ',', str(self.t) ]
        s.append(')')
        return ''.join(s)

    def __repr__ (self):
        return str(self)

    def __eq__ (self, p):
        return (
            self.__class__ == p.__class__
            and self.lat == p.lat and self.lon == p.lon
            and self.ele == p.ele and self.t == p.t)

    def __ne__ (self, p):
        return not (self == p)

    def __hash__ (self):
        return hash((self.lat, self.lon, self.ele, self.t))

    def distance (self, p, q=None):
        """
        calculate geographic distance [m]

        distance(p):
            distance between self and p

        distance(p,q):
            distance between self and the great circle that connects p and q
        """
        if q == None:
            return Util.geo_distance(self.lat, self.lon, p.lat, p.lon)
        else:
            # calculate the angle between (p, self) and (p, q)
            # first, calculate the side lengths of the spherical triangle (self, p, q)
            a = Util.angular_distance(self.lat, self.lon, q.lat, q.lon)
            b = Util.angular_distance(q.lat, q.lon, p.lat, p.lon)
            c = Util.angular_distance(p.lat, p.lon, self.lat, self.lon)
            if a == 0.0 or c == 0.0:
                # self equals p or q
                return 0.0
            elif b == 0.0:
                raise ValueError("p and q are equal and do not define a great circle")
            # now we can calculate the angle using the half-angle formula for sin(alpha/2)
            alpha = Util.inv_haversin (
                math.sin((a+c-b)/2) * math.sin((a+b-c)/2)
                / math.sin(b) / math.sin(c))
            # now we can calculate the distance as the height of the rectangular spherical triangle
            # with hypotenuse (p, self)
            d = Util.r_earth * math.asin(math.sin(c) * math.sin(alpha))
            return d

    def transMercator (self, centerlon=0.0):
        """
        transversal Mercator projection - returns (x,y) tuple (unit: m)
        """
        return Util.transMercator(self.lat, self.lon, centerlon)

# ----------------------------------------
# Representation of a GPX track point
# ----------------------------------------
class TrackPoint(Point):
    """
    Representation of a GPX track point

    Constructor: TrackPoint(lat, lon, ele=None, t=None)
    """

    def toGPX (self, doc):
        """
        convert to GPX Element <trkpt>
        """
        res = doc.createElement("trkpt")
        res.setAttribute ("lat", ("%.6f" % self.lat))
        res.setAttribute ("lon", ("%.6f" % self.lon))
        if self.ele != None:
            e = doc.createElement("ele")
            e.appendChild (doc.createTextNode(str(self.ele)))
            res.appendChild (e)
        if self.t != None:
            e = doc.createElement("time")
            t = self.t.strftime ("%Y-%m-%dT%H:%M:%SZ")
            e.appendChild (doc.createTextNode(t))
            res.appendChild (e)
        return res

    @staticmethod
    def fromPoint (p):
        """
        convert any Point object to a TrackPoint
        """
        return TrackPoint (p.lat, p.lon, p.ele, p.t)

    @staticmethod
    def fromGPX (trkpt):
        """
        parse GPX Element <trkpt>
        """
        lat = float(trkpt.getAttribute("lat"))
        lon = float(trkpt.getAttribute("lon"))
        ele = None
        for e in trkpt.getElementsByTagName("ele"):
            ele = float(e.childNodes[0].data)
        t = None
        for e in trkpt.getElementsByTagName("time"):
            t = dateutil.parser.parse (e.childNodes[0].data)
        return TrackPoint (lat, lon, ele, t)

    def toKML (self, doc):
        """
        convert to KML-formatted coordinate string
        """
        if self.ele is None:
            return "%f,%f,0" % (self.lon, self.lat)
        else:
            return "%f,%f,%.3f" % (self.lon, self.lat, self.ele)

# ----------------------------------------
# Representation of a GPX waypoint
# ----------------------------------------
class Waypoint(Point):
    """
    Representation of a GPX waypoint

    Constructor: Waypoint(name, lat, lon, ele=None, t=None)
    """

    def __init__ (self, name, lat, lon, ele=None, t=None):
        Point.__init__(self, lat, lon, ele, t)
        self.name = name

    def __str__ (self):
        s = [ self.__class__.__name__, '(', self.name, ',', str(self.lat), ',', str(self.lon) ]
        if self.ele != None:
            s += [ ',', str(self.ele) ]
        if self.t != None:
            s += [ ',', str(self.t) ]
        s.append(')')
        return ''.join(s)

    def __repr__ (self):
        return str(self)

    def __eq__ (self, p):
        return (
            self.__class__ == p.__class__
            and self.lat == p.lat and self.lon == p.lon
            and self.ele == p.ele and self.t == p.t)

    def __ne__ (self, p):
        return not (self == p)

    def __hash__ (self):
        return hash((self.name, self.lat, self.lon, self.ele, self.t))

    @staticmethod
    def fromPoint (p, name):
        """
        convert any Point object to a Waypoint
        """
        return Waypoint (name, p.lat, p.lon, p.ele, p.t)

    def toGPX (self, doc):
        """
        convert to GPX Element <wpt>
        """
        res = doc.createElement("wpt")
        res.setAttribute ("lat", ("%.6f" % self.lat))
        res.setAttribute ("lon", ("%.6f" % self.lon))
        if self.name != None:
            e = doc.createElement("name")
            e.appendChild (doc.createTextNode(self.name))
            res.appendChild (e)
        if self.ele != None:
            e = doc.createElement("ele")
            e.appendChild (doc.createTextNode(str(self.ele)))
            res.appendChild (e)
        if self.t != None:
            e = doc.createElement("time")
            t = self.t.strftime ("%Y-%m-%dT%H:%M:%SZ")
            e.appendChild (doc.createTextNode(t))
            res.appendChild (e)
        return res

    def toKML (self, doc):
        """
        convert to KML Element <Placemark>
        """
        res = doc.createElement ("Placemark")
        if self.name != None:
            e = doc.createElement ("name")
            e.appendChild (doc.createTextNode (self.name))
            res.appendChild (e)
        if self.t != None:
            ts = doc.createElement ("TimeStamp")
            e = doc.createElement ("when")
            e.appendChild (doc.createTextNode (self.t.strftime ("%Y-%m-%dT%H:%M:%SZ")))
            ts.appendChild (e)
            res.appendChild (ts)
        p = doc.createElement ("Point")
        c = doc.createElement ("coordinates")
        if self.ele is None:
            coords = "%f,%f,0" % (self.lon, self.lat)
        else:
            coords = "%f,%f,%.3f" % (self.lon, self.lat, self.ele)
        c.appendChild (doc.createTextNode (coords))
        p.appendChild (c)
        res.appendChild (p)
        return res

    @staticmethod
    def fromGPX (wpt):
        """
        parse GPX Element <wpt>
        """
        lat = float(wpt.getAttribute("lat"))
        lon = float(wpt.getAttribute("lon"))
        name = None
        for e in wpt.getElementsByTagName("name"):
            name = e.childNodes[0].data
        ele = None
        for e in wpt.getElementsByTagName("ele"):
            ele = float(e.childNodes[0].data)
        t = None
        for e in wpt.getElementsByTagName("time"):
            t = dateutil.parser.parse (e.childNodes[0].data)
        return Waypoint (name, lat, lon, ele, t)

    @staticmethod
    def fromKML (placemark):
        """
        parse KML Element <Placemark>
        """
        name = "(unnamed)"
        for e in placemark.getElementsByTagName ("name"):
            name = e.childNodes[0].data.strip()
        for p in placemark.getElementsByTagName ("Point"):
            coords = p.getElementsByTagName("coordinates")[0].childNodes[0].data
            lon, lat, ele = coords.split (",")
            lon = float(lon)
            lat = float(lat)
            if ele == "0":
                ele = None
            else:
                ele = float(ele)
        t = None
        for ts in placemark.getElementsByTagName ("TimeStamp"):
            for e in ts.getElementsByTagName ("when"):
                t = dateutil.parser.parse (e.childNodes[0].data)
        return Waypoint (name, lat, lon, ele, t)
