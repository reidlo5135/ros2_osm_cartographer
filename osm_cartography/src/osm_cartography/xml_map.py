import os
import uuid

from xml.etree import ElementTree

from geodesy import bounding_box

from geographic_msgs.msg import GeographicMap
from geographic_msgs.msg import KeyValue
from geographic_msgs.msg import MapFeature
from geographic_msgs.msg import WayPoint
from unique_identifier_msgs.msg import UUID

PACKAGE_NAME = "osm_cartography"
SHARE_DIR = os.path.join("share", PACKAGE_NAME)


def get_required_attribute(el, key):
    val = el.get(key)
    if val is None:
        raise ValueError('required attribute missing: ' + key)
    return val

def make_osm_unique_id(namespace, el_id):
    if namespace not in {'node', 'way', 'relation'}:
        raise ValueError('invalid OSM namespace: ' + namespace)
    ns = 'http://openstreetmap.org/' + namespace + '/'
    return UUID(uuid=list(uuid.uuid5(uuid.NAMESPACE_URL, ns + str(el_id)).bytes))

def get_tag(el):
    pair = None
    key = el.get('k')
    if key is not None:
        pair = KeyValue()
        pair.key = key
        pair.value = get_required_attribute(el, 'v')
        return pair

def get_osm(url, bounds):
    if url.startswith('file:///'):
        filename = url[7:]
        print("[INFO] xml_map get_osm file name from file : ", filename)
    elif url.startswith('package://'):
        pkg_name, slash, pkg_path = url[10:].partition('/')
        filename = SHARE_DIR + '/' + pkg_path
        print("[INFO] xm_map get_osm file name from package : ", filename)
    else:
        raise ValueError('unsupported URL: ' + url)

    gmap = GeographicMap(id=UUID(uuid=list(uuid.uuid5(uuid.NAMESPACE_URL, url).bytes)))

    xm = None
    try:
        with open(filename, 'r') as f:
            xm = ElementTree.parse(f)
    except IOError:
        raise ValueError('unable to read ' + str(url))
    except ElementTree.ParseError:
        raise ValueError('XML parse failed for ' + str(url))
    osm = xm.getroot()

    for el in osm.iterfind('bounds'):
        minlat = float(get_required_attribute(el, 'minlat'))
        minlon = float(get_required_attribute(el, 'minlon'))
        maxlat = float(get_required_attribute(el, 'maxlat'))
        maxlon = float(get_required_attribute(el, 'maxlon'))
        gmap.bounds = bounding_box.makeBounds2D(minlat, minlon, maxlat, maxlon)

    for el in osm.iterfind('node'):

        way = WayPoint()
        el_id = el.get('id')
        if el_id is None:
            raise ValueError('node id missing')
        way.id = make_osm_unique_id('node', el_id)

        way.position.latitude = float(get_required_attribute(el, 'lat'))
        way.position.longitude = float(get_required_attribute(el, 'lon'))
        way.position.altitude = float(el.get('ele', float('nan')))

        for tag_list in el.iterfind('tag'):
            kv = get_tag(tag_list)
            if kv is not None:
                way.props.append(kv)

        gmap.points.append(way)

    for el in osm.iterfind('way'):
        feature = MapFeature()
        el_id = el.get('id')
        if el_id is None:
            raise ValueError('way id missing')
        feature.id = make_osm_unique_id('way', el_id)

        for nd in el.iterfind('nd'):
            way_id = get_required_attribute(nd, 'ref')
            feature.components.append(make_osm_unique_id('node', way_id))

        for tag_list in el.iterfind('tag'):
            kv = get_tag(tag_list)
            if kv is not None:
                feature.props.append(kv)

        gmap.features.append(feature)

    for el in osm.iterfind('relation'):

        feature = MapFeature()
        el_id = el.get('id')
        if el_id is None:
            raise ValueError('relation id missing')
        feature.id = make_osm_unique_id('relation', el_id)

        for mbr in el.iterfind('member'):
            mbr_type = get_required_attribute(mbr, 'type')
            if mbr_type in {'node', 'way', 'relation'}:
                mbr_id = get_required_attribute(mbr, 'ref')
                feature.components.append(make_osm_unique_id(mbr_type, mbr_id))
            else:
                print('unknown relation member type: ' + mbr_type)

        for tag_list in el.iterfind('tag'):
            kv = get_tag(tag_list)
            if kv is not None:
                feature.props.append(kv)

        gmap.features.append(feature)

    return gmap


interesting_tags = {'access',
                    'amenity',
                    'boundary',
                    'bridge',
                    'building',
                    'ele',
                    'highway',
                    'landuse',
                    'lanes',
                    'layer',
                    'maxheight',
                    'maxspeed',
                    'maxwidth',
                    'name',
                    'network',
                    'oneway',
                    'railway',
                    'ref',
                    'restriction',
                    'route',
                    'street',
                    'tunnel',
                    'type',
                    'width'}

ignored_values = {'bridleway',
                  'construction',
                  'cycleway',
                  'footway',
                  'path',
                  'pedestrian',
                  'proposed',
                  'steps'}