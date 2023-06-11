import uuid
from geographic_msgs.msg import GeographicMap
from geographic_msgs.msg import WayPoint
from geometry_msgs.msg import Point

class GeoMap():
    def __init__(self, gmap):
        self.gmap = gmap

        self.feature_ids = {}
        self.n_features = len(self.gmap.features)
        for fid in range(self.n_features):
            feat = self.gmap.features
            self.feature_ids[str(uuid.UUID(bytes=bytes(feat[fid].id.uuid)))] = fid

    def bounds(self):
        return self.gmap.bounds

    def header(self):
        return self.gmap.header

class GeoMapFeatures():
    def __init__(self, geomap):
        self.gmap = geomap

    def __contains__(self, item):
        return item in self.gmap.feature_ids

    def __getitem__(self, key):
        index = self.gmap.feature_ids[key]
        return self.gmap.features[index]

    def __iter__(self):
        self.iter_index = 0
        return self

    def __len__(self):
        return len(self.gmap.gmap.features)

    def __next__(self):
        i = self.iter_index
        if i >= self.gmap.n_features:
            raise StopIteration
        self.iter_index = i + 1
        return self.gmap.gmap.features[i]
