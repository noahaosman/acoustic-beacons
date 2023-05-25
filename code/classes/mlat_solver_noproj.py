#!/usr/bin/env python3

import numpy as np
from scipy.optimize import minimize
import logging

# use these lines for older than python 3.9
root_logger = logging.getLogger()
root_logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('/home/pi/nav/mlat.log', 'w', 'utf-8')
root_logger.addHandler(handler)

# the line below requires python 3.9 or above
# logging.basicConfig(filename='/home/pi/nav/mlat.log', encoding='utf-8', level=logging.DEBUG)

class Mlat:
    "True-range multilateration solver"

    def __init__(self, config):
        # If we're in a local coordinate system, i.e. passive beacon locations
        # are static and specified in meters, then act as if we're in an
        # azimuthal equidistant projection centered at 0'0"N 0'0"E. We can then
        # convert prescribed locations (in meters) to degrees lat/lon and
        # proceed as if we were using GPS input for positions.
        # if config['settings']['coords'] == 'local':
        #     self.lat0 = 0
        #     self.lon0 = 0
        # Otherwise, if we're in lat/lon mode, use the defined origin as
        # the center of our local coordinate reference.
        # elif config['settings']['coords'] == 'latlon':
        #    self.lat0 = config['settings']['lat0']
        #    self.lon0 = config['settings']['lon0']

        # jasmine's edit - use lat0 and lon0 regardless of coord system
        self.lat0 = config['settings']['lat0']
        self.lon0 = config['settings']['lon0']

    def gps2local(self, lat, lon):
        """ Converts gps coordinates to local coordinates.
        Assumes a flat earth."""

        earthRadius = 6371000  # meters

        y = 2*np.pi*earthRadius/360 * (lat - self.lat0)
        x = 2*np.pi*earthRadius/360 * np.cos(np.pi/180 * self.lat0) * (lon - self.lon0)

        return x, y

    def local2gps(self, x, y):
        """ Converts local coordinates in meters to gps coordinates.
        Assumes a flat earth."""

        earthRadius = 6371000  # meters

        lat = y * 360 / (2*np.pi*earthRadius) + self.lat0
        lon = ((x * 360 / (2*np.pi*earthRadius)) / np.cos(np.pi/180 * self.lat0)) + self.lon0

        return lat, lon

    def obj_fun(self, x, P, D):
        # The estimated position is the point x which
        # minimizes the difference between:
        # - Measured distance from all passive beacons
        # - Computed distance between x and all passive beacons
        # Compute the RMS of this difference over all beacons
        dists = np.linalg.norm(P-x, axis=1)
        return np.sqrt(np.mean(np.square(dists-D)))

    def solve(self, locs, dists, x0=None):
        """Estimate a position given a list of passive beacon
        locations and distances"""

        try:

            # Convert the lat, lons of all passive beacons to a matrix
            # [x1, y1, z1;
            #  x2, y2, z2;
            #    ...     ]
            P = np.array(
                [self.gps2local(locs[m]['lat'], locs[m]['lon']) + (locs[m]['z'],)
                 for m in locs.keys()])

            # Convert distances to a matrix
            D = np.array([dists[m]['range'] for m in dists.keys()])

            # Initial guess: average of passive beacon locations if none given.
            if x0 is None:
                x0 = np.mean(P, axis=0)

        except Exception as e:
            logging.debug(e)

        logging.debug(P)
        logging.debug(D)
        logging.debug(x0)

        try:
            # z must be negative (below sea-level)
            z_guess = x0[2]
            z_low = z_guess - 10
            z_high = min(z_guess + 10, 0)
            bounds = [(None, None), (None, None), (z_low, z_high)]
        except Exception as e:
            logging.debug(e)

        logging.debug(bounds)

        # Estimate position by numerically minimizing the objective function
        minout = minimize(self.obj_fun, x0, args=(P, D), method='TNC',
                          bounds=bounds, options={'eps': 1e-1, 'ftol': 1e-4})
        x = minout.x
        logging.debug(x)
        # Convert estimate to lat,lon
        lat, lon = self.local2gps(x[0], x[1])
        # Return coordinates lat,lon,z
        return lat, lon, x[2]
