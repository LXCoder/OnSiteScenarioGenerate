from collections import namedtuple

RefPoint = namedtuple('RefPoint', ["rx", "ry", "rs", "rtheta", "rkappa", "rdkappa"])
TrajPoint = namedtuple('TrajPoint', ["x", "y", "v", "a", "theta", "kappa"])