# -*- coding: utf-8 -*-

import numpy


def encode_road_section_lane_width_id(roadId, sectionId, laneId, widthId):
    """

    Args:
      roadId:
      sectionId:
      laneId:
      widthId:

    Returns:

    """
    return ".".join([str(roadId), str(sectionId), str(laneId), str(widthId)])


def encode_road_section_lane_width_type_id(roadId, sectionId, laneId, widthId, typeId):
    """

    Args:
      roadId:
      sectionId:
      laneId:
      widthId:
      typeId: 0 - road, 1 - junction

    Returns:

    """
    return ".".join([str(roadId), str(sectionId), str(laneId), str(widthId), str(typeId)])


def decode_road_section_lane_width_id(encodedString: str):
    """

    Args:
      encodedString:

    Returns:

    """

    parts = encodedString.split(".")

    if len(parts) != 4:
        raise Exception()

    return (int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3]))


def decode_road_section_lane_width_type_id(encodedString: str):
    """

    Args:
      encodedString:

    Returns:

    """

    parts = encodedString.split(".")

    if len(parts) != 5:
        raise Exception()

    return (int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]))


def allCloseToZero(array):
    """Tests if all elements of array are close to zero.

    Args:
      array:

    Returns:

    """

    return numpy.allclose(array, numpy.zeros(numpy.shape(array)))