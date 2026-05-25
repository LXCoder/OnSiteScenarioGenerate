import xml.etree.ElementTree as ET

def get_lane_section_speed_limits(file_path):
    # Load the xodr file
    tree = ET.parse(file_path)
    root = tree.getroot()
    # Initialize the dictionary
    lane_section_speed_limits = {}
    # Iterate over all roads
    for road in root.findall('road'):
        road_id = road.get('id')
        road_length = float(road.get('length', '0'))
        # Build a list of speed limit sections
        speed_sections = []
        for speed_element in road.findall('speed'):
            s_offset = float(speed_element.get('sOffset', '0'))
            max_speed = float(speed_element.get('max', '0'))
            speed_sections.append((s_offset, max_speed))
        # If no speed elements under road, try to get speed from <type><speed> elements
        if not speed_sections:
            for type_element in road.findall('type'):
                s_offset = float(type_element.get('s', '0'))
                speed_element = type_element.find('speed')
                if speed_element is not None:
                    max_speed = float(speed_element.get('max', '0'))
                    speed_sections.append((s_offset, max_speed))
        # Sort the speed_sections by s_offset
        speed_sections.sort()
        # Append an end section to cover until the end of the road
        speed_sections.append((road_length, None))
        # Now we have speed_sections like [(s0, v0), (s1, v1), ..., (sn, None)]
        # Iterate over all laneSections in the road
        lanes = road.find('lanes')
        if lanes is None:
            continue
        for laneSection in lanes.findall('laneSection'):
            lane_section_s = float(laneSection.get('s', '0'))
            # Determine the applicable speed limit for this laneSection
            speed_limit = None
            for i in range(len(speed_sections) - 1):
                s0, v0 = speed_sections[i]
                s1, _ = speed_sections[i + 1]
                if s0 <= lane_section_s < s1:
                    speed_limit = v0/3.6
                    break
            # Build the key in the specified format
            key = f"Road-{road_id}_LaneSection-{int(lane_section_s)}"
            lane_section_speed_limits[key] = speed_limit
    return lane_section_speed_limits



