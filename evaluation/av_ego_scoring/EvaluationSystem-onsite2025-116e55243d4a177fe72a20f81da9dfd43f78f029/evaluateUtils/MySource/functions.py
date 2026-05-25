import math

def calculate_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance

def find_minimum_distance(points1, points2):
    min_distance = float('inf')  #
    for point1 in points1:
        for point2 in points2:
            distance = calculate_distance(point1, point2)
            if distance < min_distance:
                min_distance = distance
    return min_distance

def judgeTTC(TTC: float):
    if TTC > 2:
        # 安全
        return 1
    if 1.5 < TTC <= 2:
        # 轻度冲突
        return 2
    if 1 < TTC <= 1.5:
        # 严重冲突
        return 3
    if TTC <= 1:
        # 危险
        return 4

def Merge(dict1: dict, dict2: dict):
    merge = {}
    merge.update(dict1)
    merge.update(dict2)
    # print(dict1, dict2, merge)
    if merge:
        # print(merge)
        return merge
    else:
        return None

