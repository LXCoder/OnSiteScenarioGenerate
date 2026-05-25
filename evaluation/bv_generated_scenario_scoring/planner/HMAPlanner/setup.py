from distutils.core import setup
import os
from Cython.Build import cythonize
import re
# folder_path = r"./planner/HMAPlanner"
# csv_regex = r'.py$'  # 这个正则表达式用不到了，但可以保留
# # 获取文件夹下的所有文件名
# file_names = os.listdir(folder_path)
# # 使用正则表达式筛选出.csv文件名
# csv_file_names = [
#     file_name for file_name in file_names if re.search(csv_regex, file_name)]
# print(csv_file_names)
# setup(ext_modules=cythonize(['Citymerge.py', 'collision_test.py', 'Comprehensive_planner.py', 'DMS_planner.py', 'Highway_act.py', 'Intersection_interaction.py', 'Intersection_replay.py', 'Location_lane.py', 'Location_lane_highway.py', 'Mixed.py', 'Roundabout_planner.py', 'scenario_extract.py', 'scenario_extract_highway.py', 'StatePredict.py','find_shortest_path.py']))
# setup(ext_modules=cythonize(['Citymerge.py', 'hmaPlanner.py', 'Highway_act.py', 'Intersection_interaction.py', 'Roundabout_planner.py']))
setup(ext_modules=cythonize(['Roundabout_planner.py']))
# python setup.py build_ext --inplace
