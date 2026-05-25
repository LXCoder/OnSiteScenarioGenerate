# class FrameCache(object):
#     def __init__(self, maxSize):
#         self.cache = {}  # 使用字典存储数据
#         self.max_size = maxSize  # 缓存大小，包括当前帧
#
#     def get(self, frame_num):
#         """获取指定帧号的数据"""
#         return self.cache.get(frame_num)
#
#     def set(self, frame_num, data):
#         """设置指定帧号的数据"""
#         self.cache[frame_num] = data
#         self._limit_size()
#
#     def _limit_size(self):
#         """限制缓存大小，删除过期数据"""
#         if len(self.cache) > self.max_size:
#             # 获取当前帧号
#             current_frame_num = max(self.cache.keys())
#             # 删除当前帧之前的帧的数据
#             for i in range(current_frame_num - self.max_size, current_frame_num):
#                 if i in self.cache:
#                     del self.cache[i]


# # 初始化缓存
# cache = FrameCache(2)
#
# # 存储当前帧数据
# cache.set(0, [0, 0])
#
# for i in range(5):
#     cache.set(i, i)
#
# print(cache)


# 定义一个缓存类，存储当前帧和上一帧数据
class FrameCache:
    def __init__(self):
        self.current_frame = None
        self.previous_frame = None

    # 存储当前帧数据
    def add_frame(self, frame_data):
        self.previous_frame = self.current_frame
        self.current_frame = frame_data

    # 获取当前帧数据
    def get_current_frame(self):
        return self.current_frame

    # 获取上一帧数据
    def get_previous_frame(self):
        return self.previous_frame

class Record(object):
    def __init__(self):
        # 记录每个场景的持续时间，如果restart了就清零
        self.recordLastTime = {}



