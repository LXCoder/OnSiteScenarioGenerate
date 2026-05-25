import json
import os
import time

from utils.RedisClient import MyRedisClient
from utils.config import BASEPATH, simulation_indexs, topic_mapping


def update():
    print("开始进行 OTA 检查")
    ota_data = json.load(open(os.path.join(BASEPATH, "Data", 'OTAData.json')))
    if ota_data:
        ota_data = {int(k): v for k, v in ota_data.items()}  # json 的key只能是字符串
        for index in simulation_indexs:
            if index in ota_data:
                ota_data[index]['globalTimeStamp'] = str(int(time.time() * 1000))
                MyRedisClient.set(f"{topic_mapping['ota']}_{index}", ota_data[index])
                print(f"自动驾驶算法: {index} 进行 OTA")
    print("OTA 结束")
    json.dump(None, open(os.path.join(BASEPATH, "Data", 'OTAData.json'), 'w'))