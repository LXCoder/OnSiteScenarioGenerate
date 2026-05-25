from redis.client import StrictRedis
from redis.connection import ConnectionPool
import json


class RedisClient:
    def __init__(self, host, port, user, password, db):
        self.pool = ConnectionPool(host=host, 
                                   port=port,
                                   username=user,
                                   password=password,
                                   db=db,
                                   decode_responses=False
                                   )
        self.redis_client = StrictRedis(connection_pool=self.pool)

    # 保存键值对
    def set(self, key, value):
        self.redis_client.set(key, json.dumps(value))

    # 根据键获取值
    def get(self, key):
        try:
            return_data = json.loads(self.redis_client.get(key).decode('utf-8'))
        except:
            return_data = None
        return return_data


# # 实例化 RedisClient
# MyRedisClient = RedisClient(
#     PC.REDIS["CONN"]["HOST"],
#     PC.REDIS["CONN"]["PORT"],
#     PC.REDIS["CONN"]["USER"],
#     PC.REDIS["CONN"]["PWD"],
#     PC.REDIS["CONN"]["DB"]
# )
#
# # Redis存默认主仿真状态（停止）
# MyRedisClient.set(PC.REDIS["KEY"]["main_status"], "stop")


