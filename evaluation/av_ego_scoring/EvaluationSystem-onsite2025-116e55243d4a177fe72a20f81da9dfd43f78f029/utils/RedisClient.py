import json

from utils.config import REDIS_HOST, REDIS_PORT, REDIS_DB, REDIS_USER, REDIS_PWD
from redis.client import StrictRedis
from redis.connection import ConnectionPool


class RedisClient:
    def __init__(self, host, port, db, user, password):
        self.pool = ConnectionPool(host=host, 
                                   port=port, 
                                   db=db, 
                                   username = user, 
                                   password = password, 
                                   decode_responses=False)
        self.redis_client = StrictRedis(connection_pool=self.pool)

    # 保存键值对
    def set(self, key, value):
        try:
            self.redis_client.set(key, json.dumps(value))
            return True
        except:
            return False

    # 根据键获取值
    def get(self, key):
        try:
            return json.loads(self.redis_client.get(key).decode('utf-8'))
        except:
            return None


# 实例化 RedisClient
MyRedisClient = RedisClient(REDIS_HOST, REDIS_PORT, REDIS_DB, REDIS_USER, REDIS_PWD)
