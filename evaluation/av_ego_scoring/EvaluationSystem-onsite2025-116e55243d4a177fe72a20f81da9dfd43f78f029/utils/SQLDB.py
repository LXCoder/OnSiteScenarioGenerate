import inspect
import logging
import time
import pymysql
import json

try:
    # Some environments provide lowercase module name.
    from dbutils.pooled_db import PooledDB
except Exception:
    # Linux with DBUtils package commonly exposes capitalized module name.
    from DBUtils.PooledDB import PooledDB
from utils.config import MySQLConfig, log_name


logger = logging.getLogger(log_name)


class DecoratedDatabaseAllMethod:
    """
        装饰数据库链接的类，当不进行数据库配置时，所有的数据库操作都被屏蔽
    """

    def __init__(self, func):
        self.func = func

    def __get__(self, obj, cls=None):
        def wrapper(*args, **kwargs):
            logger.info(f'调用数据库方法: {PC.TESSNG["isUseDatabaseConfig"]} {self.func.__name__} {args, kwargs}')
            if not PC.TESSNG["isUseDatabaseConfig"]:
                # 不使用数据库配置，所有的增删改查都被屏蔽，统一返回 None
                return

            # 使用数据库配置时，返回正常操作
            try:
                # 实例方法
                ret = self.func(obj, *args, **kwargs)
            except TypeError:
                # 类方法或者静态方法, 类方法不需要传递本身
                ret = self.func(*args, **kwargs)
            return ret
        for attr in "__module__", "__name__", "__doc__":
            setattr(wrapper, attr, getattr(self.func, attr))
        return wrapper

def decorate_database_class(cls):
    """
        对类的装饰器，将某类的所有方法进行遍历装饰
    :param cls:
    :return:
    """
    for name, meth in inspect.getmembers(cls):
        if inspect.ismethod(meth) or inspect.isfunction(meth):
            logger.info(f"识别方法: {name} {inspect.ismethod(meth)} {inspect.isfunction(meth)}")
            setattr(cls, name, DecoratedDatabaseAllMethod(meth))
    return cls


# @decorate_database_class
class SQLDB:
    def __init__(self, host, port, user, password, db):
        self.POOL = PooledDB(
            creator=pymysql,  # 使用链接数据库的模块
            maxconnections=0,  # 连接池允许的最大连接数，0和None表示不限制连接数
            mincached=2,  # 初始化时，链接池中至少创建的空闲的链接，0表示不创建
            maxcached=10,  # 链接池中最多闲置的链接，0和None不限制
            maxshared=0,  # 链接池中最多共享的链接数量，0和None表示连接不共享
            blocking=True,  # 连接池中如果没有可用连接后，是否阻塞等待。True，等待；False，不等待然后报错
            maxusage=100,  # 一个链接最多被重复使用的次数，None表示无限制
            setsession=[],  # 开始会话前执行的命令列表，如：["set datestyle to ...", "set time zone ..."]
            ping=0,  # 工作中用的最多的就是0、4、7
            host=host,
            port=port,
            user=user,
            password=password,
            db=db,
            charset='utf8',
            connect_timeout=600,
            read_timeout=600,
            write_timeout=600,
        )

    def get_new_uuid(self):
        is_ok, result = self.execute_sql("SELECT UUID()")
        return result and result[0]['UUID()']

    # 执行SQL语句
    def execute_sql(self, sql, param_values=None):
        logger.info(f"当前连接池状态, 当前正在使用的连接数: {self.POOL._connections}")

        # 从连接池拿一个链接
        conn = self.POOL.connection()
        cursor = conn.cursor(cursor=pymysql.cursors.DictCursor)

        is_ok = True
        result = None
        try:
            cursor.execute(sql, param_values)
            result = cursor.fetchall()
            conn.commit()
            print(f"数据库操作执行成功: {sql, param_values and [i if len(str(i)) < 100 else '长度:' + str(len(str(i))) for i in (param_values or [])]}")
        except:
            print(f"数据库操作执行错误: {sql, param_values and [i if len(str(i)) < 100 else '长度:' + str(len(str(i))) for i in (param_values or [])]}")
            is_ok = False
        finally:
            cursor.close()
            conn.close()
        return is_ok, result

    def insert_one_with_dict(self, table, data):
        # 通过字典插入数据库
        sql = f"INSERT INTO {table} (" + ", ".join(data.keys()) + ") VALUES (" + ", ".join(["%s"] * len(data)) + ")"
        is_ok, _ = MySQLDB.execute_sql(sql, tuple(data.values()))
        return is_ok

    # 仿真方案列表获取
    def select_scheme_list(self, ):
        # 查询方案列表
        # sql = f"SELECT id, name, DATE_FORMAT(createdTime, '%y-%m-%d %H:%i:%s') as createdTime FROM {MYSQL_TABLE_MAIN} ORDER BY createdTime"
        sql = f"""
        SELECT
            pm.id,
            pm.name,
            CASE
                WHEN EXISTS (
                    SELECT 1
                    FROM {PC.MYSQL['TABLE']['subSimu_info']} si
                    WHERE si.schemeId = pm.id AND si.currentStatus = 'start'
                ) THEN 1
                ELSE 0
            END AS isRunning
        FROM {PC.MYSQL["TABLE"]["parm_main"]} pm;
        """
        schemes = self.execute_sql(sql)
        schemes = [
            {'id': item['id'], 'name': item['name'], 'isRunning': bool(item['isRunning'])}
            for item in schemes
        ]

        return schemes

    # 仿真方案下发
    def select_parms_data(self, ID):
        # 建立连接
        conn = self.POOL.connection()
        cursor = conn.cursor(cursor=pymysql.cursors.DictCursor)
        
        # (1)查询仿真方案数据
        sql_select_main = F"SELECT * FROM {PC.MYSQL['TABLE']['parm_main']} WHERE id = %s"
        cursor.execute(sql_select_main, (ID, ))
        main = cursor.fetchall()

        # (2)查询发车点数据
        sql_select_dp = f"SELECT * FROM {PC.MYSQL['TABLE']['parm_departure']} WHERE schemeId = %s"
        cursor.execute(sql_select_dp, (ID,))
        dp = cursor.fetchall()

        # (3)查询车辆速度数据
        sql_select_vs = f"SELECT * FROM {PC.MYSQL['TABLE']['parm_speed']} WHERE schemeId = %s"
        cursor.execute(sql_select_vs, (ID, ))
        vs = cursor.fetchall()

        # (4)查询转向比例数据
        sql_select_tr = f"SELECT * FROM {PC.MYSQL['TABLE']['parm_turning']} WHERE schemeId = %s"
        cursor.execute(sql_select_tr, (ID, ))
        tr = cursor.fetchall()

        # 关闭连接
        conn.commit()
        cursor.close()
        conn.close()
        
        if main:
            parms_return = {
                "departurePoint": {},
                "vehicleSpeed": {},
                "turningRatio": {},
            }
            exclude = ["id", "name", "createTime", "schemeId", "schemeName", "dpId", "number", "vehType", "interName", "direction", "updateTime"]

            # (1)转换仿真方案数据
            parms_return.update({key:value for key,value in main[0].items() if key not in exclude})
            # 标量转为布尔型
            for key in ["isCompleteMapping", "isUseFinalActualSpeed"]:
                parms_return[key] = bool(parms_return[key])

            # (2)转换发车点数据
            for value in dp:
                dpId = value["dpId"]
                if dpId not in parms_return["departurePoint"]:
                    parms_return["departurePoint"][dpId] = []
                parms_return["departurePoint"][dpId].append({k:v for k,v in value.items() if k not in exclude})
            # 规整发车点信息 TODO
            parms_return["departurePoint"] = [{"id": dpId, "interval": interval} for dpId, interval in parms_return["departurePoint"].items()]

            # (3)转换车辆速度数据
            for value in vs:
                vehType = value["vehType"]
                parms_return["vehicleSpeed"][vehType] = {k:v for k,v in value.items() if k not in exclude}
            
            # (4)转换转向比例数据
            for value in tr:
                interName = value["interName"]
                direction = value["direction"]
                if interName not in parms_return["turningRatio"]:
                    parms_return["turningRatio"][interName] = {}
                parms_return["turningRatio"][interName][direction] = {k:v for k,v in value.items() if k not in exclude}

            # TODO 重置转向数据
            parms_return["turningRatio"] = json.loads(main[0]['turningRatio'])
            return parms_return
        
        else:
            return {}
    
    # 仿真方案保存
    def insert_parms_data(self, parms):
        # 建立连接
        conn = self.POOL.connection()
        cursor = conn.cursor(cursor=pymysql.cursors.DictCursor)

        parms['turningRatio'] = json.dumps(parms['turningRatio'])
        # (1)插入仿真方案数据
        # 仿真方案表的字段
        parm1 = ["name", "accuracy", "accemultiples", "isCompleteMapping", "isUseFinalActualSpeed", "safeInterval", "safeDistance", "changeLaneFrequency", 'turningRatio']
        sql_insert_main = f"INSERT INTO {PC.MYSQL['TABLE']['parm_main']} (" + ", ".join(parm1) + ") VALUES (" + ", ".join(["%s"] * len(parm1)) + ")"
        values_main = tuple([parms[key] for key in parm1])
        cursor.execute(sql_insert_main, values_main)
        
        # (1.5)查询获取最新一行的ID
        sql_select_id_lastest = f"SELECT id FROM {PC.MYSQL['TABLE']['parm_main']} ORDER BY createTime DESC LIMIT 1"
        cursor.execute(sql_select_id_lastest)
        schemeId = cursor.fetchall()[0]["id"]
        schemeName = parms["name"]

        # (2)插入发车点数据
        # 发车点表的字段
        parm2 = ["schemeId", "schemeName", "dpId", "number", "duration", "vehCount", "composition", "vehCombinationCode"]
        sql_insert_dp = f"INSERT INTO {PC.MYSQL['TABLE']['parm_departure']} (" + ", ".join(parm2) + ") VALUES (" + ", ".join(["%s"] * len(parm2)) + ")"
        for dp in parms["departurePoint"]:
            dpId = dp['id']
            for i, value in enumerate(dp['interval']):
                values_dp = tuple([schemeId, schemeName, dpId, i] + [value[key] for key in parm2[4:]])
                cursor.execute(sql_insert_dp, values_dp)

        # parm2 = ["schemeId", "schemeName", "dpId", "interval"]
        # sql_insert_dp = f"INSERT INTO {PC.MYSQL['TABLE']['parm_departure']} (" + ", ".join(parm2) + ") VALUES (" + ", ".join(["%s"] * len(parm2)) + ")"
        # for dp in parms["departurePoint"]:
        #     # values_dp = tuple([schemeId, schemeName, dpId, i] + [value[key] for key in parm2[4:]])
        #     cursor.execute(sql_insert_dp, (schemeId, schemeName, dp['id'], json.dumps(dp['interval'])))


        # (3)插入车辆速度数据
        # 车辆速度表的字段
        parm3 = ["schemeId", "schemeName", "vehType", "desireSpeed", "speedStd", "maxLimitSpeed", "minLimitSpeed"]
        sql_insert_vs = f"INSERT INTO {PC.MYSQL['TABLE']['parm_speed']} (" + ", ".join(parm3) + ") VALUES (" + ", ".join(["%s"] * len(parm3)) + ")"
        for vehTypeCode, values in parms["vehicleSpeed"].items():
            values_vs = tuple([schemeId, schemeName, vehTypeCode] + [parms["vehicleSpeed"][vehTypeCode][key] for key in parm3[3:]])
            cursor.execute(sql_insert_vs, values_vs)

        # (4)插入转向比例数据
        # # 转向比例表的字段
        # parm4 = ["schemeId", "schemeName", "interName", "direction", "straight", "turnLeft", "turnRight", "changeLanePreparationDistance"]
        # for interName in parms["turningRatio"].keys():
        #     for direction, values in parms["turningRatio"][interName].items():
        #         # 三个转向不一定都有
        #         parm4_temp = parm4.copy()
        #         for key in ["straight", "turnLeft", "turnRight"]:
        #             if key not in parms["turningRatio"][interName][direction]:
        #                 parm4_temp.remove(key)
        #         sql_insert_tr = f"INSERT INTO {PC.MYSQL['TABLE']['parm_turning']} (" + ", ".join(parm4_temp) + ") VALUES (" + ", ".join(["%s"] * len(parm4_temp)) + ")"
        #         values_tr = tuple([schemeId, schemeName, interName, direction] + [parms["turningRatio"][interName][direction][key] for key in parm4_temp[4:]])
        #         cursor.execute(sql_insert_tr, values_tr)
        
        # 关闭连接
        conn.commit()
        cursor.close()
        conn.close()

    # 仿真方案删除
    def delete_parms_data(self, ID):
        # 删除数据
        sql = f"DELETE FROM {PC.MYSQL['TABLE']['parm_main']} WHERE id = %s"
        MySQLDB.execute_sql(sql, (ID,))

    # 仿真方案改名
    def rename_scheme(self, name, ID):
        # 修改数据
        sql = f"UPDATE {PC.MYSQL['TABLE']['parm_main']} SET name = %s WHERE id = %s"
        MySQLDB.execute_sql(sql, (name, ID))

    # 仿真方案平行仿真
    def insert_subSimu_data(self, schemeIds):
        mainSimu_id = self.select_mainSimu_data('id')

        # 建立连接
        conn = self.POOL.connection()
        cursor = conn.cursor(cursor=pymysql.cursors.DictCursor)

        # (1)插入子仿真批次数据-关联主仿真
        parm1 = ["expectedSimuDuration", "mainSimuId"]
        expectedSimuDuration = 3
        sql_insert_batch = f"INSERT INTO {PC.MYSQL['TABLE']['subSimu_batch']} (" + ", ".join(parm1) + ") VALUES (" + ", ".join(["%s"] * len(parm1)) + ")"
        values_batch = tuple([expectedSimuDuration, mainSimu_id])
        cursor.execute(sql_insert_batch, values_batch)

        # (1.5)查询获取最新一行的批次ID
        sql_select_id_lastest = f"SELECT batchId FROM {PC.MYSQL['TABLE']['subSimu_batch']} ORDER BY createTime DESC LIMIT 1"
        cursor.execute(sql_select_id_lastest)
        batchId = cursor.fetchall()[0]["batchId"]

        # (2)插入子仿真参数数据
        parm2 = ["batchId", "schemeId", "simuParms"]
        sql_insert_info = f"INSERT INTO {PC.MYSQL['TABLE']['subSimu_info']} (" + ", ".join(parm2) + ") VALUES (" + ", ".join(["%s"] * len(parm2)) + ")"
        for schemeId in schemeIds:
            parms = self.select_parms_data(schemeId)
            values_info = tuple([batchId, schemeId, json.dumps(parms)])
            cursor.execute(sql_insert_info, values_info)

        # 关闭连接
        conn.commit()
        cursor.close()
        conn.close()

    # 平行仿真批次列表
    def select_subBatch_list(self):
        # 查询仿真批次信息
        sql = f"SELECT * FROM {PC.MYSQL['TABLE']['subSimu_batch']} order by createTime DESC"  # TODO 分页限制数量

        subBatch_list = []
        for subBatch_data in MySQLDB.execute_sql(sql):
            subBatch_list.append(subBatch_data)
        return subBatch_list

    # 平行仿真批次的信息
    def select_subBatch_data(self, subBatchId):
        # 查询仿真批次信息
        sql = f"SELECT * FROM {PC.MYSQL['TABLE']['subSimu_batch']} WHERE batchId='{subBatchId}'"
        # 批次基本信息
        subBatch_data = MySQLDB.execute_sql(sql)[0]

        subBatch_data['simu_data'] = []
        # 补充批次的子仿真信息
        sql = f"SELECT * FROM {PC.MYSQL['TABLE']['subSimu_info']} WHERE batchId='{subBatchId}'"
        for simu_data in MySQLDB.execute_sql(sql):
            subBatch_data['simu_data'].append(simu_data)
        return subBatch_data

    # 平行仿真子仿真的信息
    def select_subSimu_data(self, subSimuId):
        # 子仿真信息
        sql = f"SELECT * FROM {PC.MYSQL['TABLE']['subSimu_info']} WHERE subSimuId='{subSimuId}'"
        subSimu_data = MySQLDB.execute_sql(sql)[0]
        return subSimu_data

    # 推演结果列表获取
    def select_result_list(self, ID):
        # 查询推演结果列表
        sql = f"SELECT simuId as name, duration, DATE_FORMAT(mtime, '%y-%m-%d %H:%i:%s') as mtime FROM {PC.MYSQL['TABLE']['result']} WHERE schemeId='{ID}' ORDER BY mtime"
        schemes = MySQLDB.execute_sql(sql)

        return schemes

    # 推演结果详情获取
    def select_result_data(self, name):
        # 查询推演结果列表
        sql = f"SELECT result FROM {PC.MYSQL['TABLE']['result']} WHERE simuId='{name}'"
        result = MySQLDB.execute_sql(sql)
        # 格式转换  TODO redult 是否会存在
        result = json.loads(result[0]["result"])

        return result

    # 推演结果保存
    def insert_result_data(self, ID, result_data):
        # 建立连接
        conn = self.POOL.connection()
        cursor = conn.cursor(cursor=pymysql.cursors.DictCursor)

        # 插入推演结果数据
        parm = ["schemeId", "result"]
        sql_insert_result = f"INSERT INTO {PC.MYSQL['TABLE']['result']} (" + ", ".join(parm) + ") VALUES (" + ", ".join(["%s"] * len(parm)) + ")"
        values_result = tuple([ID, json.dumps(result_data)])
        cursor.execute(sql_insert_result, values_result)

        # 关闭连接
        conn.commit()
        cursor.close()
        conn.close()

    # 推演结果删除
    def delete_result(self, name):
        # 删除数据
        sql = f"DELETE FROM {PC.MYSQL['TABLE']['result']} WHERE simuId = %s"
        MySQLDB.execute_sql(sql, (name,))

    # 新增主仿真记录
    def insert_mainSimu_data(self, simuData):
        # 建立连接
        conn = self.POOL.connection()
        cursor = conn.cursor(cursor=pymysql.cursors.DictCursor)
        sql_insert_batch = f"INSERT INTO {PC.MYSQL['TABLE']['mainSimu']} (" + ", ".join(simuData.keys()) + ") VALUES (" + ", ".join(["%s"] * len(simuData)) + ")"
        values_batch = tuple(simuData.values())
        cursor.execute(sql_insert_batch, values_batch)
        # 关闭连接
        conn.commit()
        cursor.close()
        conn.close()

    def select_mainSimu_data(self, column=None):
        sql = f"SELECT * from {PC.MYSQL['TABLE']['mainSimu']} ORDER BY createTime DESC limit 1"
        result = MySQLDB.execute_sql(sql)
        # 格式转换
        if result:
            result = result[0]
            if column:
                # 只获取单列数据
                result = result[column]
        else:
            result = None
        return result

    def update_mainSimu_data(self, parms):
        main_simu_id = self.select_mainSimu_data('id')
        # 更新主仿真数据，包含仿真状态以及仿真参数
        sql = f"UPDATE {PC.MYSQL['TABLE']['mainSimu']} SET " + ",".join([f"{k}=%s" for k in parms]) + " where id = %s"
        MySQLDB.execute_sql(sql, tuple(list(parms.values()) + [main_simu_id]))
        return

    def insert_event_data(self, data):
        # 新增事件(目前只针对主仿真)
        sql = f"INSERT INTO {PC.MYSQL['TABLE']['simuEvent']} (" + ", ".join(data.keys()) + ") VALUES (" + ", ".join(["%s"] * len(data)) + ")"
        MySQLDB.execute_sql(sql, tuple(data.values()))
        return

    def select_event_data(self, simu_id, event_id=None, status=None):
        parm_tuple = [simu_id]
        # 获取事件信息
        sql = f"SELECT * from {PC.MYSQL['TABLE']['simuEvent']} where simuId=%s"
        if event_id:
            sql += " and id = %s"
            parm_tuple.append(event_id)
        if status:
            sql += " and status in %s"
            parm_tuple.append(tuple(status))
        result = MySQLDB.execute_sql(sql, tuple(parm_tuple))
        return result

    def update_event_data(self, event_id, parms):
        # 更新子仿真的仿真信息
        sql = f"UPDATE {PC.MYSQL['TABLE']['simuEvent']} SET " + ",".join([f"{k}=%s" for k in parms]) + " where id = %s"
        MySQLDB.execute_sql(sql, tuple(list(parms.values()) + [event_id]))
        return

    def stop_event(self, simu_id, event_id=None):
        """
            通过更新事件的结束时间移除事件
        :param simu_id:
        :param event_id:
        :return:
        """
        parm_tuple = [int(time.time()), simu_id]
        sql = f"UPDATE {PC.MYSQL['TABLE']['simuEvent']} SET endTime=%s where simuId = %s and status != 'stop'"
        if event_id:
            sql += " and id = %s"
            parm_tuple.append(event_id)
        MySQLDB.execute_sql(sql, tuple(parm_tuple))

    def update_evaluation_data(self, evaluation_id, parms):
        # 更新主仿真数据，包含仿真状态以及仿真参数
        sql = f"UPDATE Evaluation SET " + ",".join([f"{k}=%s" for k in parms]) + " where id = %s"
        MySQLDB.execute_sql(sql, tuple(list(parms.values()) + [evaluation_id]))
        return

# 实例化 SQLDB
# MySQLDB = SQLDB(
#     MySQLConfig.host,
#     MySQLConfig.port,
#     MySQLConfig.user,
#     MySQLConfig.pwd,
#     MySQLConfig.db
# )
MySQLDB = None
