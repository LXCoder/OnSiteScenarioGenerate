import time
import traceback
from multiprocessing import Process

from utils.SQLDB import MySQLDB
from utils.config import *
from utils.functions import calc_evaluation
from utils.log import setup_log
from utils.server import api_server


if __name__ == '__main__':
    logger = setup_log(os.path.join(LOG_FILE_DIR, 'main'), log_name, when="MIDNIGHT")
    # api_server()

    # 开启 HTTP 服务
    api_p = Process(target=api_server, args=())
    api_p.start()

    # 轮询数据库，检查是否需要进行测试记录的评价
    while True:
        try:
            # 检索数据库，一旦发现新的数据，进行计算
            # 查询需要进行计算的评价
            sql = "select * from Evaluation where status='wait' order by createTime limit 1"
            is_ok, evaluation_records = MySQLDB.execute_sql(sql)

            if not evaluation_records:
                logger.warning(f"不存在需评价记录")
                time.sleep(2)
                continue

            # 发现新的测试记录，子进程进行评价
            evaluation_record = evaluation_records[0]
            print(f'测试记录: {evaluation_record["id"]}')
            api_p = Process(target=calc_evaluation, args=(evaluation_record,))
            # 数据入库,记录仿真的单次得分，后续展示时根据实时的权重分配进行整合
            MySQLDB.update_evaluation_data(evaluation_record['id'], {"status": 'start'})
            api_p.start()
        except:
            logger.exception(f"检查测试记录出错")
            time.sleep(1)
