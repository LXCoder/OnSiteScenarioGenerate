import logging
import os
import re
import sys

from logging.handlers import TimedRotatingFileHandler

from utils.config import log_level


def setup_log(log_dir, log_name, when="MIDNIGHT", interval=1, backupCount=7):
    """
        封装好的日志函数，调用此函数可以直接生成日志对象
    :param log_dir: 日志文件夹
    :param log_name: 日志文件名
    :param when: 单位时间 "S": Seconds, "M": Minutes, "H": Hours, "D": Days, "W": Week
                其中"W" 必须指定滚动日期，如 0 is Monday
                when="MIDNIGHT", interval=1 表示每天0点为更新点，每天生成一个文件
    :param interval: 滚动周期，指interval个when后，进行切割
    :param backupCount: 表示日志保存个数
    :return:
    """
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # 创建logger对象, 传入logger名字
    logger = logging.getLogger(log_name)
    # 移除其他文件写入，只保留一个，防止多进程间日志互相写入
    for handler in logger.handlers:
        logger.removeHandler(handler)

    log_path = os.path.join(log_dir, log_name)
    # 设置日志记录等级
    logger.setLevel(log_level)

    file_handler = TimedRotatingFileHandler(filename=log_path, when=when, interval=interval, backupCount=backupCount,
                                            encoding='utf-8')

    # filename="mylog" suffix设置，会生成文件名为 mylog.2020-02-25.log
    file_handler.suffix = "%Y-%m-%d.log"
    # extMatch是编译好正则表达式，用于匹配日志文件名后缀
    # 需要注意的是suffix和extMatch一定要匹配的上，如果不匹配，过期日志不会被删除。
    file_handler.extMatch = re.compile(r"^\d{4}-\d{2}-\d{2}.log$")

    # 定义日志输出格式
    logFormatStr = '[%(asctime)s] p%(process)s {%(pathname)s:%(lineno)d} %(levelname)s - %(message)s'
    file_handler.setFormatter(logging.Formatter(logFormatStr))
    logger.addHandler(file_handler)

    # 新增控制台日志输出，只记录 WARNING 及以上级别日志
    rf_handler = logging.StreamHandler(sys.stderr)  # 默认是sys.stderr
    rf_handler.setLevel(logging.ERROR)
    rf_handler.setFormatter(logging.Formatter(logFormatStr))
    logger.addHandler(rf_handler)
    return logger
