import sys, os

# 添加本开发包绝对路径到搜索路径中
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)


def send_signal(context, value, network_info=None, error=False):
    """
        发送信号，通过此接口调用进度条，异步修改信息窗
    Args:
        context: 
        value: 进度
        network_info: 提示窗内容
        error: 是否有异常
    """
    if not (context and context.get("signal") and context.get("pb")):
        return

    signal = context["signal"]
    pb = context["pb"]
    network_info = network_info or {}
    signal.emit(pb, value, network_info, error)
    return
