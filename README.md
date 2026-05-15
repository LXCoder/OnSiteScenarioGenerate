# OnSiteScenarioGenerate

## 1. 项目入口

项目入口是 [main.py](/home/dt/workspace/OnSiteScenarioGenerate/main.py:1)。

当前支持两种启动方式：

1. 单次仿真
2. 批量仿真


## 2. 为什么调整启动方式

之前的想法是同一个 Python 进程里反复执行 `main()`，并在每次执行前修改 `Utils/Constant.py` 里的常量。

现在改成了另一种方式：批量模式下，由一个父进程按顺序启动多个 `python main.py` 子进程，每个子进程负责一个场景。

这样更合适，原因是：

- `NET_PATH`、`DATA_DIR`、`BG_MODEL_FILENAME` 这类配置在多个模块中会在导入时读取
- 如果在同一个进程里反复切换配置，容易受到模块缓存影响
- 每个场景启动一个新进程，配置隔离更清晰，运行也更稳定


## 3. 单次启动

直接执行：

```bash
python main.py
```

这会按 [Utils/Constant.py](/home/dt/workspace/OnSiteScenarioGenerate/Utils/Constant.py:1) 当前配置启动一次仿真。


## 4. 批量启动

执行：

```bash
python main.py --batch-config batch_config.example.json
```

`--batch-config` 指向一个 JSON 文件，文件内容必须是数组，每一项表示一个仿真任务。

批量模式执行逻辑：

1. 读取 JSON 配置
2. 将当前任务转换为环境变量
3. 启动一个新的 `python main.py --single-run` 子进程
4. 当前任务结束后，继续下一个任务

`--single-run` 是内部参数，给批量模式使用，平时不需要手动传。


## 5. Utils/Constant.py 配置说明

配置文件位置：

- [Utils/Constant.py](/home/dt/workspace/OnSiteScenarioGenerate/Utils/Constant.py:1)

### 5.1 归一化/车辆参数

这些参数主要用于车辆动力学和观测归一化：

- `MAX_LANE_WIDTH`
- `MAX_SPEED`
- `MAX_STEER_ANGLE`
- `MAX_ACCEL`
- `MAX_DECEL`
- `MAX_STEER_DELTA`
- `MAX_ACC_DELTA`
- `WHEEL_BASE`

通常不需要为了切换场景频繁修改。


### 5.2 运行配置

#### `TRAIN_MODE`

- 含义：是否训练模式
- 默认值：`False`
- 环境变量：`TESSNG_TRAIN_MODE`

为 `False` 时，按推理/仿真模式运行。  
为 `True` 时，按训练模式运行。


#### `TOTAL_TIMESTEPS`

- 含义：训练总步数
- 默认值：`4000000`

只在训练模式下生效。


#### `DATA_DIR`

- 含义：场景数据根目录
- 默认值：`Data`
- 环境变量：`TESSNG_DATA_DIR`

代码里会进一步拼接：

- 推理/测试模式：`<DATA_DIR>/test`
- 训练模式：`<DATA_DIR>/train`

如果 `USE_TEST_LOGIC=False`，则一般用于读取 OpenSCENARIO 目录结构，比如 `Data/prod`。


#### `FILTER_SCENES`

- 含义：过滤掉不加载的场景文件
- 默认值：`scene_03.json` 到 `scene_11.json`
- 环境变量：`TESSNG_FILTER_SCENES`

支持两种传法：

1. JSON 数组字符串
2. 逗号分隔字符串

示例：

```bash
export TESSNG_FILTER_SCENES='["scene_01.json","scene_02.json"]'
```

或

```bash
export TESSNG_FILTER_SCENES='scene_01.json,scene_02.json'
```


#### `NET_PATH`

- 含义：TESSNG 路网文件路径
- 默认值：`Data\YYT_TJST_0619_unlimited.tess`
- 环境变量：`TESSNG_NET_PATH`

这是批量切换场景时最关键的配置之一。


#### `RL_ALGO`

- 含义：强化学习算法
- 当前值：`PPO`

目前代码中是固定值，不走环境变量覆盖。


#### `MODEL_SAVE_DIR`

- 含义：模型目录
- 计算方式：`"tessng_" + RL_ALGO.lower()`

当前 `RL_ALGO="PPO"` 时，对应目录为：

```text
tessng_ppo
```


#### `EGO_MODEL_FILENAME`

- 含义：主车模型文件名
- 默认值：`model.zip`
- 环境变量：`TESSNG_EGO_MODEL_FILENAME`


#### `BG_MODEL_FILENAME`

- 含义：背景车模型文件名
- 默认值：`model.zip.v6`
- 环境变量：`TESSNG_BG_MODEL_FILENAME`

这是批量切换任务时需要动态修改的关键配置之一。


#### `TENSORBOARD_LOG`

- 含义：TensorBoard 日志目录
- 默认值：`./Log/`


#### `TRAIN_MAX_STEPS`

- 含义：训练单回合最大步数配置
- 默认值：`1000`


#### `USE_TEST_LOGIC`

- 含义：是否使用测试场景 JSON 加载逻辑
- 默认值：`True`
- 环境变量：`TESSNG_USE_TEST_LOGIC`

说明：

- `True`：按 `Data/test/*.json` 或 `Data/train/*.json` 方式加载
- `False`：按 `.xosc + .xodr` 目录结构加载


#### `START_TIME_THRESHOLD`

- 含义：OpenSCENARIO 起始时间阈值
- 默认值：`1.0`


#### `REPEAT_SINGLE_SCENARIO`

- 含义：是否重复执行同一个场景
- 默认值：`False`
- 环境变量：`TESSNG_REPEAT_SINGLE_SCENARIO`

为 `True` 时，当前场景回合结束后会重复执行，不自动切换下一个场景。  
为 `False` 时，结束后会按当前逻辑切换场景或结束进程。


#### `EXIT_ON_SIMULATION_STOP`

- 含义：仿真停止后是否直接退出进程
- 默认值：`False`
- 环境变量：`TESSNG_EXIT_ON_SIMULATION_STOP`

这个配置是为了批量模式加的。

批量模式下会自动设为 `1`，这样每个子进程在仿真结束后会退出，父进程才能继续下一个任务。


## 6. 环境变量覆盖机制

现在 `Utils/Constant.py` 已支持通过环境变量覆盖部分配置，所以不需要真的去改源码里的常量值。

已支持覆盖的配置：

- `TESSNG_TRAIN_MODE`
- `TESSNG_DATA_DIR`
- `TESSNG_FILTER_SCENES`
- `TESSNG_NET_PATH`
- `TESSNG_EGO_MODEL_FILENAME`
- `TESSNG_BG_MODEL_FILENAME`
- `TESSNG_USE_TEST_LOGIC`
- `TESSNG_REPEAT_SINGLE_SCENARIO`
- `TESSNG_EXIT_ON_SIMULATION_STOP`

例如，手动执行单次仿真时，也可以这样传：

```bash
TESSNG_NET_PATH="Data/scenario_0a6bf824.tess" \
TESSNG_BG_MODEL_FILENAME="model.zip.v6" \
TESSNG_DATA_DIR="Data" \
python main.py
```


## 7. 批量配置文件格式

参考示例文件：

- [batch_config.example.json](/home/dt/workspace/OnSiteScenarioGenerate/batch_config.example.json:1)

示例：

```json
[
  {
    "name": "test-scene-a",
    "NET_PATH": "Data/scenario_0a6bf824.tess",
    "BG_MODEL_FILENAME": "model.zip.v6",
    "DATA_DIR": "Data",
    "FILTER_SCENES": ["scene_01.json", "scene_02.json"],
    "USE_TEST_LOGIC": true
  },
  {
    "name": "prod-scene-b",
    "NET_PATH": "Data/YYT_TJST_0619_unlimited.tess",
    "BG_MODEL_FILENAME": "model.zip.v7",
    "DATA_DIR": "Data/prod",
    "USE_TEST_LOGIC": false
  }
]
```

### 7.1 必填字段

每一项必须包含：

- `NET_PATH`
- `BG_MODEL_FILENAME`
- `DATA_DIR`


### 7.2 可选字段

每一项还可以配置：

- `name`
- `EGO_MODEL_FILENAME`
- `FILTER_SCENES`
- `USE_TEST_LOGIC`
- `REPEAT_SINGLE_SCENARIO`
- `TRAIN_MODE`


## 8. 推荐使用方式

如果只是调试一个场景：

```bash
python main.py
```

如果要按多个场景、多个模型组合批量跑：

```bash
python main.py --batch-config your_batch_config.json
```

推荐优先使用批量 JSON 配置，不建议反复手工改 `Utils/Constant.py` 再执行。

容器启动方式
```bash
docker run --rm -it --cap-add=SYS_ADMIN --network=host \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /etc/fonts:/etc/fonts \
-v /usr/share/fontconfig:/usr/share/fontconfig \
-v /usr/share/fonts:/usr/share/fonts \
-v /home/dt/workspace/OnSiteScenarioGenerate/Cert:/app/Cert \
-e TZ="Asia/Shanghai" -e DISPLAY=$DISPLAY \
--name tess_auto_x2 \
jida-inspur-4:2443/library/tess_auto:20260515

python main.py
# or
python main.py --batch-config your_batch_config.json
```


## 9. 当前实现限制

当前批量模式是串行执行：

- 上一个任务结束后
- 才会启动下一个任务

如果某个任务中途异常退出，批量流程会直接停止，并返回非 0 状态码。

另外，`RL_ALGO` 目前仍然写死在 `Utils/Constant.py` 里。如果后面需要按任务动态切换 `PPO/DQN`，建议再补一层环境变量支持。
