# Backend

This directory is the backend service root for the autonomous driving model testing platform.

## Run

```bash
cd backend
python run.py
```

Environment variables:

- `BACKEND_HOST` default `0.0.0.0`
- `BACKEND_PORT` default `5000`
- `BACKEND_DEBUG` default `false`
- `BACKEND_LOG_DIR` default `backend/logs`
- `BACKEND_LOG_LEVEL` default `INFO`
- `BACKEND_LOG_BACKUP_COUNT` default `14`
- `BACKEND_TASK_DATA_ROOT` default `backend/task_data`
- `BACKEND_USE_DOCKER` default `false`
- `BACKEND_DOCKER_IMAGE` Docker image used by the worker when Docker mode is enabled
- `BACKEND_TASK_TIMEOUT` default `3600`
- `BACKEND_MYSQL_HOST` default `127.0.0.1`
- `BACKEND_MYSQL_PORT` default `3306`
- `BACKEND_MYSQL_USER` default `root`
- `BACKEND_MYSQL_PASSWORD` default empty
- `BACKEND_MYSQL_DATABASE` default `onsite_backend`
- `BACKEND_MYSQL_CHARSET` default `utf8mb4`
- `BACKEND_JWT_SECRET_KEY` JWT 解码密钥
- `BACKEND_JWT_ALGORITHMS` default `HS256`
- `BACKEND_JWT_ISSUER` default `auth0`

## Logging

后端现在使用标准 `logging` 模块，日志会按天切分：

- `backend.log`: 应用综合日志
- `backend.error.log`: `WARNING` 及以上
- `gunicorn.access.log`: Gunicorn 访问日志
- `gunicorn.error.log`: Gunicorn 错误日志

日志目录默认是 `backend/logs`，可通过 `BACKEND_LOG_DIR` 覆盖。默认保留 14 天，可通过 `BACKEND_LOG_BACKUP_COUNT` 调整。

本地直接运行：

```bash
cd backend
python run.py
```

生产环境建议使用：

```bash
cd backend
gunicorn -c gunicorn.conf.py run:app
```

常用 Gunicorn 环境变量：

- `BACKEND_GUNICORN_BIND` default `0.0.0.0:5000`
- `BACKEND_GUNICORN_WORKERS` default `2`
- `BACKEND_GUNICORN_WORKER_CLASS` default `sync`
- `BACKEND_GUNICORN_TIMEOUT` default `120`
- `BACKEND_GUNICORN_GRACEFUL_TIMEOUT` default `30`
- `BACKEND_GUNICORN_KEEPALIVE` default `5`

## Auth

当前接口使用请求头承载最小身份信息：

- `Token`

权限规则：

- 管理员可以查看、下载、取消所有任务
- 普通用户只能查看、下载、取消自己创建的任务
- 创建任务时会根据 `Token` 解析出的 `username` 查询 `user` 表，并将对应 `userId` 写入 `tasks.user_id`

这些规则由统一认证包装器 [verification.py](/home/dt/workspace/OnSiteScenarioGenerate/backend/app/utils/verification.py) 处理，接口本身不再重复解析请求头。

## API

### `POST /tasks/create`

创建任务。支持两种输入方式：

1. `application/json`
2. `multipart/form-data`，可同时上传文件

JSON 请求体示例：

```json
{
  "name": "demo-task",
  "bg_model": "model.zip.v6",
  "qtype": 0
}
```

请求头示例：

```http
Token: <jwt-token>
```

`qtype`: 赛题类型
  - 0: A
  - 1: B
  - 2: C

`bg_model`: 使用 `/home/dt/workspace/OnSiteScenarioGenerate/tessng_ppo` 下的模型

需要上传模型文件的，请使用表单 `form-data` 发起请求，其他字段和上述的 `json` 字段一样，`bg_model` 可以忽略。

兼容字段：

- `batch_items`
- `batch_config`
- `task`

如果都不传，会把除 `name` 外的其他字段当作单个任务项。

成功响应：`201 Created`

```json
{
  "task_id": "task_20260518123045_a1b2c3",
  "user_id": "u_1001",
  "name": "demo-task",
  "status": "PENDING",
  "create_time": "2026-05-18T12:30:45.123456+00:00",
  "start_time": null,
  "finish_time": null,
  "batch_config_path": ".../<task_data_root>/task_.../batch_config.json",
  "request_json": ".../<task_data_root>/task_.../request.json",
  "log_dir": ".../<task_data_root>/task_.../logs",
  "output_dir": ".../<task_data_root>/task_.../output",
  "message": "Task created",
  "exit_code": null,
  "cancel_requested": 0,
  "batch_config": [
    {
      "name": "scene-a",
      "NET_PATH": "Data/scenario_0a6bf824.tess",
      "BG_MODEL_FILENAME": "model.zip.v6",
      "DATA_DIR": "Data/test",
      "USE_TEST_LOGIC": true
    }
  ],
  "request": {
    "payload": {
      "name": "demo-task",
      "batch_items": [...]
    },
    "uploads": []
  }
}
```

失败响应：`400 Bad Request`

```json
{
  "error": "Request must provide batch_items or batch_config"
}
```

注意：请详细查看 `OnSiteScenarioGenerate/backend/app/docker/runner.py:161` 拉起的 docker 容器挂在卷配置。

### `GET /tasks`

查询任务列表。

成功响应：`200 OK`

```json
{
  "count": 1,
  "items": [
    {
      "task_id": "task_20260518123045_a1b2c3",
      "user_id": "u_1001",
      "name": "demo-task",
      "status": "RUNNING",
      "create_time": "2026-05-18T12:30:45.123456+00:00",
      "start_time": "2026-05-18T12:30:50.000000+00:00",
      "finish_time": null,
      "message": "Running"
    }
  ]
}
```

### `GET /tasks/<task_id>`

查询单个任务详情。返回结构与 `POST /tasks/create` 的成功响应一致。

失败响应：`404 Not Found`

```json
{ "error": "Task not found" }
```

### `GET /tasks/<task_id>/logs`

获取日志文本。

查询参数：

- `stream`，可选值 `combined`、`stdout`、`stderr`，默认 `combined`
- `tail`，返回最后多少行，默认 `200`。`<=0` 返回全部

成功响应：`200 OK`，`text/plain`

### `GET /tasks/<task_id>/download`

下载任务归档压缩包。

成功响应：`200 OK`，返回 `application/zip`

压缩包里会包含：

- `batch_config.json`
- `request.json`
- `logs/`
- `output/`
- `upload/`

### `GET /tasks/<task_id>/evaluation`

读取任务输出目录下最新一次仿真的评价结果。

后端会优先读取：

- `output/evaluation/<latest_dir>/summary_averages_100.csv`
- `output/evaluation/<latest_dir>/per_scene_detailed_100.csv`

成功响应：`200 OK`

```json
{
  "task_id": "task_20260522012219_9fef2a",
  "summary": {
    "topic": "A",
    "scene_count": 6,
    "bv_safety_20_avg": 6.667,
    "total_100_avg": 20.442
  },
  "per_scene": [
    {
      "topic": "A",
      "scene": "scenario_1eee3255",
      "total_100": 44.6,
      "av_status": "ok",
      "av_error": null
    }
  ]
}
```

### `POST /tasks/<task_id>/cancel`

取消任务。

成功响应：`200 OK`

返回任务最新状态；如果任务还未开始，会直接变为 `CANCELLED`。

## Task Status

任务状态枚举：

- `PENDING`
- `RUNNING`
- `SUCCESS`
- `FAILED`
- `TIMEOUT`
- `CANCELLED`

## Task storage

Runtime data is stored under `BACKEND_TASK_DATA_ROOT`.

默认值是 `backend/task_data/`，也可以配置成独立目录，例如：

```bash
export BACKEND_TASK_DATA_ROOT=/data/onsite/backend-task-data
```

或者在 `.env` 配置文件中配置环境变量
`
BACKEND_TASK_DATA_ROOT=/data/onsite/backend-task-data
`

## Database

后端已从 SQLite 切换到 MySQL。启动时会自动确保 `tasks` 表存在。

用户认证依赖已有 `user` 表。见生产环境数据库 `onsite_prod`。
```yml
BACKEND_MYSQL_HOST="sh-cynosdbmysql-grp-7cwak2fy.sql.tencentcdb.com"
BACKEND_MYSQL_PORT="20908"
BACKEND_MYSQL_USER="root"
BACKEND_MYSQL_PASSWORD="SZdGuvr8vcEWR9pCvkaWNsr2"
BACKEND_MYSQL_DATABASE="onsite_prod"
BACKEND_MYSQL_CHARSET="utf8mb4"
```

当前 `tasks` 表核心字段包括：

- `task_id`
- `user_id`
- `name`
- `status`
- `create_time`
- `start_time`
- `finish_time`
- `batch_config_path`
- `request_json`
- `log_dir`
- `output_dir`
- `message`
- `exit_code`
- `cancel_requested`
