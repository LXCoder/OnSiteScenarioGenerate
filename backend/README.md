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
- `BACKEND_USE_DOCKER` default `false`
- `BACKEND_DOCKER_IMAGE` Docker image used by the worker when Docker mode is enabled
- `BACKEND_TASK_TIMEOUT` default `3600`

## API

### `POST /tasks/create`

创建任务。支持两种输入方式：

1. `application/json`
2. `multipart/form-data`，可同时上传文件

JSON 请求体示例：

```json
{
  "name": "demo-task",
  "batch_items": [
    {
      "name": "scene-a",
      "NET_PATH": "Data/scenario_0a6bf824.tess",
      "BG_MODEL_FILENAME": "model.zip.v6",
      "DATA_DIR": "Data/test",
      "USE_TEST_LOGIC": true
    }
  ]
}
```

兼容字段：

- `batch_items`
- `batch_config`
- `task`

如果都不传，会把除 `name` 外的其他字段当作单个任务项。

成功响应：`201 Created`

```json
{
  "task_id": "task_20260518123045_a1b2c3",
  "name": "demo-task",
  "status": "PENDING",
  "create_time": "2026-05-18T12:30:45.123456+00:00",
  "start_time": null,
  "finish_time": null,
  "batch_config_path": ".../backend/task_data/task_.../batch_config.json",
  "request_json": ".../backend/task_data/task_.../request.json",
  "log_dir": ".../backend/task_data/task_.../logs",
  "output_dir": ".../backend/task_data/task_.../output",
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

### `GET /tasks`

查询任务列表。

成功响应：`200 OK`

```json
{
  "count": 1,
  "items": [
    {
      "task_id": "task_20260518123045_a1b2c3",
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
- `tail`，返回最后多少行，默认 `200`

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

Runtime data is stored under `backend/task_data/`.
