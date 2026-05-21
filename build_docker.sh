#!/bin/bash

# 1. 获取传入的参数 $1，如果 $1 为空，则使用 date +%Y%m%d 作为默认值
TAG=${1:-$(date +%Y%m%d)}

# 2. 拼接完整的镜像名称
IMAGE_NAME="jida-inspur-4:2443/library/tess_auto:${TAG}"

# 3. 打印一下提示信息，方便确认
echo "========================================="
echo "准备构建镜像: ${IMAGE_NAME}"
echo "========================================="

# 4. 执行 docker build
docker build -t "${IMAGE_NAME}" .

# 5. 可选：构建成功后打印推送命令提示
if [ $? -eq 0 ]; then
    echo "✅ 构建成功！如需推送，请执行: docker push ${IMAGE_NAME}"
else
    echo "❌ 构建失败，请检查日志"
fi
