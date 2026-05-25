FROM jida-inspur-4:2443/library/tess_auto:1.0

WORKDIR /app

COPY requirements.txt .

RUN pip install torch==1.10.2 -i https://mirrors.aliyun.com/pypi/simple -vv
RUN pip install -r requirements.txt -i https://mirrors.aliyun.com/pypi/simple -vv

COPY . .

ENV LD_LIBRARY_PATH=/app/TessngLib
ENV QT_LOGGING_RULES="*.debug=false;*.info=true;qt.widgets.painting=false"

CMD [ "bash" ]

