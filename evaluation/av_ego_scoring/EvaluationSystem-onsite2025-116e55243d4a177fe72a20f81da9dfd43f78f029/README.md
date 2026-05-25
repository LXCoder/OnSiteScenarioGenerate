### ProjectName

EvaluationSystem



### 配置环境

python version: 3.6.8
packages详见requirements.txt



### 文件目录

```
filetree 
├── main.py：运行在线接口
├── offline_evaluate: 运行离线评价
├── /utils/：工程工具
├── /opendrive2tessng/：场景地图处理
├── /evaluateUtils/：评价计算
│  ├── /Config/
│  ├── /MySource/
│  ├── /TrafficRule_compliance/：交规符合性计算工具
│  │  ├── /KnowledgeBase/
│  │  │  ├── china.csv：交规
│  │  │  ├── lefthand_flag.csv：左行场景清单
│  ├── standard_parameter.py：评价相关阈值及各分项权重等参数
├── /testData/：测试数据
│  ├── /trajectory/：轨迹文件目录
│  ├── /scenario/：场景文件目录
│  │  ├── /replay/：场景类型
├── /Data/：临时文件存储，离线评价未使用
├── README.md
├── requirements.txt
```



### 操作指南
```
①用户在testData/trajectory文件夹中放入待测轨迹文件，在testData/scenario/...(轨迹对应的测试类型)文件夹中放入轨迹对应的待测场景文件
②更新左行场景清单evaluateUtils/TrafficRule_compliance/KnowledgeBase/lefthand_flag.csv
③运行offline_evaluate.py
```


