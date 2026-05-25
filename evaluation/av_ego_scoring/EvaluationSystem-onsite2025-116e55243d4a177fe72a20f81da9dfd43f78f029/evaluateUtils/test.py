import json

from kafka import KafkaConsumer, TopicPartition

# # 内部kafka的地址和端口
# KAFKA_HOST = "129.211.28.237"
# KAFKA_PORT = 29092
# todo 0925畅行演示kafka服务器
KAFKA_HOST = "202.120.189.56"
KAFKA_PORT = 5011

# KAFKA_HOST = "106.120.201.126"
# KAFKA_PORT = 19359
# topic = 'TJTestSceneData'  # 测试场景状态数据
# topic = 'TJAutoMatchResultMiniData'  # 自动驾驶数据
# topic = 'TJ002AllMatchResultMiniData'  # 仿真全量轨迹数据
# topic = 'TJSimplifyMatchResultMiniData'  # 自动驾驶车辆周边场景仿真交通流数据

send_topic_mapping = {
    "all": "TJ002AllMatchResultMiniData",  # 仿真全量轨迹数据
    "simplify": "TJSimplifyMatchResultMiniData",  # 自动驾驶车辆周边场景仿真交通流数据
    "auto": "TJAutoMatchResultMiniData",  # 自动驾驶数据
    "scene": "TJTestSceneData",  # 测试场景状态数据
    "evaluate": "TJTestEvaluateData",  # 仿真评价数据
    "sceneIscore": "TJTestSceneIscore",  # 单个场景的指标数据
    "totalScore": "TJTestTotalScore",  # 全部场景的指标数据累积
    "TessTrajectory": "TessTrajectoryData"  # 轨迹数据
}


consumer = KafkaConsumer(
            bootstrap_servers=[f'{KAFKA_HOST}:{KAFKA_PORT}'],
            api_version=(0, 10, 1),
            # request_timeout_ms=3000,
        )
tp = TopicPartition(send_topic_mapping['totalScore'], 0)
consumer.assign([tp])
consumer.seek_to_end(tp)
for message in consumer:
    print(json.loads(message.value))


# {
# 	'startTime': '2023-09-22 14:55:41.048',
# 	'endTime': '2023-09-22 14:57:11.337',
# 	'testDuration': 91.2,
# 	'dangerTimeProportion': 0.0,
# 	'allSenseScore': 100.0,
# 	'testScene': [
#         {
# 		'senceID': 3,
# 		'missionAccomplish': 1,
# 		'senseScore': 100.0,
# 		'senseScore*senseWeight': 20.0,
# 		'senseAggScore': 20.0,
# 		'senseWeight': 20.0,
# 		'info': {
# 			'efficiency': [{
# 				'index': '任务完成',
# 				'score': '15/15',
# 				'time': 0
# 			}, {
# 				'index': '任务耗时',
# 				'score': '15/15',
# 				'time': 13.0,
# 				'expectTime':10,
# 				'overTime': 0.3
# 			}],
# 			'comfortable': [{
# 				'index': '横向舒适度',
# 				'score': '-0.0',
# 				'type': 'a',
# 				'time': 0.0
# 			}, {
# 				'index': '横向舒适度',
# 				'score': '-0.0',
# 				'type': 'j',
# 				'time': 0.0
# 			}, {
# 				'index': '纵向舒适度',
# 				'score': '-0.0',
# 				'type': 'a',
# 				'time': 0.0
# 			}, {
# 				'index': '纵向舒适度',
# 				'score': '-0.0',
# 				'type': 'j',
# 				'time': 0.0
# 			}, {
# 				'index': '转弯舒适度',
# 				'score': '-0.0',
# 				'type': 'a',
# 				'time': 0.0
# 			}, {
# 				'index': '转弯舒适度',
# 				'score': '0',
# 				'type': 'j',
# 				'time': 0
# 			}],
# 			'safe': [{
# 				'index': '碰撞',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '驶出行车道',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': 'TTC',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '驶入对向车道',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '压实线',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '超速',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '闯红灯',
# 				'score': '-0',
# 				'time': 0
# 			}]
# 		}
# 	},
#         {
# 		'senceID': 4,
# 		'missionAccomplish': 1,
# 		'senseScore': 100.0,
# 		'senseScore*senseWeight': 30.0,
# 		'senseAggScore': 30.0,
# 		'senseWeight': 30.0,
# 		'info': {
# 			'efficiency': [{
# 				'index': '任务完成',
# 				'score': '15/15',
# 				'time': 0
# 			}, {
# 				'index': '任务耗时',
# 				'score': '15/15',
# 				'time': 19.133000000000003,
# 				'expectTime':15,
# 				'overTime': 0.27
# 			}],
# 			'comfortable': [{
# 				'index': '横向舒适度',
# 				'score': '-0.0',
# 				'type': 'a',
# 				'time': 0.0
# 			}, {
# 				'index': '横向舒适度',
# 				'score': '-0.0',
# 				'type': 'j',
# 				'time': 0.0
# 			}, {
# 				'index': '纵向舒适度',
# 				'score': '-0.0',
# 				'type': 'a',
# 				'time': 0.0
# 			}, {
# 				'index': '纵向舒适度',
# 				'score': '-0.0',
# 				'type': 'j',
# 				'time': 0.0
# 			}, {
# 				'index': '转弯舒适度',
# 				'score': '-0.0',
# 				'type': 'a',
# 				'time': 0.0
# 			}, {
# 				'index': '转弯舒适度',
# 				'score': '0',
# 				'type': 'j',
# 				'time': 0
# 			}],
# 			'safe': [{
# 				'index': '碰撞',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '驶出行车道',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': 'TTC',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '驶入对向车道',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '压实线',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '超速',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '闯红灯',
# 				'score': '-0',
# 				'time': 0
# 			}]
# 		}
# 	},
#         {
# 		'senceID': 5,
# 		'missionAccomplish': 1,
# 		'senseScore': 100.0,
# 		'senseScore*senseWeight': 20.0,
# 		'senseAggScore': 20.0,
# 		'senseWeight': 20.0,
# 		'info': {
# 			'efficiency': [{
# 				'index': '任务完成',
# 				'score': '15/15',
# 				'time': 0
# 			}, {
# 				'index': '任务耗时',
# 				'score': '15/15',
# 				'time': 10.133000000000003,
# 				'expectTime':10,
# 				'overTime': 0.0
# 			}],
# 			'comfortable': [{
# 				'index': '横向舒适度',
# 				'score': '-0.0',
# 				'type': 'a',
# 				'time': 0.0
# 			}, {
# 				'index': '横向舒适度',
# 				'score': '-0.0',
# 				'type': 'j',
# 				'time': 0.0
# 			}, {
# 				'index': '纵向舒适度',
# 				'score': '-0.0',
# 				'type': 'a',
# 				'time': 0.0
# 			}, {
# 				'index': '纵向舒适度',
# 				'score': '-0.0',
# 				'type': 'j',
# 				'time': 0.0
# 			}, {
# 				'index': '转弯舒适度',
# 				'score': '-0.0',
# 				'type': 'a',
# 				'time': 0.0
# 			}, {
# 				'index': '转弯舒适度',
# 				'score': '0',
# 				'type': 'j',
# 				'time': 0
# 			}],
# 			'safe': [{
# 				'index': '碰撞',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '驶出行车道',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': 'TTC',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '驶入对向车道',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '压实线',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '超速',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '闯红灯',
# 				'score': '-0',
# 				'time': 0
# 			}]
# 		}
# 	},
#         {
# 		'senceID': 6,
# 		'missionAccomplish': 1,
# 		'senseScore': 100.0,
# 		'senseScore*senseWeight': 30.0,
# 		'senseAggScore': 30.0,
# 		'senseWeight': 30.0,
# 		'info': {
# 			'efficiency': [{
# 				'index': '任务完成',
# 				'score': '15/15',
# 				'time': 0
# 			}, {
# 				'index': '任务耗时',
# 				'score': '15/15',
# 				'time': 38.0,
# 				'expectTime':30,
# 				'overTime': 0.26
# 			}],
# 			'comfortable': [{
# 				'index': '横向舒适度',
# 				'score': '-0.0',
# 				'type': 'a',
# 				'time': 0.0
# 			}, {
# 				'index': '横向舒适度',
# 				'score': '-0.0',
# 				'type': 'j',
# 				'time': 0.0
# 			}, {
# 				'index': '纵向舒适度',
# 				'score': '-0.0',
# 				'type': 'a',
# 				'time': 0.0
# 			}, {
# 				'index': '纵向舒适度',
# 				'score': '-0.0',
# 				'type': 'j',
# 				'time': 0.0
# 			}, {
# 				'index': '转弯舒适度',
# 				'score': '-0.0',
# 				'type': 'a',
# 				'time': 0.0
# 			}, {
# 				'index': '转弯舒适度',
# 				'score': '0',
# 				'type': 'j',
# 				'time': 0
# 			}],
# 			'safe': [{
# 				'index': '碰撞',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '驶出行车道',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': 'TTC',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '驶入对向车道',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '压实线',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '超速',
# 				'score': '-0.0',
# 				'time': 0.0
# 			}, {
# 				'index': '闯红灯',
# 				'score': '-0',
# 				'time': 0
# 			}]
# 		}
# 	}],
#     'diagnose': {
# 		'安全性方面': '安全性方面，取得良好成绩，符合驾驶规范，可保证行车安全。',
# 		'舒适性方面': '舒适性方面，取得良好成绩，无不良驾驶行为，乘坐舒适度较高。',
# 		'交互决策方面': '交互决策保守，拟人程度低，导致通过场景时间较长，效率指标良',
# 		'建议': '建议提升驾驶策略，增加域控制器算法的激进程度，主动与其他车辆进行交互。'
# 	}
# }