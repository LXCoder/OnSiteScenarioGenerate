# pythonCode
# -*- coding: utf-8 -*-
# @Time : 2024/10/14 9:56
# @Author : cyz
# @File : AutoSelfDrivingFactory.py
# @Email : cyz.sh@foxmail.com

import time

from utils.RequestLogger import logger

from PySide2.QtCore import QPointF


class Point3D:
	def __init__(self, x=0.0, y=0.0, z=0.0):
		self.x = x
		self.y = y
		self.z = z


class AutoSelfDrivingMsg:
	def __init__(self, id, name, position: Point3D, speed, timestamp, length, width, height, angle, color):
		self._id = id
		self._name = name
		self._position = position
		self._speed = speed
		self._timestamp = timestamp

		self._length = length
		self._width = width
		self._height = height

		self._angle = angle
		self._color = color

	@property
	def id(self):
		return self._id

	@id.setter
	def id(self, value):
		self._id = value

	@property
	def name(self):
		return self._name

	@name.setter
	def name(self, value):
		self._name = value

	@property
	def position(self):
		return self._position

	@position.setter
	def position(self, value):
		self._position = value

	@property
	def speed(self):
		return self._speed

	@speed.setter
	def speed(self, value):
		self._speed = value

	@property
	def timestamp(self):
		return self._timestamp

	@timestamp.setter
	def timestamp(self, value):
		self._timestamp = value

	@property
	def length(self):
		return self._length
	@length.setter
	def length(self, value):
		self._length = value

	@property
	def width(self):
		return self._width
	@width.setter
	def width(self, value):
		self._width = value
	@property
	def height(self):
		return self._height
	@height.setter
	def height(self, value):
		self._height = value

	@property
	def angle(self):
		return self._angle
	@angle.setter
	def angle(self, value):
		self._angle = value
	@property
	def color(self):
		return self._color
	@color.setter
	def color(self, value):
		self._color = value


class AutoSelfDrivingFactory:
	def __init__(self):
		# 对应创建的tessng车辆类型
		self.tessngVehicleTypeCode = 1

		# 已经被成功创建的av数据的id和name
		self.alreadyLaunchedAvIdSet = set()
		self.alreadyLaunchedAvNameSet = set()
		self.alreadyLaunchedTessngIdSet = set()

		# 创建的av车辆id对应的tessng车辆id表
		self.avName2TessngIdMap = dict()

		# Tessng车辆Id对应的Av车辆Id表
		self.tessngId2AvNameMap = dict()

		# 由Av数据创建的Tessng车辆的时间戳表
		self.tessngVehicleIdTimestampMap = dict()

		# av数据通道和对应的av数据表
		self.avChannel2AvMsgMap = dict()

	def setAvChannel2AvMsgMap(self, avData: dict):
		"""
		外部车数据 {'type': 'trajectory', 'GKQResult': {'timestamp': 1713425513.7773476, 'value': {'主车1': {'frameId': 163, 'speed': 10.189054584503173, 'courseAngle': 140.33428055084798, 'tessngPos': [-660.1005150601349, -22.674477475149107], 'length': 476, 'width': 190}}}}

		Args:
			avData: 'value': {'主车1': {'frameId': 163, 'speed': 10.189054584503173, 'courseAngle': 140.33428055084798, 'tessngPos': [-660.1005150601349, -22.674477475149107], 'length': 476, 'width': 190}}}

		Returns: bool

		"""

		# av车的数据通道
		# avName = list(avData.keys())[1]
		# value = avData.get(avName).get("value")
		# valueKey = list(value.keys())[0]
		#
		# avValue = value.get(valueKey)
		#
		# tessngPos = avValue.get("tessngPos")
		# position = Point3D(tessngPos[0], tessngPos[1])
		# speed = avValue.get("speed")
		#
		# msg = AutoSelfDrivingMsg(avName, avName, position, speed, int(time.time()))
		#
		# self.avChannel2AvMsgMap[avName] = msg

		for avName, avValue in avData.items():
			if avName != "type":
				# av车的数据通道
				value = avData.get(avName).get("value")
				if not value:
					continue
				valueKey = list(value.keys())[0]

				avValue = value.get(valueKey)

				tessngPos = avValue.get("tessngPos")
				position = Point3D(tessngPos[0], tessngPos[1])
				speed = avValue.get("speed")
				length = avValue.get("length")
				width = avValue.get("width")
				height = 150
				angle = avValue.get("courseAngle")
				color = avValue.get("color", "#00BFFF")

				msg = AutoSelfDrivingMsg(avName, avName, position, speed, int(time.time()), length, width, height, angle, color)

				self.avChannel2AvMsgMap[avName] = msg

	def createAv(self, avMsg, simuInterface, netInterface):
		location = None

		avName = avMsg.name
		if avName in self.alreadyLaunchedAvNameSet:
			return True

		avMsg = self.avChannel2AvMsgMap.get(avName)
		pos = QPointF(avMsg.position.x, -avMsg.position.y)
		locations = netInterface.locateOnCrid(pos, 9)

		if locations:
			location = locations[0]

		currentTimestamp = int(time.time())
		dt = abs(currentTimestamp - avMsg.timestamp)

		if avName not in self.alreadyLaunchedAvNameSet and dt <= 1:
			vehi = None
			if location:
				from Tessng import Online
				dvp = Online.DynaVehiParam()

				dvp.vehiTypeCode = self.tessngVehicleTypeCode
				dvp.speed = avMsg.speed
				dvp.color = avMsg.color
				dvp.dist = location.distToStart
				target = location.pLaneObject
				# 如果是路段
				if target.isLane():
					lane = target.castToLane()
					dvp.roadId = lane.link().id()
					dvp.laneNumber = lane.number()
				# 如果是连接段
				else:
					lane_connector = target.castToLaneConnector()
					dvp.roadId = lane_connector.connector().id()
					dvp.laneNumber = lane_connector.fromLane().number()
					dvp.toLaneNumber = lane_connector.toLane().number()
				vehi = simuInterface.createGVehicle(dvp)

			if vehi:
				tessngVehicleId = vehi.id()
				# 由外部数据创建的tessng车辆数据不发
				self.alreadyLaunchedTessngIdSet.add(tessngVehicleId)
				# vehi.setTextTag("av")
				# 记录
				self.alreadyLaunchedAvNameSet.add(avName)
				self.avName2TessngIdMap[avName] = tessngVehicleId
				self.tessngId2AvNameMap[tessngVehicleId] = avName
				self.tessngVehicleIdTimestampMap[tessngVehicleId] = currentTimestamp

			return False
		else:
			return True

	def moveAv(self, pIVehicle, avMsg, netInterface):
		locations = []

		avName = avMsg.name
		tessngVehicleId = self.avName2TessngIdMap.get(avName)

		if pIVehicle.id() == tessngVehicleId:
			pos = QPointF(avMsg.position.x, -avMsg.position.y)
			locations = netInterface.locateOnCrid(pos, 9)

		if not locations:
			return False

		tempLaneId = pIVehicle.lane().id() if pIVehicle.roadIsLink() else pIVehicle.laneConnector().fromLane().id()

		targetLocation = None
		for location in locations:
			lane_object = location.pLaneObject
			if lane_object.length() < 0.1:
				continue
			if lane_object.isLane():
				targetLocation = location
				break
			else:
				loc_cast_to_lc = lane_object.castToLaneConnector()
				if loc_cast_to_lc and loc_cast_to_lc.fromLane().id() == tempLaneId:
					targetLocation = location
					break

		targetLocation = targetLocation or locations[0]

		if targetLocation:
			laneObj = targetLocation.pLaneObject
			pIVehicle.vehicleDriving().move(laneObj, targetLocation.distToStart)
			self.tessngVehicleIdTimestampMap[pIVehicle.id()] = int(time.time())
			return True
		else:
			return False

		# if locations:
		# 	location = locations[0]
		# 	laneObj = location.pLaneObject
		# 	pIVehicle.vehicleDriving().move(laneObj, location.distToStart)
		# 	self.tessngVehicleIdTimestampMap[pIVehicle.id()] = int(time.time())
		# 	return True
		# else:
		# 	return False

	def deleteAv(self, pIVehicle, avMsg):
		tessngId = pIVehicle.id()
		avName = avMsg.name

		currentTimestamp = self.tessngVehicleIdTimestampMap.get(tessngId)
		avTimestamp = avMsg.timestamp
		dt = abs(currentTimestamp - avTimestamp)

		if dt >= 2:
			pIVehicle.vehicleDriving().stopVehicle()
			self.alreadyLaunchedTessngIdSet.remove(tessngId)
			self.alreadyLaunchedAvNameSet.remove(avName)
			self.avName2TessngIdMap.pop(avName)
			self.tessngId2AvNameMap.pop(tessngId)
			self.tessngVehicleIdTimestampMap.pop(tessngId)
			logger.info(f"{avName} delete success")
			return True

		return False

	def popAvTessngMap(self, avName, avTessngId):
		self.alreadyLaunchedAvNameSet.remove(avName)
		self.alreadyLaunchedTessngIdSet.remove(avTessngId)
		self.avName2TessngIdMap.pop(avName)
		self.tessngId2AvNameMap.pop(avTessngId)
		self.tessngVehicleIdTimestampMap.pop(avTessngId)

	def deleteCache(self):
		self.alreadyLaunchedAvNameSet.clear()
		self.alreadyLaunchedTessngIdSet.clear()
		self.avName2TessngIdMap.clear()
		self.tessngId2AvNameMap.clear()
		self.tessngVehicleIdTimestampMap.clear()

	def vehicleCreateAndMove(self, lAllVehicleList, simuInterface, netInterface):
		tessngVehicleIdList = [vehi.id() for vehi in lAllVehicleList]

		for avTessngId in self.alreadyLaunchedTessngIdSet:
			if avTessngId not in tessngVehicleIdList:
				avName = self.tessngId2AvNameMap.get(avTessngId)
				if avName in self.alreadyLaunchedAvNameSet:
					self.alreadyLaunchedAvNameSet.remove(avName)

		for avName, avMsg in self.avChannel2AvMsgMap.items():
			createSuccess = self.createAv(avMsg, simuInterface, netInterface)
			# logger.info(f"{avName} create success {createSuccess}")

		for vehicle in lAllVehicleList:
			tessngId = vehicle.id()
			avName = self.tessngId2AvNameMap.get(tessngId)
			avMsg = self.avChannel2AvMsgMap.get(avName)

			if avMsg:
				self.deleteAv(vehicle, avMsg)
				self.moveAv(vehicle, avMsg, netInterface)
