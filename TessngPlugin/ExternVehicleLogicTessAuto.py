# pythonCode
# -*- coding: utf-8 -*-
# @Time : 2025/6/16 14:23
# @Author : cyz
# @File : ExternVehicleLogicTessAuto.py
# @Email : cyz.sh@foxmail.com


from typing import Dict

from AutoPilot.Player.VehicleState import VehicleState
from Tessng import TessAutonomous


class TessAutoPyInterface(object):
	def __init__(self, tessngIface):
		self.iface = tessngIface

		# avName → VehicleState
		self.avChannel2AvMsgMap: Dict[str, VehicleState] = dict()

		self.avName2TessngIdMap = dict()
		self.tessngId2AvNameMap = dict()

		self.alreadyLaunchedAvIdSet = set()
		self.alreadyLaunchedAvNameSet = set()
		self.alreadyLaunchedTessngIdSet = set()

		self.mainVehiclePtrDict = dict()

		self._removeAvNames = []

	def Stop(self):
		self.avChannel2AvMsgMap.clear()
		self.avName2TessngIdMap.clear()
		self.tessngId2AvNameMap.clear()

		self.alreadyLaunchedAvIdSet.clear()
		self.alreadyLaunchedAvNameSet.clear()
		self.alreadyLaunchedTessngIdSet.clear()

		self.mainVehiclePtrDict.clear()

		self._removeAvNames.clear()

	# ================================================================
	#  写入数据
	# ================================================================

	def setAvChannel2AvMsgMap(self, vehicle_states: Dict[str, VehicleState]):
		"""
		更新自动驾驶车辆状态

		Args:
			vehicle_states: 选手名 → VehicleState 的字典
		"""
		for name, state in vehicle_states.items():
			self.avChannel2AvMsgMap[name] = state

	# ================================================================
	#  构建 Tessng 状态结构
	# ================================================================

	@staticmethod
	def buildVehicleStatus(avName: str, state: VehicleState):
		"""从 VehicleState 构建 Tessng 创建车辆所需的 VehiStatus"""
		vs = TessAutonomous.VehiStatus()
		autoType = TessAutonomous.AutoVehiType()

		vs.vehiStableStatus.autoType = autoType.AV
		vs.vehiStableStatus.typeCode = 1
		vs.vehiStableStatus.length = state.length / 100
		vs.vehiStableStatus.width = state.width / 100
		vs.vehiStableStatus.height = state.height / 100

		vs.vehiDynaStatus.x = state.x
		vs.vehiDynaStatus.y = -state.y
		vs.vehiDynaStatus.speed = state.speed
		vs.vehiDynaStatus.angle = state.heading

		return vs

	@staticmethod
	def buildVehicleDynaStatus(state: VehicleState):
		"""从 VehicleState 构建 Tessng 更新车辆所需的 VehiDynaStatus"""
		vds = TessAutonomous.VehiDynaStatus()

		vds.x = state.x
		vds.y = -state.y
		vds.speed = state.speed
		vds.angle = state.heading

		return vds

	# ================================================================
	#  创建 / 更新车辆
	# ================================================================

	def createTessngAutoAv(self, avName: str, state: VehicleState):
		"""在 Tessng 中创建一辆自动驾驶车辆"""
		autoInterface = self.iface.autoInterface()
		if not autoInterface:
			print(f"[ERROR] failed to get auto interface, autoInterface is {autoInterface}")
			return -1

		if avName in self.alreadyLaunchedAvNameSet:
			return 0

		vs = self.buildVehicleStatus(avName, state)

		avVehiclePtr = autoInterface.createExternalVehicle(vs)
		print(f"avName {avName}, try to create av {avVehiclePtr.get()}, point is [{state.x}, {-state.y}]")
		
		if avVehiclePtr:
			mainVehi = avVehiclePtr.get()
			mainVehiId = mainVehi.id()

			if avName == "ego":
				veh = avVehiclePtr.getVehicle()
				veh.setColor("#02f13e")
				print(f"avName {avName}, set color to #fc0703")

			self.alreadyLaunchedTessngIdSet.add(mainVehiId)
			self.alreadyLaunchedAvNameSet.add(avName)
			self.avName2TessngIdMap[avName] = mainVehiId
			self.tessngId2AvNameMap[mainVehiId] = avName
			self.mainVehiclePtrDict[avName] = avVehiclePtr

			print(f"avName->{avName}, avTessngId->{mainVehiId}, succeed to create av, point is [{state.x}, {-state.y}]")
			self.alreadyLaunchedAvIdSet.add(mainVehiId)
			return 0
		else:
			print(f"avName {avName}, failed to create av, point is [{state.x}, {-state.y}]")

		return -1

	def updateTessngAutoAv(self, avName: str, state: VehicleState, pIVehicle=None):
		"""更新 Tessng 中已有车辆的动态状态"""
		autoInterface = self.iface.autoInterface()

		if avName not in self.mainVehiclePtrDict:
			return

		avVehiclePtr = self.mainVehiclePtrDict[avName]
		if not avVehiclePtr:
			return

		mainVehi = avVehiclePtr.get()
		iVehi = mainVehi.getVehicle()
		iVehi.setJsonProperty("name", avName)
		mainVehiId = mainVehi.id()

		if iVehi.isStarted() and mainVehiId == pIVehicle.id():
			vds = self.buildVehicleDynaStatus(state)
			ok = autoInterface.updateExternalVehicle(mainVehiId, vds)
			if not ok:
				print(f"update failed, id: {mainVehiId}, pos: [{vds.x}, {vds.y}]")
				return False
			return True

	# ================================================================
	#  批量操作
	# ================================================================

	def vehicleCreateAndUpdate(self, pIVehicle=None):
		
		for avName, state in self.avChannel2AvMsgMap.items():
			# if avName in self._removeAvNames:
			# 	continue
			result = self.createTessngAutoAv(avName, state)
			self.updateTessngAutoAv(avName, state, pIVehicle)
			if result < 0:
				self._removeAvNames.append(avName)

	def vehicleCreate(self):
		for avName, state in self.avChannel2AvMsgMap.items():
			# if avName in self._removeAvNames:
			# 	continue
			createSuccess = self.createTessngAutoAv(avName, state)
			if createSuccess < 0:
				self._removeAvNames.append(avName)
		

	def vehicleUpdate(self, pIVehicle=None):
		for avName, state in self.avChannel2AvMsgMap.items():
			self.updateTessngAutoAv(avName, state, pIVehicle)
