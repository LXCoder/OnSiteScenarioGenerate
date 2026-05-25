from typing import Tuple, Optional, Callable, Dict, List
import numpy as np
import cvxpy

Scenario = Dict[str, float]


class Barrier():
    # dimension
    N_OBJ = 3

    # index
    OBJ_X = 0
    OBJ_Y = 1
    OBJ_V = 2

    def __init__(
        self,
        dt: float,
        obj_param,
        key: str,
        ) -> None:
        self.dt = dt

        self.obj_x = obj_param.x
        self.obj_y = obj_param.y
        self.obj_speed = obj_param.v
        self.obj_yaw = obj_param.yaw
        self.obj_a = obj_param.a
                
        self.obj_length = obj_param.length
        self.obj_width = obj_param.width
        self.a = (np.sqrt(self.obj_length ** 2 + (self.obj_width / 2.0) ** 2) + self.obj_width / 2.0) / 2.0
        self.b = np.sqrt(self.a ** 2 - (self.obj_length / 2.0) ** 2)

        self.name = key

    @property
    def n_obj(self) -> int:
        return Barrier.N_OBJ

    def cal(
        self,
        state: np.array,
        ego_length: float,
        ego_width: float,
        safe_distance: Optional[float] = None,
        ) -> None:
        self.h = self._get_h(state, ego_length, ego_width, safe_distance)

        self.dh_dstate = np.zeros((1, VehicleModel.N_DIMS))
        self.dh_dx, self.dh_dy = self._get_dh_dstate()
        self.dh_dstate[0, VehicleModel.X_GLOBAL] = self.dh_dx
        self.dh_dstate[0, VehicleModel.Y_GLOBAL] = self.dh_dy

        self.dh_dobj = np.zeros((Barrier.N_OBJ, 1))
        self.dh_dobj[Barrier.OBJ_X, 0] = - self.dh_dx
        self.dh_dobj[Barrier.OBJ_Y, 0] = - self.dh_dy

        self.dobj_dt = np.zeros((1, Barrier.N_OBJ))
        self.dobj_dt[0, Barrier.OBJ_X] = self.obj_speed * np.cos(self.obj_yaw)
        self.dobj_dt[0, Barrier.OBJ_Y] = self.obj_speed * np.sin(self.obj_yaw)
        self.dobj_dt[0, Barrier.OBJ_V] = self.obj_a

        self.ddh_dx_dx, self.ddh_dx_dy, self.ddh_dy_dy = self._get_ddh_dstate_dstate()
        self.ddh_dstate_dobj = np.zeros((Barrier.N_OBJ, VehicleModel.N_DIMS))
        self.ddh_dstate_dobj[Barrier.OBJ_X, VehicleModel.X_GLOBAL] = - self.ddh_dx_dx
        self.ddh_dstate_dobj[Barrier.OBJ_X, VehicleModel.Y_GLOBAL] = - self.ddh_dx_dy
        self.ddh_dstate_dobj[Barrier.OBJ_Y, VehicleModel.X_GLOBAL] = - self.ddh_dx_dy
        self.ddh_dstate_dobj[Barrier.OBJ_Y, VehicleModel.Y_GLOBAL] = - self.ddh_dy_dy

        self.ddhobj_dt_dobj = np.zeros((Barrier.N_OBJ, 1))
        self.ddhobj_dt_dobj[Barrier.OBJ_X, 0] = self.ddh_dx_dx * self.obj_speed * np.cos(self.obj_yaw) + self.ddh_dx_dy * self.obj_speed * np.sin(self.obj_yaw)
        self.ddhobj_dt_dobj[Barrier.OBJ_Y, 0] = self.ddh_dx_dy * self.obj_speed * np.cos(self.obj_yaw) + self.ddh_dy_dy * self.obj_speed * np.sin(self.obj_yaw)
        self.ddhobj_dt_dobj[Barrier.OBJ_V, 0] = - self.dh_dx * np.cos(self.obj_yaw) - self.dh_dy * np.sin(self.obj_yaw)

        # dh_dobj * dobj_dt
        self.dhobj_dt = np.dot(self.dobj_dt, self.dh_dobj)
        # ddh_dstate_dobj * dobj_dt
        self.ddhobj_dstate_dt = np.dot(self.dobj_dt, self.ddh_dstate_dobj)
        # dobj_dt * ddh_dobj_dobj * dobj_dt
        self.ddhobjobj_dt_dt = np.dot(self.dobj_dt, self.ddhobj_dt_dobj)

    def _get_h(
        self, 
        state: np.array, # system state
        ego_length: float,
        ego_width: float,
        safe_distance: Optional[float] = None,
        ) -> float:
        x_g = state[VehicleModel.X_GLOBAL, 0]
        y_g = state[VehicleModel.Y_GLOBAL, 0]
        phi = state[VehicleModel.PHI, 0]
        v_lon = state[VehicleModel.V_LON, 0]

        self.delta_x = - self.obj_x + x_g
        self.delta_y = - self.obj_y + y_g
        # theta is considered ubchanged during {dt} time
        theta = np.arctan2( - self.delta_y,  - self.delta_x) - self.obj_yaw 
        l = self.a * self.b / np.sqrt(self.a ** 2 * np.sin(theta) ** 2 + self.b ** 2 * np.cos(theta) ** 2)
        self.dis = np.sqrt((self.delta_x) ** 2 + (self.delta_y) ** 2)

        diff_x = - self.delta_x * np.cos(phi) - self.delta_y * np.sin(phi)
        if v_lon > self.obj_speed * np.cos(self.obj_yaw - phi) and diff_x > 0:
            self.status = "Front"
        elif v_lon < self.obj_speed * np.cos(self.obj_yaw - phi) and diff_x < 0:
            self.status = "Behind"
        else:
            self.status = "Other"

        if safe_distance is None:
            lon = np.sqrt(ego_length ** 2 + (ego_width / 2.0) ** 2)
            lat = ego_width / 2.0
            # theta0 is considered ubchanged during {dt} time
            theta0 = np.arctan2(self.delta_y, self.delta_x) - phi
            safe_distance = lon * lat / np.sqrt(lon ** 2 * np.sin(theta0) ** 2 + lat ** 2 * np.cos(theta0) ** 2) + 1.0

        h = self.dis - l - safe_distance
        return h

    def _get_dh_dstate(self) -> Tuple[float, float]:
        # dh/dx, dh/dy
        return self.delta_x / self.dis, self.delta_y / self.dis

    def _get_ddh_dstate_dstate(self) -> Tuple[float, float, float]:
        # ddh/dxx, ddh/dxy, ddh/dyy
        dis_tri = self.dis ** 3
        return self.delta_y ** 2 / dis_tri, - self.delta_x * self.delta_y / dis_tri, self.delta_x ** 2 / dis_tri


class VehicleModel():
    

    # number of state and control
    N_DIMS = 4
    N_CONTROL = 2

    # state
    X_GLOBAL = 0
    Y_GLOBAL = 1
    V_LON = 2
    PHI = 3

    # control
    A_LON = 0
    TAN_STEER = 1

    def __init__(
        self, 
        param,
        ) -> None:    
        self.ego_length = param.length
        self.ego_width = param.width
        self.L = self.ego_length / 1.7

    @property
    def n_dim(self) -> int:
        return VehicleModel.N_DIMS

    @property
    def n_control(self) -> int:
        return VehicleModel.N_CONTROL
    
    def get_dynamic(
        self,
        state: np.array,
        ) -> Tuple[np.array, np.array]:
        return self._f(state), self._g(state)

    def _f(
        self,
        state: np.array,
        ) -> np.array:
        f = np.zeros((self.n_dim, 1))

        v_lon = state[VehicleModel.V_LON, 0]
        phi = state[VehicleModel.PHI, 0]

        f[VehicleModel.X_GLOBAL, 0] = v_lon * np.cos(phi)
        f[VehicleModel.Y_GLOBAL, 0] = v_lon * np.sin(phi)
        f[VehicleModel.V_LON, 0] = 0
        f[VehicleModel.PHI, 0] = 0
        return f
    
    def _g(
        self,
        state: np.array,
        ) -> np.array:
        g = np.zeros((self.n_dim ,self.n_control))

        v_lon = state[VehicleModel.V_LON, 0]

        g[VehicleModel.X_GLOBAL, VehicleModel.A_LON] = 0.0
        g[VehicleModel.Y_GLOBAL, VehicleModel.A_LON] = 0.0
        g[VehicleModel.V_LON, VehicleModel.A_LON] = 1.0
        g[VehicleModel.PHI, VehicleModel.A_LON] = 0.0

        # g[VehicleModel.X_GLOBAL, VehicleModel.TAN_STEER] = - v_lon * np.sin(phi) * self.l_r / (self.l_r + self.l_f)
        # g[VehicleModel.Y_GLOBAL, VehicleModel.TAN_STEER] = v_lon * np.cos(phi) * self.l_r / (self.l_r + self.l_f)
        g[VehicleModel.X_GLOBAL, VehicleModel.TAN_STEER] = 0.0
        g[VehicleModel.Y_GLOBAL, VehicleModel.TAN_STEER] = 0.0
        g[VehicleModel.V_LON, VehicleModel.TAN_STEER] = 0.0
        g[VehicleModel.PHI, VehicleModel.TAN_STEER] = v_lon / self.L
        return g


class VehicleCBF():
    
    def __init__(
        self,
        vehicleModel: VehicleModel,
        objParam: List,
        dt: float,
        alpha: Optional[float] = None,
        Q:Optional[np.array] = None, 
        ) -> None:
        self.vehicleModel = vehicleModel
        self.dt = dt

        if alpha is None:
            alpha = 0.5
        self.alpha = alpha

        if Q is None:
            Q = np.diag([0.05, 20.0])
        self.Q = Q

        self.obj = []
        for itm in objParam:
            barrier = Barrier(dt, itm[0], itm[1])
            self.obj.append(barrier)

        self.objSize = len(self.obj)
    
    def solove(
        self,
        state: np.array,
        ego_length: float,
        ego_width: float,
        action_ref: np.array,
        a_lower_limit: float,
        a_upper_limit: float,
        steer_lower_limit: float,
        steer_upper_limit: float,
        ) -> List[float]:
        f, g = self.vehicleModel.get_dynamic(state)
        x = state[self.vehicleModel.X_GLOBAL, 0]
        y = state[self.vehicleModel.Y_GLOBAL, 0]
        v_lon = state[self.vehicleModel.V_LON, 0]
        phi = state[self.vehicleModel.PHI, 0]
        action_ref[self.vehicleModel.TAN_STEER, 0] = np.tan(action_ref[self.vehicleModel.TAN_STEER, 0])

        if v_lon > 5:
            self.Q[1, 1] = 5 + v_lon * 3

        action_cbf = cvxpy.Variable((self.vehicleModel.n_control, 1))
        lam = cvxpy.Variable((self.objSize, 1), nonneg = True)
        action_cbf.project_and_assign(action_ref)
        lam.project_and_assign(np.zeros((self.objSize, 1)))
        # print(f"action_ref {action_ref}") # KAI
        constraints = []
        cost = cvxpy.quad_form(action_ref - action_cbf, self.Q) + cvxpy.sum(lam)

        constraints = constraints + [action_cbf[self.vehicleModel.A_LON, 0] <= a_upper_limit]
        constraints = constraints + [action_cbf[self.vehicleModel.A_LON, 0] >= a_lower_limit]
        constraints = constraints + [action_cbf[self.vehicleModel.TAN_STEER, 0] <= np.tan(steer_upper_limit)]
        constraints = constraints + [action_cbf[self.vehicleModel.TAN_STEER, 0] >= np.tan(steer_lower_limit)]

        status_list = []
        barrier_list = []

        for idx in range(self.objSize):
            obj = self.obj[idx]
            obj.cal(state, ego_length, ego_width)

            h = obj.h

            Lfh = np.dot(obj.dh_dstate, f)
            h_dot = Lfh + obj.dhobj_dt

            dLfh_dstate = np.zeros((1, self.vehicleModel.n_dim))
            dLfh_dstate[0, self.vehicleModel.X_GLOBAL] = obj.ddh_dx_dx * v_lon * np.cos(phi) + obj.ddh_dx_dy * v_lon * np.sin(phi)
            dLfh_dstate[0, self.vehicleModel.Y_GLOBAL] = obj.ddh_dx_dy * v_lon * np.cos(phi) + obj.ddh_dy_dy * v_lon * np.sin(phi)
            dLfh_dstate[0, self.vehicleModel.V_LON] = obj.dh_dx * np.cos(phi) + obj.dh_dy * np.sin(phi)
            dLfh_dstate[0, self.vehicleModel.PHI] = - obj.dh_dx * v_lon * np.sin(phi) + obj.dh_dy * v_lon * np.cos(phi)
            LfLfh = np.dot(dLfh_dstate, f)
            LgLfh = np.dot(dLfh_dstate, g)

            A = - LgLfh
            b = LfLfh + 2.0 * np.dot(obj.ddhobj_dstate_dt, f) + obj.ddhobjobj_dt_dt + self.alpha ** 2 * h + 2.0 * self.alpha * h_dot
            
            # print(f"obj: {obj.name}, h: {h}, h_dot: {h_dot}, A: {A}, b: {b}") # KAI

            # constraints = constraints + [lam[idx, 0] >= 0.0]

            if np.linalg.norm(A, ord=2) < 1e-3:
                continue
            else:
                constraints = constraints + [A @ action_cbf <= b + np.max([np.min([h, (h + 0.5 * h_dot)[0, 0]]), 0.0]) * lam[idx, 0]]
                barrier_list.append((h + 0.5 * h_dot)[0, 0])
                status_list.append(obj.status)

        # problem building
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        # solving
        prob.solve(solver = cvxpy.ECOS, verbose = False, warm_start = True)

        # result
        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            accel_cbf = action_cbf.value[self.vehicleModel.A_LON, 0]
            steer_cbf = np.arctan(action_cbf.value[self.vehicleModel.TAN_STEER, 0])
            # print(f"action_cbf {[accel_cbf, steer_cbf]}") # KAI
            # print(f"lambda {lam.value.T}") # KAI
            # for con in constraints:
            #     print(f"dual value {con.dual_value}") 
            return [accel_cbf, steer_cbf]

        else:
            # print("==============CVXPY ERROR==============") # KAI
            Status_dict = {'status': status_list, 'barrier': barrier_list}
            index = np.argmin(Status_dict["barrier"])
            if Status_dict["status"][index] == 'Front':
                accel_cbf = a_lower_limit
            elif Status_dict["status"][index] == 'Behind':
                accel_cbf = a_upper_limit
            else:
                accel_cbf = action_ref[self.vehicleModel.A_LON, 0]
            steer_cbf = np.arctan(action_ref[self.vehicleModel.TAN_STEER, 0])       
            # print(f"action_cbf {[accel_cbf, steer_cbf]}") # KAI
            return [accel_cbf, steer_cbf]
    

    

        