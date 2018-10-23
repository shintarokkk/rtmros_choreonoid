#!/usr/bin/env python
#for choreonoid

from hrpsys_choreonoid_tutorials.choreonoid_hrpsys_config import *

class JAXON_RED_HrpsysConfigurator(ChoreonoidHrpsysConfigurator):
    ltc = None
    ltc_svc = None
    ltc_version = None
    def getRTCList (self):
        ##return self.getRTCListUnstable()
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['tf', "TorqueFilter"],
            ['kf', "KalmanFilter"],
            ['vs', "VirtualForceSensor"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            ['es', "EmergencyStopper"],
            ['ic', "ImpedanceController"],
            ['abc', "AutoBalancer"],
            ['st', "Stabilizer"],
            ['ltc', 'LimbTorqueController'],
            # ['tc', "TorqueController"],
            # ['te', "ThermoEstimator"],
            # ['tl', "ThermoLimiter"],
            ['co', "CollisionDetector"],
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            ['log', "DataLogger"]
            ]

    def myconnect(self):
        # connection for ic
        if self.ltc:
            connectPorts(self.rh.port("q"), self.ltc.port("qCurrent"))
            connectPorts(self.rh.port("dq"), self.ltc.port("dqCurrent"))
            connectPorts(self.rh.port("tau"), self.ltc.port("tqCurrent"))
            connectPorts(self.st.port("q"), self.ltc.port("qRef"))
            connectPorts(self.ltc.port("tq"),self.rh.port("tauRef"))
            if StrictVersion(self.seq_version) >= StrictVersion('315.3.0'):
                connectPorts(self.sh.port("basePosOut"), self.ltc.port("basePosIn"))
                connectPorts(self.sh.port("baseRpyOut"), self.ltc.port("baseRpyIn"))

    def startLTC (self):
        if self.ltc:
            self.ltc_svc.startLimbTorqueController("rarm")
            self.ltc_svc.startLimbTorqueController("larm")

    def getJointAngleControllerList(self):
        '''!@brief
        Get list of controller list that need to control joint angles
        '''
        controller_list = [self.es, self.ic, self.abc, self.st, self.ltc, self.co, self.hes, self.el]
        return filter(lambda c: c != None, controller_list)  # only return existing controllers

    def defJointGroups (self):
        rarm_group = ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'RARM_JOINT7']]
        larm_group = ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7']]
        rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
        lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
        head_group = ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']]
        torso_group = ['torso', ['CHEST_JOINT0', 'CHEST_JOINT1', 'CHEST_JOINT2']]
        rhand_group = ['rhand', ['RARM_F_JOINT0', 'RARM_F_JOINT1']]
        lhand_group = ['lhand', ['LARM_F_JOINT0', 'LARM_F_JOINT1']]
        range_group = ['range', ['motor_joint']]
        self.Groups = [rarm_group, larm_group, rleg_group, lleg_group, head_group, torso_group, rhand_group, lhand_group, range_group]

    def setServoGain(self, l, a, b):
        if(b > a):
            for i in l:
                self.rh_svc.setServoTorqueGainPercentage(i, b)
            for i in l:
                self.rh_svc.setServoGainPercentage(i, a)
        else:
            for i in l:
                self.rh_svc.setServoGainPercentage(i, a)
            for i in l:
                self.rh_svc.setServoTorqueGainPercentage(i, b)
        for i in l:
            self.rh_svc.setJointControlMode(i, OpenHRP.RobotHardwareService.POSITION_TORQUE)

    def setAllParam(self, a,b):
        l = ["RARM_JOINT0","RARM_JOINT1","RARM_JOINT2","RARM_JOINT3","RARM_JOINT4","RARM_JOINT5", 'RARM_JOINT6', 'RARM_JOINT7', 'LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7', 'RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5', 'LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5', 'HEAD_JOINT0', 'HEAD_JOINT1', 'CHEST_JOINT0', 'CHEST_JOINT1', 'CHEST_JOINT2', 'RARM_F_JOINT0', 'RARM_F_JOINT1', 'LARM_F_JOINT0', 'LARM_F_JOINT1'] #motor_joint?
        self.setServoGain(l,a,b)

    def setPositionMode(self):
        l = ["RARM_JOINT0","RARM_JOINT1","RARM_JOINT2","RARM_JOINT3","RARM_JOINT4","RARM_JOINT5"]
        for i in l:
            self.rh_svc.setJointControlMode(i, OpenHRP.RobotHardwareService.POSITION)
    #TODO: full-body position torque mode
    def setPositionTorqueMode(self):
        lt = ["RARM_JOINT0","RARM_JOINT1","RARM_JOINT2","RARM_JOINT3","RARM_JOINT4","RARM_JOINT5", 'RARM_JOINT6', 'RARM_JOINT7', 'LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7']
        lp = ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5', 'LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5', 'HEAD_JOINT0', 'HEAD_JOINT1', 'CHEST_JOINT0', 'CHEST_JOINT1', 'CHEST_JOINT2', 'RARM_F_JOINT0', 'RARM_F_JOINT1', 'LARM_F_JOINT0', 'LARM_F_JOINT1']
        for i in lt:
            self.rh_svc.setJointControlMode(i, OpenHRP.RobotHardwareService.TORQUE)
        for i in lp:
            self.rh_svc.setJointControlMode(i, OpenHRP.RobotHardwareService.POSITION_TORQUE)


    def startABSTIMP (self):
        self.el_svc.setServoErrorLimit("motor_joint",   sys.float_info.max)
        self.el_svc.setServoErrorLimit("RARM_F_JOINT0", sys.float_info.max)
        self.el_svc.setServoErrorLimit("RARM_F_JOINT1", sys.float_info.max)
        self.el_svc.setServoErrorLimit("LARM_F_JOINT0", sys.float_info.max)
        self.el_svc.setServoErrorLimit("LARM_F_JOINT1", sys.float_info.max)
        #self.setPositionTorqueMode()
        self.startAutoBalancer()
        #self.seq_svc.setJointAngles(self.jaxonResetPose()+[0.0, 0.0, 0.0, 0.0] , 1.0)
        #self.ic_svc.startImpedanceController("larm")
        #self.ic_svc.startImpedanceController("rarm")
        self.startStabilizer()

if __name__ == '__main__':
    hcf = JAXON_RED_HrpsysConfigurator("JAXON_RED")
    [sys.argv, connect_constraint_force_logger_ports] = hcf.parse_arg_for_connect_ports(sys.argv)
    sys.argv
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
        #hcf.init(sys.argv[1], sys.argv[2], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.myconnect()
        hcf.startABSTIMP()
        hcf.startLTC()
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
        #hcf.init(sys.argv[1], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.myconnect()
        hcf.startABSTIMP()
        hcf.startLTC()
    else :
        hcf.init()
        #hcf.init(connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
