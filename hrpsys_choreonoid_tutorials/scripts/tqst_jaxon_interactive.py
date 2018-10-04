#!/usr/bin/env python

from hrpsys_choreonoid_tutorials.choreonoid_hrpsys_config import *

class JAXON_RED_HrpsysConfigurator(ChoreonoidHrpsysConfigurator):
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
            ['co', "CollisionDetector"],
            # ['tc', "TorqueController"],
            # ['te', "ThermoEstimator"],
            # ['tl', "ThermoLimiter"],
            ['rfu', "ReferenceForceUpdater"],
            ['octd', "ObjectContactTurnaroundDetector"],
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            ['log', "DataLogger"]
            ]

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

    def startABSTIMP (self):
        ### not used on hrpsys
        self.el_svc.setServoErrorLimit("motor_joint",   sys.float_info.max)
        self.el_svc.setServoErrorLimit("RARM_F_JOINT0", sys.float_info.max)
        self.el_svc.setServoErrorLimit("RARM_F_JOINT1", sys.float_info.max)
        self.el_svc.setServoErrorLimit("LARM_F_JOINT0", sys.float_info.max)
        self.el_svc.setServoErrorLimit("LARM_F_JOINT1", sys.float_info.max)
        ###
        self.startAutoBalancer()
        # Suppress limit over message and behave like real robot that always angle-vector is in seq.
        # Latter four 0.0 are for hands.
        self.seq_svc.setJointAngles(self.jaxonResetPose()+[0.0, 0.0, 0.0, 0.0] , 1.0)
        #self.ic_svc.startImpedanceController("larm")
        #self.ic_svc.startImpedanceController("rarm")
        self.startStabilizer()

def setServoGain(l, a, b):
    if(b > a):
        for i in l:
            hcf.rh_svc.setServoTorqueGainPercentage(i, b)
        time.sleep(3)
        for i in l:
            hcf.rh_svc.setServoGainPercentage(i, a)
        time.sleep(3)
    else:
        for i in l:
            hcf.rh_svc.setServoGainPercentage(i, a)
        time.sleep(3)
        for i in l:
            hcf.rh_svc.setServoTorqueGainPercentage(i, b)
        time.sleep(3)
        for i in l:
            hcf.rh_svc.setJointControlMode(i, OpenHRP.RobotHardwareService.POSITION_TORQUE)

def setAllParam(a, b):
    l = ["LLEG_JOINT1","RLEG_JOINT1","LLEG_JOINT5","RLEG_JOINT5","LLEG_JOINT2","RLEG_JOINT2","LLEG_JOINT3","RLEG_JOINT3","LLEG_JOINT4","RLEG_JOINT4","LLEG_JOINT0","RLEG_JOINT0", 'RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'RARM_JOINT7', 'LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7', 'HEAD_JOINT0', 'HEAD_JOINT1', 'CHEST_JOINT0', 'CHEST_JOINT1', 'CHEST_JOINT2']
    setServoGain(l, a, b)

def setTorqueControl():
    hcf.st_svc.stopStabilizer()
    hcf.abc_svc.stopAutoBalancer()
    setAllParam(100, 0)
    setAllParam(0, 100)

if __name__ == '__main__':
    hcf = JAXON_RED_HrpsysConfigurator("JAXON_RED")
    [sys.argv, connect_constraint_force_logger_ports] = hcf.parse_arg_for_connect_ports(sys.argv)
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
    else :
        hcf.init(connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
