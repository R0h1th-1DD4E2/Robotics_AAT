import math
import numpy as np

def inverse_kinematics(x, y, z, L1=0.2):
    theta1 = math.atan2(y, x)
    r_xy = math.sqrt(x**2 + y**2)
    theta2 = math.atan2(z, r_xy)
    d3 = math.sqrt(x**2 + y**2 + z**2) - L1
    return theta1, theta2, d3

def sysCall_init():
    sim = require('sim')
    simUI = require('simUI')

    self.joint_r1 = sim.getObject('/Joint_r1')
    self.joint_r2 = sim.getObject('/Joint_r2')
    self.joint_p1 = sim.getObject('/Joint_p1')
    self.endeff = sim.getObject('/EndEff')

    self.endeff_trace = sim.addDrawingObject(sim.drawing_linestrip, 5, 0, -1, 10000, [0, 1, 0])

    xml = '''<ui title="IK Target" closeable="false" resizeable="false" placement="topRight">
    <label text="Target X (m)"/> <edit id="1" value="0.2"/>
    <label text="Target Y (m)"/> <edit id="2" value="0.0"/>
    <label text="Target Z (m)"/> <edit id="3" value="0.3"/>
    <button text="Go!" on-click="onMoveClick"/>
    </ui>'''
    self.ui = simUI.create(xml)

def onMoveClick(ui, id):
    sim = require('sim')
    simUI = require('simUI')

    x = float(simUI.getEditValue(self.ui, 1))
    y = float(simUI.getEditValue(self.ui, 2))
    z = float(simUI.getEditValue(self.ui, 3))

    theta1, theta2, d3 = inverse_kinematics(x, y, z)

    sim.setJointTargetPosition(self.joint_r1, theta1)
    sim.setJointTargetPosition(self.joint_r2, theta2)
    sim.setJointTargetPosition(self.joint_p1, d3)

    print(f"Moved to Target: x={x:.3f}, y={y:.3f}, z={z:.3f}")
    print(f"Computed IK: ?1={theta1:.2f}, ?2={theta2:.2f}, d3={d3:.3f}")

def sysCall_sensing():
    sim = require('sim')
    pos = sim.getObjectPosition(self.endeff, sim.handle_world)
    sim.addDrawingObjectItem(self.endeff_trace, pos)

def sysCall_cleanup():
    simUI = require('simUI')
    if hasattr(self, 'ui'):
        simUI.destroy(self.ui)
