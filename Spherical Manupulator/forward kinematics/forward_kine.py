import math
import numpy as np

def dh_transform(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,     sa,     ca,    d],
        [0,      0,      0,    1]
    ])

def sysCall_init():
    sim = require('sim')
    simUI = require('simUI')

    self.joint_r1 = sim.getObject('/Joint_r1')
    self.joint_r2 = sim.getObject('/Joint_r2')
    self.joint_p1 = sim.getObject('/Joint_p1')
    self.endeff = sim.getObject('/EndEff')
    self.endeff_trace = sim.addDrawingObject(sim.drawing_linestrip, 5, 0, -1, 10000, [1, 0, 0])

    xml = '''
        <ui title="Joint Control Panel" closeable="false" resizeable="false" placement="topRight">
            <label text="Base Rotation (rad -3.14 to 3.14)"/>
            <hslider minimum="-314" maximum="314" on-change="sliderChanged" id="1"/>
            <label text="Elbow Rotation (rad -1.0 to 1.0)"/>
            <hslider minimum="-100" maximum="100" on-change="sliderChanged" id="2"/>
            <label text="Extension (m 0.1 to 0.3)"/>
            <hslider minimum="10" maximum="30" on-change="sliderChanged" id="3"/>
        </ui>
        '''
    
    self.ui = simUI.create(xml)
    self.slider_vals = [0, 0, 0]

def sliderChanged(ui, id, val):
    if id == 1:
        self.slider_vals[0] = val / 100.0
    elif id == 2:
        self.slider_vals[1] = val / 100.0
    elif id == 3:
        self.slider_vals[2] = val / 100.0

def forward_kinematics(theta1, theta2, d3, L1=0.2):
    # DH-based transformation
    T1 = dh_transform(theta1, 0, 0, math.pi / 2)
    T2 = dh_transform(theta2, 0, L1, 0)
    T3 = dh_transform(0, d3, 0, 0)
    T = T1 @ T2 @ T3
    position = T[0:3, 3]
    return position

def sysCall_actuation():
    sim = require('sim')

    theta1 = self.slider_vals[0]
    theta2 = self.slider_vals[1]
    d3 = self.slider_vals[2]

    sim.setJointTargetPosition(self.joint_r1, theta1)
    sim.setJointTargetPosition(self.joint_r2, theta2)
    sim.setJointTargetPosition(self.joint_p1, d3)

    pos_fk = forward_kinematics(theta1, theta2, d3)
    print(f"FK Position: x={pos_fk[0]:.3f}, y={pos_fk[1]:.3f}, z={pos_fk[2]:.3f}")
    print(f"Theta1: {round(theta1, 2)} rad, Theta2: {round(theta2, 2)} rad, Extension: {round(d3, 3)} m")

def sysCall_sensing():
    sim = require('sim')
    pos = sim.getObjectPosition(self.endeff, sim.handle_world)
    sim.addDrawingObjectItem(self.endeff_trace, pos)

def sysCall_cleanup():
    simUI = require('simUI')
    if hasattr(self, 'ui'):
        simUI.destroy(self.ui)