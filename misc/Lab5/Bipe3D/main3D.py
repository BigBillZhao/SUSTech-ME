import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

class Biped3D(object):
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.resetDebugVisualizerCamera(cameraDistance=0.6, cameraYaw=0,
                                     cameraPitch=0, cameraTargetPosition=[0, 0, 0.2])
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

        self.ground = p.loadURDF("plane.urdf")
        self.robot = p.loadURDF("/yanshee/robots/yanshee.urdf", [0, 0, -0.1], [0, 0, 0, 1], useFixedBase=False)
        self.joints = self.get_joints()
        self.n_j = len(self.joints)
        self.simu_f = 200 # Simulation frequency, Hz
        p.setTimeStep(1.0/self.simu_f)
        self.motion_f = 2  # Controlled motion frequency, Hz
        self.world_joint = 0
        self.robo_pose = 0
        self.l1 = 0.05 # leg length 1
        self.l2 = 0.06 # leg length 2
        self.support_leg = 'left' # init left
        self.step_length = 0.04
        self.T = 0 # determined at run time
        self.n_T = 1
        self.mid_speed = 0.1 # speed at mid point
        self.omega = np.sqrt(9.81 / 0.1) # $angular velocity = \sqrt{g / z}$

        self.x_0 = -self.step_length/2
        self.x_dot_0 = np.sqrt(self.mid_speed ** 2 + self.omega ** 2 * (self.step_length / 2) ** 2)
        self.cosh = lambda x : (np.exp(x) + np.exp(-x)) / 2
        self.sinh = lambda x : (np.exp(x) - np.exp(-x)) / 2

        self.iq_vec = np.zeros(self.n_j)
        self.q_vec = np.zeros(self.n_j)
        self.dq_vec = np.zeros(self.n_j)
        self.q_mat = np.zeros((self.simu_f * 3, self.n_j)) # record data of 3 seconds
        self.q_d_mat = np.zeros((self.simu_f * 3, self.n_j)) # record data of 3 seconds
        self.init_plot()


    def run(self):
        for i in range(int(1e3)):
            t = i / self.simu_f
            torque_array = self.controller(t)
            self.q_vec, self.dq_vec = self.step(torque_array)
            if 0 == i % 20:
                self.update_plot()
            time.sleep(5/self.simu_f)
        p.disconnect()

    def step(self, torque_array):
        self.robo_pose = self.set_motor_torque_array(torque_array, 0.1 + self.robo_pose)
        p.resetDebugVisualizerCamera(cameraDistance=0.6, cameraYaw=0,
                                      cameraPitch=0, cameraTargetPosition=[self.robo_pose-0.1, 0, 0.2])
        p.stepSimulation()
        self.q_mat[:-1] = self.q_mat[1:]
        self.q_mat[-1] = self.q_vec
        return self.get_joint_states()

    def get_joints(self):
        all_joints = []
        for j in range(p.getNumJoints(self.robot)):
            # Disable motor in order to use direct torque control.
            info = p.getJointInfo(self.robot, j)
            joint_type = info[2]
            if (joint_type == p.JOINT_REVOLUTE):
                all_joints.append(j)
                p.setJointMotorControl2(self.robot, j,
                                        controlMode=p.POSITION_CONTROL, force=0)
            if (joint_type == p.JOINT_PRISMATIC): 
                self.world_joint = j
                p.setJointMotorControl2(self.robot, j, 
                                        controlMode=p.POSITION_CONTROL, force=0)
        joints = all_joints[0:]
        print("All joints are", all_joints)
        # print("Number of All Joints:", p.getNumJoints(self.robot))
        # print("Number of All Revolute Joints:", joints)
        return joints

    def get_joint_states(self):
        '''
        :return: q_vec: joint angle, dq_vec: joint angular velocity
        '''
        q_vec = np.zeros(self.n_j)
        dq_vec = np.zeros(self.n_j)
        for j in range(self.n_j):
            q_vec[j], dq_vec[j], _, _  = p.getJointState(self.robot, self.joints[j])
        return q_vec, dq_vec

    def set_motor_torque_array(self, torque_array = None, robot_posi=0):
        assert torque_array is not None
        for j in range(len(self.joints)):
            p.setJointMotorControl2(self.robot, self.joints[j], p.POSITION_CONTROL, targetPosition=torque_array[j])
        p.setJointMotorControl2(self.robot, self.world_joint, p.POSITION_CONTROL, targetPosition=robot_posi)
        return robot_posi

    def controller(self, t, type='joint'):
        if 'joint' == type:
            return self.joint_controller(t)

    def joint_controller(self, t):
        # $x(t) = x(0)cosh(\omega t) + \frac{\dot{x(0)}}{\omega} sinh(\omega t)$
        # $\dot{x(t)} = x(0)\omega sinh(\omega t) + \dot{x(0)} cosh(\omega t)$
        # $\omega$ = `self.omega`
        # $x(0)$ = `-self.step_length/2`
        # $\frac{1}{2} \dot{x}^2 - \frac{g}{2z} x^2 = E (Orbital Energy)$
        # $\dot{x(0)} = `np.sqrt(self.mid_speed ** 2 + self.omega ** 2 * (self.step_length / 2) ** 2)`
        x_0 = self.x_0
        x_dot_0 = self.x_dot_0
        cosh = self.cosh
        sinh = self.sinh
        y = 9.81 / self.omega ** 2
        t -= self.T * self.n_T
        x = x_0 * cosh(self.omega * t) + x_dot_0 * sinh(self.omega * t) / self.omega
        self.robo_pose = self.n_T * self.step_length - self.step_length / 2 + x
        if x > self.step_length / 2 and self.T == 0:
            self.T = t
            if self.support_leg == 'left':
                self.support_leg = 'right'
            else:
                self.support_leg = 'left'
        if self.T != 0 and t > self.T:
            self.n_T += 1
            if self.support_leg == 'left':
                self.support_leg = 'right'
            else:
                self.support_leg = 'left'
        x += 0.001
        ####################### Notations ####################
        # q, dq                                        | d
        # q stands for angle and dq stand for velocity | d stand for desired 
        # a, k, h                                      | p, w
        # a stands for ankle, k for knee and h for hip | p stand for support and w for swing
        q_d_a_p, q_d_k_p, _ = self.ik_planner_2R(x, y)
        q_d_h_p = np.pi / 2 - q_d_a_p - q_d_k_p
        # print('q_d_h_p = ', q_d_h_p/np.pi*180, ', q_d_k_p = ',
        #       q_d_k_p/np.pi*180, ', q_d_a_p = ', q_d_a_p/np.pi*180)
        q_d_h_w, q_d_k_w, _ = self.ik_planner_2R(self.step_length / 2, y / 5, bit=0)
        q_d_h_w = q_d_h_w - np.pi/2
        # q_d_k_w remain unchanged
        q_d_a_w = -q_d_h_w - q_d_k_w - np.pi / 12
        if self.T != 0:
            if x < 0: _y = y/5
            if x > 0: _y = y
            q_d_h_w, q_d_k_w, _ = self.ik_planner_2R((t/self.T-0.5)*self.step_length, _y, bit=0)
            q_d_h_w = q_d_h_w - np.pi/2
            # q_d_k_w remain unchanged
            q_d_a_w = -q_d_h_w - q_d_k_w - np.pi / 12
        print('q_d_h_w = ', q_d_h_w/np.pi*180, ', q_d_k_w = ',
              q_d_k_w/np.pi*180, ', q_d_a_w = ', q_d_a_w/np.pi*180)
        if self.support_leg == 'left':
            q_d_vec = np.array([-q_d_h_p, -q_d_k_p, (np.pi/2 - q_d_a_p)*0.9, 
                                q_d_h_w, q_d_k_w, q_d_a_w*0.8])
        else:
            q_d_vec = np.array([-q_d_h_w, -q_d_k_w, -q_d_a_w*0.8, 
                                q_d_h_p, q_d_k_p, (q_d_a_p - np.pi/2)*0.9])
        self.q_d_mat[:-1] = self.q_d_mat[1:]
        self.q_d_mat[-1] = q_d_vec
        return q_d_vec

        # dq_d_vec = np.array([0, 0, 0, 0, 0, 0])
        # # PID controller gains
        # P_vec = np.array([100, 50, 50, 100, 50, 50])
        # I_vec = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
        # D_vec = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
        # return self.joint_PID_controller(P_vec, I_vec, D_vec, self.q_vec, self.dq_vec, q_d_vec, dq_d_vec)

    def ik_planner_2R(self, x, y, bit=1):
        l1 = self.l1
        l2 = self.l2
        assert y > 0
        assert l1 > 0
        assert l2 > 0
        theta2 = np.pi - np.arccos((l1**2+l2**2-x**2-y**2)/(2*l1*l2))
        theta_avg = np.arctan2(y, x)
        theta_delta = np.arctan2(l2*np.sin(theta2), l1+l2*np.cos(theta2))
        if bit == 1:
            theta1 = theta_avg - theta_delta
        else:
            theta1 = theta_avg + theta_delta
            theta2 = -theta2
        return theta1, theta2, bit
        

    def joint_PID_controller(self, P_vec, I_vec, D_vec, q_vec, dq_vec, q_d_vec, dq_d_vec):
        self.iq_vec = self.iq_vec + (q_d_vec - q_vec)
        return P_vec*(q_d_vec-q_vec) + I_vec*self.iq_vec + D_vec*(dq_d_vec-dq_vec)

    def init_plot(self):
        self.fig = plt.figure(figsize=(5, 9))
        joint_names = ['left_hip', 'left_knee', 'left_ankle', 'right_hip', 'right_knee', 'right_ankle']
        self.q_d_lines = []
        self.q_lines = []
        for i in range(6):
            plt.subplot(6, 1, i+1)
            q_d_line, = plt.plot(self.q_d_mat[:, i], '-')
            q_line, = plt.plot(self.q_mat[:, i], '--')
            self.q_d_lines.append(q_d_line)
            self.q_lines.append(q_line)
            plt.ylabel('q_{} (rad)'.format(joint_names[i]))
            plt.ylim([-3, 3])
        plt.xlabel('Simulation steps')
        self.fig.legend(['q_d', 'q'], loc='lower center', ncol=2, bbox_to_anchor=(0.49, 0.97), frameon=False)
        self.fig.tight_layout()
        plt.draw()

    def update_plot(self):
        for i in range(6):
            self.q_d_lines[i].set_ydata(self.q_d_mat[:, i])
            self.q_lines[i].set_ydata(self.q_mat[:, i])
        plt.draw()
        plt.pause(0.001)


if __name__ == '__main__':
    robot = Biped3D()
    robot.run()