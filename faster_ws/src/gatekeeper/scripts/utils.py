import numpy as np
from geometry_msgs.msg import Pose, Vector3, Quaternion
from numpy import linalg as LA
from tf.transformations import quaternion_matrix, quaternion_from_matrix, euler_from_quaternion

def vector3_to_np(v):
    return np.array([v.x, v.y, v.z])

def quat_to_np(q):
    return np.array([q.x, q.y, q.z, q.w])

def np_to_vector3(v):
    return Vector3(v[0], v[1], v[2])

def quaternion_to_rotation(q):
    qq = np.array([q.x, q.y, q.z, q.w])
    H = quaternion_matrix(qq / LA.norm(qq))
    return H[0:3, 0:3]

def hat(v):
    return np.array([
                [0., -v[2], v[1]],
                [v[2], 0., -v[0]],
                [-v[1], v[0], 0.]
            ])

def vee(M):
    return np.array([
        M[2,1],
        M[0,2],
        M[1,0]
        ])

def getYaw(state):
    # from a snapstack state
    r, p, y = euler_from_quaternion(quat_to_np(state.quat))
    return y

def np_rotation_to_quat(R):
    
    # first project to SO(3)
    # usvT = LA.svd(R)
    # R = np.matmul(usvT[0], usvT[2])

    # make it a homogenous matrix
    H = np.block([[R, np.zeros([3,1])],[np.zeros([1,3]), 1]])

    # convert to quaternion
    q = quaternion_from_matrix(H)

    # return as quaternion object
    return Quaternion(q[0], q[1], q[2], q[3])


def normalize(v):
    return v / LA.norm(v)

# I despise python's np.matmul
def mm(v1, v2):
    return np.matmul(v1, v2)

def mmm(v1, v2, v3):
    return mm(v1, mm(v2, v3))

def mmmm(v1, v2, v3, v4):
    return mm( mm(v1, v2), mm(v3, v4) )

def log(v, label, every):
    str_ = label + ": %f, %f, %f" % (v[0], v[1], v[2])
    rospy.loginfo_throttle(every, str_)



def snapstack_state_to_numpy(state):
    # state is of type snapstack_msgs.msg.State: [pos, vel, quat, w, abias, gbias]

    # extract states
    x = vector3_to_np(state.pos)
    v = vector3_to_np(state.vel)
    R = quaternion_to_rotation(state.quat) 
    w = vector3_to_np(state.w)

    # flatten into single vector
    R_flat = np.reshape(R, 9)
    
    return np.hstack([x,v,R_flat, w])


def numpy_to_snapstack_state(np_state):
    x_np = np_state[0:3]
    v_np = np_state[3:6]
    R_np = np.reshape(np_state[6:15], [3,3])
    w_np = np.state[15:18]

    state = State()
    state.pos = np_to_vector3(x_np)
    state.vel   = np_to_vector3(v_np)
    state.quat  = np_rotation_to_quat(R_np)
    state.w     = np_to_vector3(w_np)
    state.abias = Vector3(0,0,0.)
    state.gbias = Vector3(0,0,0.)

    return state


def snapstack_goal_to_numpy_goal(goal):

        # extract state
        p = vector3_to_np(goal.p)
        v = vector3_to_np(goal.v)
        a = vector3_to_np(goal.a)
        j = vector3_to_np(goal.j)
        s = np.zeros(3) # since this is not specified in the goal


        # implement my own yaw controller instead
        #yaw = goal.yaw
        #dyaw = goal.dyaw
        #ddyaw = 0.0

        ## TODO(dev): update dyaw from goal too
        yaw = goal.yaw
        dyaw = 0.0
        ddyaw = 0.0

        return np.hstack([p, v, a, j, s, yaw, dyaw, ddyaw])
