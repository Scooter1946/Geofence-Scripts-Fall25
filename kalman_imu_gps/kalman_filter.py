import math
import time

# --- matrix helpers using pure Python lists ---
def mat_eye(n):
    return [[1 if i==j else 0 for j in range(n)] for i in range(n)]

def mat_mul(A,B):
    n,m,p = len(A), len(B), len(B[0])
    C = [[0]*p for _ in range(n)]
    for i in range(n):
        for j in range(p):
            s = 0
            for k in range(m):
                s += A[i][k]*B[k][j]
            C[i][j] = s
    return C

def mat_vec_mul(A,v):
    return [sum(A[i][j]*v[j] for j in range(len(v))) for i in range(len(A))]

def mat_transpose(A):
    return [list(row) for row in zip(*A)]

def mat_add(A,B):
    return [[A[i][j]+B[i][j] for j in range(len(A[0]))] for i in range(len(A))]

def mat_sub(A,B):
    return [[A[i][j]-B[i][j] for j in range(len(A[0]))] for i in range(len(A))]

def symmetrize(A):
    n = len(A)
    return [[0.5*(A[i][j]+A[j][i]) for j in range(n)] for i in range(n)]

def inv_2x2(M):
    det = M[0][0]*M[1][1] - M[0][1]*M[1][0]
    return [[ M[1][1]/det, -M[0][1]/det],
            [-M[1][0]/det,  M[0][0]/det]]

# ensures angle is within acceptable range and not circled around i.e. -1.5pi instead of 0.5 pi
def wrap_angle(a):
    """Keep angle in [-pi, pi)."""
    return (a + math.pi) % (2*math.pi) - math.pi

# filter
class ExtendedKalmanFilter():
    """EKF with state [x, y, theta, vx, vy].
    - Predict step uses IMU accel + gyro
    - Update step uses GPS position
    """

    def __init__(self, x0, P0):
        self.x = [float(v) for v in x0]   # state
        self.P = [row[:] for row in P0]   # covariance

    def predict_function(self, x_state, u, dt):
        """Compute next state (nonlinear motion model)."""
        ax_b, ay_b, omega = u
        x, y, theta, vx, vy = x_state
        # converts from IMU readings to real world values
        # heading angle
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        # acceleration in x y directions using heading
        ax_w = cos_t*ax_b - sin_t*ay_b
        ay_w = sin_t*ax_b + cos_t*ay_b
        # predicts new position using current position, velocity, and acceleration
        x_new = x + vx*dt + 0.5*ax_w*dt*dt
        y_new = y + vy*dt + 0.5*ay_w*dt*dt
        # predicts a new heading angle
        theta_new = wrap_angle(theta + omega*dt)
        # predicts new velocities
        vx_new = vx + ax_w*dt
        vy_new = vy + ay_w*dt

        return [x_new, y_new, theta_new, vx_new, vy_new]

    # tell filter how uncertainties may affect predicted positions
    def jacobian_G(self, u, dt):
        """Compute analytic Jacobian of motion model."""
        # get accelerations and find cos/sin of heading
        ax_b, ay_b, _ = u
        _, _, theta, _, _ = self.x
        c = math.cos(theta)
        s = math.sin(theta)
        # partial derivative in accordance with heading theta
        # propagates uncertainty in theta into x/y position
        dax_dtheta = -s*ax_b - c*ay_b
        day_dtheta =  c*ax_b - s*ay_b
        # dt^2 used in x = x1 + vt + 0.5*dt2*d_dtheta
        dt2 = dt*dt

        return [
            [1, 0, 0.5*dt2*dax_dtheta, dt, 0], # [2]how small change in theta affects x position, [3]how x velocity affects x position
            [0, 1, 0.5*dt2*day_dtheta, 0, dt], # [2]how small change in theta affects y position, [3]how y velocity affects y position
            [0, 0, 1,                  0, 0 ], #
            [0, 0, dt*dax_dtheta,      1, 0 ], # how theta affects x velocity
            [0, 0, dt*day_dtheta,      0, 1 ]  # how theta affects y velocity
        ]

    def propagate(self, u, dt, R):
        """Predict next state and covariance using IMU data."""
        # predicts new state
        self.x = self.predict_function(self.x, u, dt)
        # ensures that theta is within certain range and not circled; i.e. -1.5pi rad instead of 0.5 rad
        self.x[2] = wrap_angle(self.x[2])
        # tells filter how small errors in current state propagate into next state
        G = self.jacobian_G(u, dt)
        # updates covariance: P_k+1 = G * P_k * G^T + R (Covariance is a measure of how uncertain an estimate is
        #                                                and how errors in one variable might relate to errors in another.)
        GP = mat_mul(G, self.P)
        GPGt = mat_mul(GP, mat_transpose(G))
        self.P = symmetrize(mat_add(GPGt, R))

    def update(self, z, Q):
        """Update state with GPS measurement z=[x,y]."""
        # maps state [x, y, theta, vx, vy] to [x,y]
        H = [[1,0,0,0,0],
             [0,1,0,0,0]]
        # S = HPH^T + Q. S = covariance of prediction in measurement space + GPS noise.
        # tells the filter how uncertain the predicted position is compared to the GPS reading.
        HP = mat_mul(H, self.P)
        HPHt = mat_mul(HP, mat_transpose(H))
        S = mat_add(HPHt, Q)
        S_inv = inv_2x2(S)
        # K =   PH^T S^-1
        # calculates kalman gain: how much to trust GPS vs IMU. large P = trust GPS, large Q = trust IMU
        Ht = mat_transpose(H)
        PHt = mat_mul(self.P, Ht)
        K = mat_mul(PHt, S_inv)
        # calculates residual/difference actual GPS reading and predicted position
        Hx = mat_vec_mul(H, self.x)
        y = [z[0]-Hx[0], z[1]-Hx[1]]
        # correction for each state
        delta = mat_vec_mul(K, y)
        # updates position, velocity, and heading
        self.x = [self.x[i] + delta[i] for i in range(5)]
        self.x[2] = wrap_angle(self.x[2])
        # updates covariance using Joseph Form
        I = mat_eye(5)
        KH = mat_mul(K, H)
        I_KH = mat_sub(I, KH)
        term1 = mat_mul(mat_mul(I_KH, self.P), mat_transpose(I_KH))
        term2 = mat_mul(mat_mul(K, Q), mat_transpose(K))
        self.P = symmetrize(mat_add(term1, term2))

    # getters
    def state(self):
        return self.x[:]
    def cov(self):
        return [row[:] for row in self.P]
