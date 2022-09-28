from PotentialFields import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from numpy.linalg import inv, det
from scipy.optimize import minimize

class splineControl:

    def __init__(self, n):
        t0 = lambda t: [np.power(t, i) for i in np.flip(np.arange(0, 6, 1))]
        t1 = lambda t: [np.power(t, max(0, i - 1)) * i for i in np.flip(np.arange(0, 6, 1))]

        self.Mat = np.block([t0(i) for i in [0, .5, 1]] + [t1(i) for i in [0, .5, 1]])
        self.MatInv = np.linalg.inv(self.Mat)

        self.size = n

    def updatePosition(self, points, normals):
        # Points, separate in an Nx2 array
        tmp = np.array(points)
        self.points = np.reshape(tmp, (max(np.array(tmp).shape), 2))

        # Normals, separate in an Nx2 array
        tmp = np.array(normals)
        self.normals = np.reshape(tmp, (max(np.array(tmp).shape), 2))

        # Generate a 3 vector list, for a closed loop
        tmp = np.concatenate((self.points, self.points[:3]))
        self.points_list = [tmp[i:i + 3, :] for i in range(max(self.points.shape))]

        # Generate a 3 vector list, for a closed loop
        tmp = np.concatenate((self.normals, self.normals[:3]))
        self.normals_list = [tmp[i:i + 3] for i in range(max(self.normals.shape))]

    def updateTarget(self, points, normals):
        # Points
        tmp = np.array(points)
        points_t = np.reshape(tmp, (max(np.array(tmp).shape), 2))

        # Normals
        tmp = np.array(normals)
        normals_t = np.reshape(tmp, (max(np.array(tmp).shape), 2))

        tmp = np.concatenate((points_t, points_t[:3]))
        self.points_t_list = [tmp[i:i + 3, :] for i in range(max(points_t.shape))]

        tmp = np.concatenate((normals_t, normals_t[:3]))
        self.normals_t_list = [tmp[i:i + 3] for i in range(max(normals_t.shape))]

        # Generate list of b vectors, defining the position and normals, premultiplying by MatInv will give polynomials
        b_t = [np.vstack((p, np.array([-n[:, 1], n[:, 0]]).T)) for p, n in zip(self.points_t_list, self.normals_t_list)]

        # Generate list of polynomials, M x 2x6
        self.pols_t_list = [self.MatInv @ b for b in b_t]

    def calculateError(self, u):
        tmp = np.concatenate((u, u[:3]))
        u_list = [tmp[i:i + 3] for i in range(len(u))]

        error = 0
        for i, (ud, pd, nd, pols_targ) in enumerate(zip(u_list, self.points_list, self.normals_list, self.pols_t_list)):
            if i%2 == 0:
                t_list = np.array([0, .5, 1])#np.linspace(0, 1, 3)
                t_list = np.linspace(0, 1, 50)

                b = np.vstack((pd + (nd.T * ud).T, np.array([-nd[:, 1], nd[:, 0]]).T))
                pols_new = self.MatInv @ b

                # Current position Curvature
                xt = pols_targ[:, 0]
                yt = pols_targ[:, 1]

                xt_d = np.polyder(xt)
                xt_dd = np.polyder(xt, 2)

                yt_d = np.polyder(yt)
                yt_dd = np.polyder(yt, 2)

                kt_num = np.polymul(xt_d, yt_dd) - np.polymul(yt_d, xt_dd)
                kt_den = np.polymul(xt_d, xt_d) + np.polymul(yt_d, yt_d)



                kt_list = np.polyval(kt_num, t_list) / (np.polyval(kt_den, t_list) ** (3 / 2))

                # New position curvature
                xn = pols_new[:, 0]
                yn = pols_new[:, 1]

                xn_d = np.polyder(xn)
                xn_dd = np.polyder(xn, 2)

                yn_d = np.polyder(yn)
                yn_dd = np.polyder(yn, 2)

                kn_num = np.polymul(xn_d, yn_dd) - np.polymul(yn_d, xn_dd)
                kn_den = np.polymul(xn_d, xn_d) + np.polymul(yn_d, yn_d)

                kn_list = np.polyval(kn_num, t_list) / (np.polyval(kn_den, t_list)**(3/2))

                Rn_list = 1/kn_list
                Rt_list = 1/kt_list
                error += np.sum((kn_list - kt_list)**2)

        return error

    def findOptimalU(self):

        #out = minimize(self.calculateError, (np.random.random(self.size) - .5)*.01, method='BFGS')
        out = minimize(self.calculateError, np.zeros(self.size), method='Nelder-Mead')

        print(out)

        return out.x

    def getCurve(self):
        x, y = self.getNewCurve(np.zeros(self.size))

        return x, y

    def getNewCurve(self, u):
        tmp = np.concatenate((u, u[:3]))
        u_list = [tmp[i:i + 3] for i in range(len(u))]

        t = np.linspace(.5, 1, 50)
        x = np.array([])
        y = np.array([])
        for ud, pd, nd, pols_targ in zip(u_list, self.points_list, self.normals_list, self.pols_t_list):
            b = np.vstack((pd + (nd.T * ud).T, (np.array([nd[:, 1], -nd[:, 0]])*(1+ud*2)).T))
            pols_new = self.MatInv @ b

            x = np.concatenate((x, np.polyval(pols_new[:, 0], t)))
            y = np.concatenate((y, np.polyval(pols_new[:, 1], t)))

        return x, y

    def getOptimalCurve(self):
        t = np.linspace(.5, 1, 50)
        x = np.array([])
        y = np.array([])
        for pd, nd, pols_targ in zip(self.points_t_list, self.normals_t_list, self.pols_t_list):
            b = np.vstack((pd, np.array([nd[:, 1], -nd[:, 0]]).T))
            pols_new = self.MatInv @ b

            x = np.concatenate((x, np.polyval(pols_new[:, 0], t)))
            y = np.concatenate((y, np.polyval(pols_new[:, 1], t)))

        return x, y


pts = np.array([[2.32, 9.209],
                [3.341, 4.971],
                [8.15, 4.686],
                [10.055, 0.285],
                [8.15, -5.216],
                [-1.222, -7.742],
                [-7.353, -2.06],
                [-6.561, 9.112],
                [-2.076, 9.82]])
#pts = np.array([[0, 0], [2.5, 1], [3, -1]])

pts_err = pts + (np.random.random(pts.shape) - .5)*3
tmp = np.linspace(0, 1, np.max(pts.shape) + 1)[:-1]
pts_err = np.array([np.sin(2 * np.pi * tmp), np.cos(2 * np.pi * tmp)]).T * 10

nts = np.array([[.707, .707],
                [.707, .707],
                [.707, .707],
                [1, 0],
                [.707, -.707],
                [0, -1],
                [-.707, -.707],
                [-.707, 0.707],
                [-.707, .707]]) * 10
#nts = np.array([[-1, 0], [.707, .707], [1, 0]])*10
nts_err = nts + (np.random.random(nts.shape) - .5)*.05
nts_err = np.array([np.sin(2 * np.pi * tmp), np.cos(2 * np.pi * tmp)]).T * 8


sp = splineControl(9)

sp.updateTarget(pts, nts)
sp.updatePosition(pts_err, nts_err)

u = sp.findOptimalU()

x, y = sp.getCurve()
xo, yo = sp.getOptimalCurve()
xn, yn = sp.getNewCurve(u)

# Center positions
xm = np.average(x)
ym = np.average(y)
x = x - xm
y = y - ym

xm = np.average(xo)
ym = np.average(yo)
xo = xo - xm
yo = yo - ym

xm = np.average(xn)
ym = np.average(yn)
xn = xn - xm
yn = yn - ym

plt.plot(x, y)
plt.plot(xo, yo, '--')
plt.plot(xn, yn)
plt.legend(['Current Position', 'Target Position', 'New Position'])
plt.show()

def getPolinomials(pts, nts):
    nts2 = np.vstack([nts[:, 1], -nts[:, 0]]).T
    b = np.block([pts.T, nts2.T]).T

    t0 = lambda t: [np.power(t, i) for i in np.flip(np.arange(0, 6, 1))]
    t1 = lambda t: [np.power(t, max(0, i - 1)) * i for i in np.flip(np.arange(0, 6, 1))]

    Mat = np.block([t0(i) for i in [0, .5, 1]] + [t1(i) for i in [0, .5, 1]])

    pols = np.linalg.lstsq(Mat, b)[0]#inv(Mat) @ b

    return pols, Mat, b

# Define target
pts = np.array([[0, 0], [2.5, 1], [3, -1]])
nts = np.array([[0, 1], [.707, .707], [1, 0]]).T
rs = np.array([1, 1, 1])*3

nts = (nts * rs).T

# Define current:
pts2 = pts + np.random.random(pts.shape)*.7-.35
nts2 = nts

pols, _, _ = getPolinomials(pts, nts)
pols2, mat2, b2 = getPolinomials(pts2, nts2)
#invMat2 = np.linalg.inv(mat2)

def optimizable(u):
    J = np.block([nts2.T, np.zeros((2, 3))]).T

    #C = invMat2 @ (b2.T + J.T * np.block([u, np.zeros(u.shape)])).T
    C = np.linalg.lstsq(mat2, (b2.T + J.T * np.block([u, np.zeros(u.shape)])).T)[0]
    cost = C - pols
    p0 = cost[:, 0]#np.polyint(cost[:, 0])
    p1 = cost[:, 1]#np.polyint(cost[:, 1])

    p0 = np.polyint(np.convolve(p0, p0))
    p1 = np.polyint(np.convolve(p1, p1))

    p0 = np.polyval(p0, 1)# - np.polyval(p0, 0)
    p1 = np.polyval(p1, 1)# - np.polyval(p1, 0)

    val = p0**2 + p1**2

    return val

out = minimize(optimizable, np.array([0, 0, 0]))

print(out)

u = out.x
J = np.block([nts2.T, np.zeros((2, 3))]).T
#C = invMat2 @ (b2.T + J.T * np.block([u, np.zeros(u.shape)])).T
C = np.linalg.lstsq(mat2, (b2.T + J.T * np.block([u, np.zeros(u.shape)])).T)[0]


S2 = lambda t: np.array([np.polyval(C[:, 0], t), np.polyval(C[:, 1], t)])
N2 = lambda t: np.array([-np.polyval(np.polyder(C[:, 1]), t), np.polyval(np.polyder(C[:, 0]), t)])

S3 = lambda t: np.array([np.polyval(pols2[:, 0], t), np.polyval(pols2[:, 1], t)])
N3 = lambda t: np.array([-np.polyval(np.polyder(pols2[:, 1]), t), np.polyval(np.polyder(pols2[:, 0]), t)])

S = lambda t: np.array([np.polyval(pols[:, 0], t), np.polyval(pols[:, 1], t)])
N = lambda t: np.array([-np.polyval(np.polyder(pols[:, 1]), t), np.polyval(np.polyder(pols[:, 0]), t)])
t = np.linspace(0, 1, 100)

plt.plot(S(t)[0], S(t)[1])
plt.plot(pts[:, 0], pts[:, 1], 'kx')


plt.plot(S2(t)[0], S2(t)[1], '--')
plt.plot(S3(t)[0], S3(t)[1], '--')

k = .25
plt.quiver(pts[:, 0], pts[:, 1], nts[:, 0], nts[:, 1])
plt.quiver(S(k)[0], S(k)[1], N(k)[0], N(k)[1])

plt.legend(['Nodes', 'Ideal Surface', 'Next step surface', 'Current surface'])

plt.show()