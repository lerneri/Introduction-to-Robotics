import numpy as np
import matplotlib.pyplot as plt
plt.style.use('ggplot')


pi = np.pi
s=0.3333
deltat=O.2
l1=0.5
l2=0.5
trelwuser = np.array([0.1, 0.2, 30.0])
treswuser = np.array([0.0, 0.0, 0.0])
srelbuser = np.array([0.0, 0.0, 0.0])
trelw = np.zeros((2, 3))
trelb = np.zeros((2, 3))
wrelb = np.zeros((2, 3))
srelb = np.zeros((2, 3))
brels = np.zeros((2, 3))
trels = np.zeros((2, 3))
wrelt = np.zeros((2, 3))
place = np.zeros(3)
current = np.zeros(3)
far = np.zeros(3)
npnt = 0
viapnt = np.zeros((5, 3))
viavel = np.zeros((5, 3))
path = np.zeros((5, 3, 4))
trajectory = np.zeros((300, 3))
deltat = 0.0
nticks = 0
sol = False
ans = ''

'''# Function to write a frame
def write_frame(foo):
    for i in range(2):
        print("[", end="")
        for j in range(3):
            print(f"{foo[i, j]:10.3f}", end="")
        print("]")

# Function to write a vector
def write_vect(foo):
    for i in range(3):
        print(f"[{foo[i]:10.3f}]")'''

def tmult(brela, crelb):
    crela = np.dot(brela, crelb)
    return crela

def tinvert(brela):
    arelb = np.linalg.inv(brela)
    return arelb


def kin(theta):
    T1 = np.array([[np.cos(theta[0]), -np.sin(theta[0]), 0],
                   [np.sin(theta[0]), np.cos(theta[0]), 0],
                   [0, 0, 1]])
    T2 = np.array([[np.cos(theta[1]), -np.sin(theta[1]), l1],
                   [np.sin(theta[1]), np.cos(theta[1]), 0],
                   [0, 0, 1]])
    wrelb = np.dot(T1, T2)

    return wrelb

    

'''# Function for kinematics
def kin(theta, wrelb):
    beta = np.sum(theta)
    wrelb[0, 2] = l1 * np.cos(theta[0]) + l2 * np.cos(np.sum(theta[:2]))
    wrelb[1, 2] = l1 * np.sin(theta[0]) + l2 * np.sin(np.sum(theta[:2]))
    wrelb[0, :2] = [np.cos(beta), -np.sin(beta)]
    wrelb[1, :2] = -wrelb[0, 1], wrelb[0, 0]'''

# Function to convert user form to matrix
def utoi(uform):
    iform[0, 2] = uform[0]
    iform[1, 2] = uform[1]
    iform[0, 0] = np.cos(uform[2] * np.deg2rad(1))
    iform[0, 1] = -np.sin(uform[2] * np.deg2rad(1))
    iform[1, 0] = -iform[0, 1]
    iform[1, 1] = iform[0, 0]
    return iform

def itou(iform):
    uform[0] = iform[0, 2]
    uform[1] = iform[1, 2]
    uform[2] = np.rad2deg(np.arctan2(iform[1, 0], iform[0, 0]))
    return uform


# Function for range normalization
def range(a):
    a %= 2 * pi
    if a > pi:
        a -= 2 * pi
    elif a < -pi:
        a += 2 * pi
    return a

# Function for inverse kinematics
def invkin(wrelb):
    goal = itou(wrelb)
    goal[2] =np.deg2rad(goal[2])
    c2 = (goal[0] ** 2 + goal[1] ** 2 - l1 ** 2 - l2 ** 2) / (2.0 * l1 * l2)
    if abs(c2) > 1.0:
        sol = False
        return near, far, sol
    else:
        sol = True
    s2 = np.sqrt(1.0 - c2 ** 2)
    near[1] = np.arctan2(s2, c2)
    far[1] = -near[1]
    k1 = l1 + l2 * c2
    k2 = l2 * s2
    temp = np.arctan2(k2, k1)
    near[0] = np.arctan2(goal[1], goal[0]) - temp
    far[0] = np.arctan2(goal[1], goal[0]) + temp
    near[2] = goal[2] - near[0] - near[1]
    far[2] = goal[2] - far[0] - far[1]
    for i in range(3):
        range_(near[i])
        range_(far[i])
    if np.linalg.norm(current - near) > np.linalg.norm(current - far):
        near, far = far, near
    return near, far, sol

# Function to solve the system
def solve(trels, current, near, far):
    trelb = np.zeros_like(trels)
    wrelt = np.zeros_like(trelb)
    wrelb = np.zeros_like(trelb)
    tmult(srelb, trels, trelb)
    wrelt = tinvert(wrelt)
    tmult(trelb, wrelt, wrelb)
    return invkin(wrelb, current, near, far)

# Function to compute the trajectory
def where(theta, trels):
    kin(theta, wrelb)
    tmult(wrelb, trelw, trelb)
    tmult(brels, trelb, trels)


# Function to compute cubic coefficients
def cub_coeff(th0, thf, thdot0, thdotf):
    s = 1.0  # Define the value of s
    cc = np.zeros(4)
    cc[0] = th0
    cc[1] = thdot0 / s
    cc[2] = 3 * (thf - th0) - 2 * thdot0 / s - thdotf / s
    cc[3] = -2 * (thf - th0) + thdotf / s + thdot0 / s
    return cc

# Function to compute joint velocities at via points
def joint_vel(viapnt, npnt, viavel):
    s = 1.0  # Define the value of s
    for j in range(3):
        viavel[0][j] = 0.0
        viavel[npnt][j] = 0.0
    if npnt > 2:
        for i in range(1, npnt - 1):
            for j in range(3):
                viavel[0][j] += 0.5 * ((viapnt[i][j] - viapnt[i - 1][j]) +
                                        (viapnt[i + 1][j] - viapnt[i][j])) * s

# Function to run the path
def run_path(path, npnt):
    deltat = 0.1  # Define the value of deltat
    s = 1.0  # Define the value of s
    time = 0.0
    i = 1
    nticks = round((npnt - 1) * 3 / deltat)
    print('path in Cartesian space (time, x, y, phi):')
    for k in range(1, nticks + 1):
        time += deltat
        tprime = time * s
        theta = [0.0] * 3
        for j in range(3):
            theta[j] = (path[i][j][0] + path[i][j][1] * tprime +
                        path[i][j][2] * tprime ** 2 + path[i][j][3] * tprime ** 3)
        where(theta, trels)
        itou(trels, trajectory[k - 1])
        print(f'{time:5.2f}', end='')
        for j in range(3):
            print(f'{trajectory[k - 1][j]:10.3f}', end='')
        print('')
        if time > 3.0:
            time = 0.0
            i += 1

# Function to plot the path
def plot_path_matplotlib():
    xscale, yscale = 30.0, 22.5
    ch = ['<', 'V', '>', '^']

    plt.figure(figsize=(8, 6))

    for k in range(1, nticks + 1):
        place = np.zeros(3)
        where(viapnt[k], trels)
        itou(trels, place)
        plt.scatter(place[0] * xscale + 40, -place[1] * yscale + 29, marker='*', color='red')

    for k in range(1, nticks + 1):
        plt.arrow(trajectory[k - 1][0] * xscale + 40, -trajectory[k - 1][1] * yscale + 29,
                0.5 * np.cos(trajectory[k - 1][2] * dtor),
                -0.5 * np.sin(trajectory[k - 1][2] * dtor),
                head_width=0.2, head_length=0.2, fc='blue', ec='blue')

    plt.xlim(0, 80)
    plt.ylim(0, 60)
    plt.gca().invert_yaxis()  # Invert y-axis to match the grid orientation
    plt.grid(True, linestyle='--', linewidth=0.5)
    plt.show()

utoi(srelbuser, srelb)
tinvert(srelb,brels)
utoi(trelwuser,trelw)
tinvert(trelw,wrelt)
npnt = 1
current = np.zeros(3)
input('Posição inicial: (x, y, phi): ')
place = np.array([float(val) for val in input().split()])
    utoi(place, trels)
    solve(trels, current, viapnt[npnt], far)
    while sol:
        npnt += 1

npnt -= 1
joint_vel(viapnt, npnt, viavel)
for i in range(1, npnt):
    for j in range(3):
        path[i][j] = cub_coeff(viapnt[i][j], viapnt[i + 1][j], viavel[i][j], viavel[i + 1][j])
    utoi(place, trels)
    solve(trels, viapnt[npnt - 1], viapnt[npnt], far)
npnt -= 1
joint_vel(viapnt, npnt, viavel)
for i in range(1, npnt):
    for j in range(3):
        path[i][j] = cub_coeff(viapnt[i][j], viapnt[i + 1][j], viavel[i][i], viavel[i + 1][j])

run_path(path, npnt)
plot_path_matplotlib()
