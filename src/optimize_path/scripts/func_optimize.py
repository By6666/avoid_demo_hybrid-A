#!/usr/bin/env python3

import matplotlib.pyplot as plt  # for show figure
import scipy.linalg as sl  # for linear alogorithm
import numpy as np  # for scipy
'''
***** polyfix function *****
* Fits polynomial p with degree n to data,
  but specify value at specific points

* input: n -> the degree of polynomial
        x, y -> fit points list
        xfix, yfix -> pass points list
        xder, dydx -> derivative construction list

* output: p -> polynomial coefficient shape=(n, 1)

* tip: need numpy & scipy
'''


def polyfix(x, y, n, xfix, yfix, xder=[], dydx=[]):

    nfit = len(x)
    if len(y) != nfit:
        raise ValueError('x and y must have the same size')

    nfix = len(xfix)
    if len(yfix) != nfix:
        raise ValueError('xfit adn yfit must have the same size')

    # transform list to col vector
    x = np.vstack(x)
    y = np.vstack(y)
    xfix = np.vstack(xfix)
    yfix = np.vstack(yfix)

    # if derivatives are specified:
    if len(xder) != 0:
        xder = np.array([xder]).reshape(-1, 1)
        dydx = np.array([dydx]).reshape(-1, 1)

    nder = len(xder)
    if len(dydx) != nder:
        raise ValueError('xder and dydx must have same size')

    nspec = nfix + nder
    specval = np.vstack((yfix, dydx))

    # first find A and pc such that A*pc = specval
    A = np.zeros((nspec, n + 1))
    # specified y values
    for i in range(n + 1):
        A[:nfix, i] = np.hstack(np.ones((nfix, 1)) * xfix**(n + 1 - (i + 1)))
    if nder > 0:
        for i in range(n):
            A[nfix:nder + nfix, i] = ((n - (i + 1) + 1) * np.ones(
                (nder, 1)) * xder**(n - (i + 1))).flatten()
    if nfix > 0:
        lastcol = n + 1
        nmin = nspec - 1
    else:
        lastcol = n
        nmin = nspec

    if n < nmin:
        raise ValueError(
            'Polynomial degree too low, cannot match all constraints')
    # find unique polynomial of degree nmin that fits the constraints
    firstcol = n - nmin
    pc0 = np.linalg.solve(A[:, firstcol:lastcol], specval)
    pc = np.zeros((n + 1, 1))
    pc[firstcol:lastcol] = pc0

    X = np.zeros((nfit, n + 1))
    for i in range(n + 1):
        X[:, i] = (np.ones((nfit, 1)) * x**(n + 1 - (i + 1))).flatten()

    yfit = y - np.polyval(pc, x)

    B = sl.null_space(A)
    z = np.linalg.lstsq(X @ B, yfit, rcond=None)[0]

    if len(z) == 0:
        return pc.flatten()
    else:
        p0 = B @ z

    p = p0 + pc
    return p.flatten()


'''
***** path optimize function *****
* segmentation path to fit

* input: fit_degree -> the degree of polynomial
        path_seg_num -> segmentation points
        prime_path -> prime path points list
        path_resolution -> final path resolution
        show_flg -> whether show figure
 
* output: final_path -> list[x, y, yaw]

'''


def path_generation(fit_degree,
                    path_seg_num,
                    prime_path,
                    path_resolution=20,
                    show_flg=False):
    path = np.array(prime_path).reshape(-1, 3)
    path_length = np.shape(path)[0]

    path_cnt = 0
    seg_list = [path_cnt]

    if path_length / path_seg_num >= 2:
        while True:
            path_cnt = path_cnt + path_seg_num - 1
            seg_list.append(path_cnt)
            if (path_length - path_cnt) < 2 * path_seg_num - 1:
                break
            else:
                pass
    else:
        pass
    seg_list.append(path_length - 1)
    # print(seg_list)

    x_dis = []
    y_dis = []
    yaw_dis = []
    # p_list = []
    # p_der_list = []

    for i in range(0, len(seg_list) - 1):
        lf = seg_list[i]
        rg = seg_list[i + 1]

        x = np.linspace(path[lf, 0], path[rg, 0], path_resolution)
        p = polyfix(path[lf:rg + 1,
                         0], path[lf:rg + 1,
                                  1], fit_degree, [path[lf, 0], path[rg, 0]],
                    [path[lf, 1], path[rg, 1]], [path[lf, 0], path[rg, 0]],
                    [np.tan(path[lf, 2]), np.tan(path[rg, 2])])
        p_der = np.polyder(p)
        # p_list.append(p)
        # p_der_list.append(p_der)
        if i == 0:
            x_dis.extend(x.tolist())
            y_dis.extend(np.polyval(p, x).tolist())
            yaw_dis.extend(np.polyval(p_der, x).tolist())
        else:
            x_dis.extend(x.tolist()[1:])
            y_dis.extend(np.polyval(p, x).tolist()[1:])
            yaw_dis.extend(np.polyval(p_der, x).tolist()[1:])

    if show_flg:
        plt.plot(x_dis, y_dis, '-', c='r')
        plt.plot(x_dis, yaw_dis, '-.', c='b')
        plt.plot(path[:, 0], path[:, 1], 'o', mfc='w')
        plt.axis([-10, 60, -10, 10])
        plt.show()

    return x_dis + y_dis + yaw_dis


# for test
if __name__ == '__main__':

    # path = [
    #     0, 0, 0, 2.87979, 0, 0, 5.75959, 0, 0, 8.63116, 0.188213, 0.1309,
    #     11.4536, 0.749632, 0.261799, 14.2353, 1.49498, 0.261799, 16.9603, 2.42,
    #     0.392699, 19.6853, 3.34501, 0.261799, 22.5077, 3.90643, 0.1309,
    #     25.3793, 4.09465, 0, 28.2509, 3.90643, -0.1309, 31.0733, 3.34501,
    #     -0.261799, 33.8958, 2.7836, -0.1309, 36.7509, 2.40771, -0.1309,
    #     39.5734, 1.84629, -0.261799, 42.3958, 1.28487, -0.1309, 45.251,
    #     0.908981, -0.1309, 48.1061, 0.533092, -0.1309, 50, 0, 0
    # ]

    path = [
        50, 0, 3.14, 47.1202, 0.00458651, 3.14, 44.2404, 0.00917302, 3.14,
        41.3606, 0.0137595, 3.14, 38.4808, 0.018346, 3.14, 35.6096, 0.211132,
        3.0091, 32.788, 0.777046, 2.8782, 30.0075, 1.52682, 2.8782, 27.2271,
        2.27659, 2.8782, 24.4466, 3.02637, 2.8782, 21.6251, 3.59228, 3.0091,
        18.7538, 3.78507, 3.14, 15.8819, 3.60143, -3.01229, 13.0586, 3.04451,
        -2.88139, 10.2757, 2.30359, -2.88139, 7.54924, 1.38292, -2.75049,
        4.82275, 0.462238, -2.88139, 1.99942, -0.094685, -3.01229, 0, 0, 3.14
    ]

    path_generation(3, 6, path, 20, True)
