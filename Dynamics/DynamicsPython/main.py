# coding:UTF-8

'''
dynamics/equilibrium analysis
Build by Luo Xueling
'''

import os
import math
import pandas as pd
from matplotlib import pyplot as plt
import matplotlib.path as mpath
import matplotlib.patches as mpatches
import random
import shutil
import imageio
import numpy as np
import multiprocessing
from functools import partial
import sys
import tkinter as tk
from tkinter import filedialog

# 1000帧的动画大致使用10G内存，注意控制时间间隄1�7
slowrate = 1  # 动画减��1�7
plt.rcParams['figure.dpi'] = 300
mode = ''


def run():
    # bgProcess程序由c++编写，编译为exe
    bgProcess()
    if mode == 'dynamics':
        outputPNG()
        #outputMP4()
    elif mode == 'equilibrium':
        outputMP4()  # 实际上只会输出一张图牄1�7
    return "Python ends"


def bgProcess():
    # https://blog.csdn.net/weixin_33946605/article/details/85139653
    # https://cloud.tencent.com/developer/ask/115790
    selfpath = os.path.realpath(sys.argv[0]).replace("main.py", "")
    exe_path = selfpath + "Dynamics.exe"
    params = "\"" + root + inpname + " " + inpname + "\\\""
    with open('./runBgProcess.bat', 'w') as file:
        # 不生成新
        file.write("@echo off\n")
        file.write("powershell.exe -command ^\n")
        file.write("  \"start " + exe_path + " \\" + params + "\" -NoNewWindow -Wait")
    os.system('runBgProcess.bat')


# 做曲线图
def outputPNG():
    print("Python function (outputPNG) start")
    # 坐标轴刻度设罄1�7 https://www.jb51.net/article/164187.htm
    # df.plot https://blog.csdn.net/h_hxx/article/details/90635650
    # 子图的标题��标签设罄1�7 https://zhuanlan.zhihu.com/p/55675217
    # 输入文件路径预处琄1�7
    bodyNum = getBodyNum()
    fig = plt.figure(figsize=(15, 4 * bodyNum))
    for i in range(bodyNum):
        pospath = root + inpname + r'/' + inpname + "_OUT_body" + str(i) + "_pos.csv"
        velpath = root + inpname + r'/' + inpname + "_OUT_body" + str(i) + "_vel.csv"
        accpath = root + inpname + r'/' + inpname + "_OUT_body" + str(i) + "_acc.csv"
        path = {'Position': pospath, 'Velocity': velpath, 'Accelaration': accpath}
        labelname = []
        labelname.append([r'$x$', r'$y$', r'$phi$', r'$x,y(m)$', r'$\phi(rad)$'])
        labelname.append([r'$\mathrm{d}x/\mathrm{d}t$', r'$\mathrm{d}y/\mathrm{d}t$', r'$\mathrm{d}\phi/\mathrm{d}t$',
                          r'$V_x,V_y(m/s)$', r'$V_\phi(rad/s)$'])
        labelname.append(
            [r'$\mathrm{d}^2x/\mathrm{d}t^2$', r'$\mathrm{d}^2y/\mathrm{d}t^2$', r'$\mathrm{d}^2\phi/\mathrm{d}t^2$',
             r'$A_x,A_y(m/s^2)$', r'$A_\phi(rad/s^2)$'])
        keys = list(path.keys())

        for cnt in range(3):
            df = pd.read_csv(path[keys[cnt]], header=None, names=['t', 'x', 'y', 'phi'])
            df = df.set_index(df['t'].values)

            ax1 = fig.add_subplot(bodyNum, 3, 3 * i + cnt + 1)
            df['x'].plot(ax=ax1, label=labelname[cnt][0], color='Blue')
            df['y'].plot(ax=ax1, label=labelname[cnt][1], color='Red')
            ax1.set_ylabel(labelname[cnt][3])
            ax1.set_xlabel(r"$t(s)$")
            plt.legend(loc="upper left")

            ax2 = ax1.twinx()
            df['phi'].plot(ax=ax2, label=labelname[cnt][2], color='Green')
            ax2.set_ylabel(labelname[cnt][4])

            ymax = max(df['x'].max(), df['y'].max())
            ymin = min(df['x'].min(), df['y'].min())
            m = max(abs(ymax), abs(ymin))
            if m != 0:
                ax1.set_ylim([-1.5 * m, 1.5 * m])

            m = max(abs(df['phi'].max()), abs(df['phi'].min()))
            if m != 0:
                ax2.set_ylim([-1.5 * m, 1.5 * m])

            plt.legend(loc="upper right")
            plt.title('Body ' + str(i) + "- " + keys[cnt] + " plot")
            plt.xlabel('t/s')

    plt.tight_layout()
    fig.savefig(root + inpname + r'/' + inpname + "_OUT_Plot.png", dpi=300)

    df = pd.read_csv(root + inpname + r'/' + inpname + "_OUT_lambda.csv", header=None)
    fig = plt.figure(figsize=(6, 4 * (df.shape[1] - 1)))
    t = df.values[:, 0]
    for i in range(1, df.shape[1]):
        ax1 = fig.add_subplot(df.shape[1] - 1, 1, i)
        y = df.values[:, i]
        ax1.plot(t, y)
        ax1.set_ylabel(r'$\lambda_' + str(i) + r'(N)$')
        ax1.set_xlabel(r"$t(s)$")
    fig.tight_layout()
    fig.savefig(root + inpname + r'/' + inpname + "_OUT_lambda.png", dpi=300)
    print('outputPNG completed')


# MP4主程庄1�7
def outputMP4():
    # https://blog.csdn.net/nkhgl/article/details/103185573
    # https://blog.csdn.net/qq_28888837/article/details/85778395
    # https://vimsky.com/zh-tw/examples/detail/python-method-imageio.mimsave.html
    # https://www.cnpython.com/qa/458636

    print("Python function (outputMP4) start")

    bodyNum = getBodyNum()
    folder = root + inpname + r'/' + inpname + "_Anim//"

    # 读取数据文件
    df = []

    for i in range(bodyNum):
        pospath = root + inpname + r'/' + inpname + "_OUT_body" + str(i) + "_pos.csv"
        dfi = pd.read_csv(pospath, header=None, names=['t', 'x', 'y', 'phi'])
        dfi = dfi.set_index(dfi['t'].values)

        df.append(dfi)

    # 随机生成刚体颜色
    def randomClr():
        randclr = (random.random(), random.random(), random.random())
        return randclr

    clr = []
    for body in range(bodyNum): clr.append(randomClr())

    # 读取形状数据
    shape = readshape(shapefile)

    # 获取各时间点坐标信息
    # points[t]=[[每个刚体x],[每个刚体y]]
    points = []
    totaltime = df[0]['t'].values[-1]
    n = len(df[0]['t'].values)
    for t in range(n):
        tmp = [[], []]
        for body in range(bodyNum):
            x1 = df[body]['x'].values[t]
            y1 = df[body]['y'].values[t]
            tmp[0].append(x1)
            tmp[1].append(y1)
        points.append(tmp)

    if os.path.exists(folder):
        shutil.rmtree(folder)

    print("Plotting image.")
    os.makedirs(folder)

    # 准备patches填入图像
    patches, all_points = draw_body(df, points, shape, clr, n)

    # 找到坐标朢�倄1�7
    all_x = []
    all_y = []
    for point in all_points:
        all_x.append(point[0])
        all_y.append(point[1])
    xmin = min(all_x)
    xmax = max(all_x)
    ymin = min(all_y)
    ymax = max(all_y)

    # 调整比例
    xl = xmin
    yl = ymin
    xr = xmax
    yr = ymax
    d = 1.3333 * (ymax - ymin) / (xmax - xmin)
    if d > 1:
        xl = xl * d - 0.5 * (xr * d - xmax)
        xr = xr * d - 0.5 * (xr * d - xmax)
    else:
        yl = yl / d - 0.5 * (yr / d - ymax)
        yr = yr / d - 0.5 * (yr / d - ymax)

    # 读取约束信息,并画约束
    conspatches = draw_constraints(df, bodyNum, n, xr, xl, yr, yl, inpFilePath)

    # https://www.itranslater.com/qa/details/2582631824283403264
    cores = multiprocessing.cpu_count()
    pool = multiprocessing.Pool(processes=math.floor(cores))
    tlist = np.linspace(0, n - 1, n, dtype=int)
    poolableFunc = partial(threadPng, df, bodyNum, folder, patches, conspatches, n, xl, xr, yl, yr)
    pool.map(poolableFunc, tlist)

    # 输出mp4
    if mode == 'dynamics':
        print("\nSaving mp4")
        frames = []
        for t in range(n):
            frames.append(imageio.imread(folder + str(t) + ".png"))
        # imageio.mimsave(root + inpname + '_output.gif', frames, 'GIF', duration=totaltime/10/n)
        imageio.mimsave(root + inpname + r'/' + inpname + '_output.mp4', frames, 'MP4', fps=n / totaltime / slowrate)
        shutil.rmtree(folder)

    print('\noutputMP4 completed')


# MP4中生成Png的函数，用于多线程运衄1�7
def threadPng(df, bodyNum, folder, patches, conspatches, n, xl, xr, yl, yr, t):
    fig, ax = plt.subplots()
    currenttime = df[0]['t'].values[t]
    ax.grid('on')
    # 作刚佄1�7
    for body in range(bodyNum):
        ax.add_patch(patches[t][body])

    # 作铰
    for patch in conspatches[t]:
        ax.add_patch(patch)

    ax.set_title("Time = " + str(round(currenttime, 5)) + " s")
    ax.set_ylim([yl, yr])
    ax.set_xlim([xl, xr])
    # 保存并记录图僄1�7
    fig.savefig(folder + str(t) + ".png", dpi=300)
    update_slider(len(os.listdir(folder)), n)
    # print("Image " + str(len(os.listdir(folder))) + "/" + str(n) + " has been plotted")
    fig.clf()
    plt.close(fig)


def readshape(shapefile):
    shape = []
    with open(shapefile, 'r')as file:
        bodyNum = file.readline()
        bodyNum = int(bodyNum.replace("\n", ""))
        for body in range(bodyNum):
            ax = plt.gca()
            type = file.readline().replace("\n", "")
            if type == "rectangle":
                p1 = file.readline().replace("\n", "")
                p3 = file.readline().replace("\n", "")

                p1 = [float(x) for x in p1.split(',')]
                p3 = [float(x) for x in p3.split(',')]
                p2 = [p1[0], p3[1]]
                p4 = [p3[0], p1[1]]
                shape.append(["rectangle", [p1, p2, p3, p4]])
            elif type == 'circle':
                p1 = file.readline().replace("\n", "")
                r = file.readline().replace("\n", "")

                p1 = [float(x) for x in p1.split(',')]
                r = float(r)
                shape.append(["circle", [p1, [r]]])
            elif type == 'any-shape':
                p = []
                pnum = int(file.readline().replace("\n", ""))
                for j in range(pnum):
                    p1 = file.readline().replace("\n", "")
                    p1 = [float(x) for x in p1.split(',')]
                    p.append(p1)
                shape.append(["any-shape", p])
    return shape


def draw_body(df, points, shape, clr, n):
    all_points = []
    patches = []
    for t in range(n):
        patches.append([])
        # 作刚佄1�7
        for body in range(bodyNum):
            phi = df[body]['phi'].values[t]
            x = points[t][0][body]
            y = points[t][1][body]
            if shape[body][0] == "rectangle":
                p0 = []
                path_data = []
                pnum = 4
                Path = mpath.Path
                for j in range(pnum):
                    pk = shape[body][1][j]
                    x0 = x + math.cos(phi) * pk[0] - math.sin(phi) * pk[1]
                    y0 = y + math.sin(phi) * pk[0] + math.cos(phi) * pk[1]
                    pk = [x0, y0]
                    all_points.append(pk)
                    if j == 0:
                        path_data.append((Path.MOVETO, pk))
                        p0 = pk
                    else:
                        path_data.append((Path.LINETO, pk))
                path_data.append((Path.CLOSEPOLY, p0))
                codes, verts = zip(*path_data)
                path = mpath.Path(verts, codes)
                patch = mpatches.PathPatch(path, color=clr[body], lw=0)
                patches[t].append(patch)

            elif shape[body][0] == "circle":
                pk = shape[body][1][0]
                r = shape[body][1][1][0]
                x0 = x + math.cos(phi) * pk[0] - math.sin(phi) * pk[1]
                y0 = y + math.sin(phi) * pk[0] + math.cos(phi) * pk[1]
                all_points.append(pk)
                all_points.append([pk[0] + r, pk[1] + r])
                all_points.append([pk[0] - r, pk[1] - r])
                circle = mpatches.Circle((x0, y0), radius=r, color=clr[body], lw=0)
                circle.set_zorder(0)
                patches[t].append(circle)
            elif shape[body][0] == "any-shape":
                p0 = []
                path_data = []
                pnum = len(shape[body][1])
                Path = mpath.Path
                for j in range(pnum):
                    pk = shape[body][1][j]
                    x0 = x + math.cos(phi) * pk[0] - math.sin(phi) * pk[1]
                    y0 = y + math.sin(phi) * pk[0] + math.cos(phi) * pk[1]
                    pk = [x0, y0]
                    all_points.append(pk)
                    if j == 0:
                        path_data.append((Path.MOVETO, pk))
                        p0 = pk
                    else:
                        path_data.append((Path.LINETO, pk))
                path_data.append((Path.CLOSEPOLY, p0))
                codes, verts = zip(*path_data)
                path = mpath.Path(verts, codes)
                patch = mpatches.PathPatch(path, color=clr[body], lw=0)
                patches[t].append(patch)
    return patches, all_points


def draw_constraints(df, bodyNum, n, xr, xl, yr, yl, inpFilePath):
    conspatches = []
    constype = []
    consb = []
    conss = []
    for t in range(n):
        conspatches.append([])
    with open(inpFilePath, 'r')as file:
        file.readline()
        file.readline()
        file.readline()
        for i in range(bodyNum):
            file.readline()
        file.readline()
        line = file.readline()
        line = line.split(' = ')
        consNum = int(line[-1].replace("\n", ""))
        for i in range(consNum):
            line = file.readline().replace("\n", "")
            line = line.split('\t')
            type = line[0]
            b = int(line[1])
            s = line[2].split(' ')
            s = [float(x) for x in s]
            constype.append(type)
            consb.append(b)
            conss.append(s)
    # 记录每个时间点被约束的点,及约束该点的约束类型
    conspoint = []
    type_for_conspoint = []
    for t in range(n):
        conspoint.append([])
        type_for_conspoint.append([])
        for i in range(consNum):
            type = constype[i]
            b = consb[i]
            s = conss[i]
            x = df[b]['x'].values[t]
            y = df[b]['y'].values[t]
            phi = df[b]['phi'].values[t]
            x_ = x + s[0] * math.cos(phi) - s[1] * math.sin(phi)
            y_ = y + s[0] * math.sin(phi) + s[1] * math.cos(phi)
            if len(conspoint[t]) > 0:
                for j in range(len(conspoint[t])):
                    # 如果该点已有其他约束
                    if abs(x_ - conspoint[t][j][0]) < 1e-5 and abs(y_ - conspoint[t][j][1]) < 1e-5:
                        type_for_conspoint[t][j].append(type)
                        break
                    # 如果尚未被约杄1�7
                    if j == len(conspoint[t]) - 1:
                        conspoint[t].append([x_, y_])
                        type_for_conspoint[t].append([type])
            else:
                conspoint[t].append([x_, y_])
                type_for_conspoint[t].append([type])

    for t in range(n):
        for j in range(len(conspoint[t])):
            x_ = conspoint[t][j][0]
            y_ = conspoint[t][j][1]
            types = type_for_conspoint[t][j]
            r = (xr - xl) * 0.004 * math.log(bodyNum + 5)
            if 'ax' in types and 'ay' not in types and 'aphi' not in types:
                r = (xr - xl) * 0.004 * math.log(bodyNum + 5)
                circle = mpatches.Circle((x_, y_), r, ec="blue", fc='white')
                circle.set_zorder(1)

                Path = mpath.Path
                path_data = [(Path.MOVETO, [x_, y_]),
                             (Path.LINETO, [x_ - 3 * r, y_ - math.sqrt(3) * r]),
                             (Path.LINETO, [x_ - 3 * r, y_ + math.sqrt(3) * r]),
                             (Path.CLOSEPOLY, [x_, y_])
                             ]
                circle2 = mpatches.Circle((x_ - 4 * r, y_), r, ec="blue", fc='white')
                codes, verts = zip(*path_data)
                path = mpath.Path(verts, codes)
                patch = mpatches.PathPatch(path, color="blue", lw=0)
                patch.set_zorder(0.5)

                conspatches[t].append(circle)
                conspatches[t].append(patch)
                conspatches[t].append(circle2)
            elif 'ay' in types and 'ax' not in types and 'aphi' not in types:
                r = (xr - xl) * 0.004 * math.log(bodyNum + 5)
                circle = mpatches.Circle((x_, y_), r, ec="blue", fc='white')
                circle.set_zorder(1)

                Path = mpath.Path
                path_data = [(Path.MOVETO, [x_, y_]),
                             (Path.LINETO, [x_ - math.sqrt(3) * r, y_ - 3 * r]),
                             (Path.LINETO, [x_ + math.sqrt(3) * r, y_ - 3 * r]),
                             (Path.CLOSEPOLY, [x_, y_])
                             ]
                circle2 = mpatches.Circle((x_, y_ - 4 * r), r, ec="blue", fc='white')
                codes, verts = zip(*path_data)
                path = mpath.Path(verts, codes)
                patch = mpatches.PathPatch(path, color="blue", lw=0)
                patch.set_zorder(0.5)

                conspatches[t].append(circle)
                conspatches[t].append(patch)
                conspatches[t].append(circle2)
            elif 'ay' in types and 'ax' in types and 'aphi' not in types:
                circle = mpatches.Circle((x_, y_), r, ec="blue", fc='white')
                circle.set_zorder(1)

                Path = mpath.Path
                path_data = [(Path.MOVETO, [x_, y_]),
                             (Path.LINETO, [x_ - math.sqrt(3) * r, y_ - 3 * r]),
                             (Path.LINETO, [x_ + math.sqrt(3) * r, y_ - 3 * r]),
                             (Path.CLOSEPOLY, [x_, y_])
                             ]
                codes, verts = zip(*path_data)
                path = mpath.Path(verts, codes)
                patch = mpatches.PathPatch(path, color="blue", lw=0)
                patch.set_zorder(0.5)
                conspatches[t].append(circle)
                conspatches[t].append(patch)
            elif 'ax' in types and 'aphi' in types and 'ay' not in types:
                rec1 = mpatches.Rectangle((x_ - 4 * r, y_ - (yr - yl)), 0.5 * r, 2 * (yr - yl), color='blue',
                                          alpha=0.1)
                rec1.set_zorder(1)
                rec2 = mpatches.Rectangle((x_ + 3.5 * r, y_ - (yr - yl)), 0.5 * r, 2 * (yr - yl), color='blue',
                                          alpha=0.1)
                rec2.set_zorder(1)
                conspatches[t].append(rec1)
                conspatches[t].append(rec2)
            elif 'ay' in types and 'aphi' in types and 'ax' not in types:
                rec1 = mpatches.Rectangle((x_ - (xr - xl), y_ - 4 * r), 2 * (xr - xl), 0.5 * r, color='blue',
                                          alpha=0.1)
                rec1.set_zorder(1)
                rec2 = mpatches.Rectangle((x_ - (xr - xl), y_ + 3.5 * r), 2 * (xr - xl), 0.5 * r, color='blue',
                                          alpha=0.1)
                rec2.set_zorder(1)
                conspatches[t].append(rec1)
                conspatches[t].append(rec2)
            if 'r' in types or 'rt' in types:
                circle = mpatches.Circle((x_, y_), r, ec="blue", fc='white')
                circle.set_zorder(1)
                conspatches[t].append(circle)

    return conspatches


# 路径预处琄1�7
def pathPreProcess(inpFilePath):
    # 字符串预处理
    st = inpFilePath.replace("\\", "/")
    st = st.split("/")
    root = ""
    for part in range(len(st) - 1):
        root += (st[part])
        root += (r"/")
    # 含后缢�各1�7
    filename = st[-1]
    # 不含后缀各1�7
    inpname = filename.split(".")[0]
    return root, filename, inpname


def getBodyNum():
    with open(inpFilePath, 'r')as file:
        file.readline()
        file.readline()
        bodyNum = int(file.readline().split()[-1])

    return bodyNum


def update_slider(a, b):
    print("\r[", end='')
    for i in range(math.floor(a * 20 / b)):
        print(">", end='')
    for i in range(20 - math.floor(a * 20 / b)):
        print("-", end='')
    print("]", end='')


if __name__ == "__main__":
    print("Python start")
    tkroot = tk.Tk()
    inpFilePath = tk.filedialog.askopenfilename(title="Choose input file", filetype=[('文本文件', '.txt')],
                                                initialdir=os.getcwd())
    with open(inpFilePath, 'r')as file:
        mode = file.readline().replace('\n', '')
    shapefile = tk.filedialog.askopenfilename(title="选择形状文件", filetype=[('文本文件', '.txt')], initialdir=inpFilePath)
    tkroot.destroy()
    root, filename, inpname = pathPreProcess(inpFilePath)
    bodyNum = getBodyNum()
    run()
