import tkinter as tk
from tkinter import filedialog
import os

# 单刚体约束
singBodyCons = ['ax', 'ay', 'aphi', 'ad', 'axd', 'ayd', 'aphid', 'add']
# 双刚体约束
multiBodyCons = ['rphi', 'rd', 't', 'r', 'rt', 'rphid', 'rrd', 'rdd', 'tdd']
# 需要输入v的约束
vCons = ['t', 'rt', 'tdd']  # rt,tdd需要b1的v
# 不需要输入s的约束
slessCons = ['aphi', 'rphi', 'aphid', 'rphid', 'rrd']
# 需要C的约束
CCons = ['add', 'ad']
# 存在c，且c为常数的约束
constCons = ['ay', 'ax', 'aphi', 'ad', 'rphi', 'rd', 'rt']
# 存在c，且c为函数的约束
funcCons = ['axd', 'ayd', 'aphid', 'add', 'rphid', 'rrd', 'rdd', 'tdd']


def run():
    root = tk.Tk()
    filename = tk.filedialog.asksaveasfilename(title="选择文件保存路径", filetype=[('文本文件', '.txt')], initialdir=os.getcwd())
    root.destroy()
    with open(filename, 'w')as file:
        bodyNum = int(input("Input the number of bodys:"))
        file.write("Number of rigidbodys = " + str(bodyNum) + '\n')
        for i in range(bodyNum):
            x = input('Initial x of body ' + str(i) + ":")
            y = input('Initial y of body ' + str(i) + ":")
            phi = input('Initial phi of body ' + str(i) + ":")
            file.write(x + "\t" + y + "\t" + phi + "\n")
        file.write('\n')
        consNum = int(input("Input the number of constraints:"))
        file.write("Number of constraints = " + str(consNum) + '\n')
        for i in range(consNum):
            type = input('Input the type of constraint ' + str(i+1) + ":")
            file.write(type + '\t')
            if type in slessCons:
                b1 = input('Body 1 (start from 0):')
                s1 = '0 0'
                v1 = '0 0'
            elif type not in vCons:
                b1 = input('Body 1 (start from 0):')
                s1 = input('s1\'^T:').replace(",", " ")
                v1 = '0 0'
            else:
                b1 = input('Body 1 (start from 0):')
                s1 = input('s1\'^T:').replace(",", " ")
                v1 = input('v1\'^T:').replace(",", " ")
            file.write(b1 + '\t' + s1 + '\t' + v1 + '\t')
            if type in multiBodyCons:
                if type == 'rt' or type=='tdd':
                    b2 = input('Body 2 (start from 0):')
                    s2 = input('s2\'^T:').replace(",", " ")
                    v2 = '0 0'
                elif type in slessCons:
                    b2 = input('Body 2 (start from 0):')
                    s2 = '0 0'
                    v2 = '0 0'
                elif type not in vCons:
                    b2 = input('Body 2 (start from 0):')
                    s2 = input('s2\'^T:').replace(",", " ")
                    v2 = '0 0'
                else:
                    b2 = input('Body 2 (start from 0):')
                    s2 = input('s2\'^T:').replace(",", " ")
                    v2 = input('v2\'^T:').replace(",", " ")
                file.write(b2 + '\t' + s2 + '\t' + v2 + '\t')
            else:
                file.write("\t\t\t")
            if type in constCons:
                c = input('Input c:')
                file.write("1" + ' ' + c + '\t')
            elif type in funcCons:
                n = input('Input the number of terms of polynome c(t):')
                file.write(n)
                for i in range(int(n)):
                    file.write(" ")
                    a = input('Input a' + str(i) + ' for a' + str(i) + 'x^' + str(i) + ":")
                    file.write(a)
                file.write("\t")
            if type in CCons:
                C = input("Fix point for type " + type + ":").replace(",", " ")
                file.write(C)
            file.write('\n')
        file.write('\n')
        t0 = input("Initial time t0:")
        te = input("End time te:")
        dt = input("Time inteval dt:")
        e1 = input("Limit for norm(PHI):")
        e2 = input("Limit for abs(det(PHIq)):")
        file.write("t0 = " + t0 + '\n')
        file.write("te = " + te + '\n')
        file.write("dt = " + dt + '\n')
        file.write("e1 = " + e1 + '\n')
        file.write("e2 = " + e2)

    return filename


if __name__ == "__main__":
    run()
