import tkinter as tk
from tkinter import filedialog
import os

def run():
    root = tk.Tk()
    filename = tk.filedialog.asksaveasfilename(title="选择文件保存路径", filetype=[('文本文件', '.txt')], initialdir=os.getcwd())
    root.destroy()
    with open(filename, 'w')as file:
        bodyNum = int(input("Input the number of bodys:"))
        file.write(str(bodyNum) + '\n')
        for body in range(bodyNum):
            type = input("Input the type of body " + str(body + 1) + " (rectangle, circle or any-shape):")
            file.write(type + '\n')
            if type == "rectangle":
                print("Suggested height of rectangle:0.06")
                p1 = input("Point 1 of 2:")
                p2 = input("Point 2 of 2:")
                p1.replace(" ", ",")
                p2.replace(" ", ",")
                p1 = p1.split('\n')[0]
                p2 = p2.split('\n')[0]

                file.write(p1 + '\n' + p2 + '\n')
            elif type == 'circle':
                p1 = input("Center:")
                r = input("Radius:")
                p1.replace(" ", ",")
                p1 = p1.split('\n')[0]
                r = r.split('\n')[0]
                file.write(p1 + '\n' + r + '\n')
            elif type == 'any-shape':
                path_data = []
                p1 = input("Point 1:")
                cnt = 0
                while not p1 == "end":
                    p1.replace(" ", ",")
                    p1 = p1.split('\n')[0]
                    path_data.append(p1)
                    cnt = cnt + 1
                    p1 = input("Point " + str(cnt) + ": (Type \"end\" to end input)")
                file.write(str(cnt) + '\n')
                for p in path_data:
                    file.write(p + "\n")
            input("Press any key to continue input")
    return filename


if __name__ == "__main__":
    run()
