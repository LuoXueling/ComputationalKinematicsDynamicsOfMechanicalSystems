# Computational Kinematics and Dynamics of Mechanical Systems

This a coursework in 2021 when I was an undergraduate student at Shanghai Jiao Tong University. It implements two dimensional computation of kinematics and dynamics of mechanical systems, including two-link mechanism, crank-slider mechanism, and others. I implemented some matrix calculations and classes of constraints in C++ myself. 

## Requirements

C++ is used for computation and Python is used for visualization. I used Visual Studio 2019 and Pycharm 2020.2 as IDEs (so you can find .sln files in the repository). For python, I used Python 3.7.6 with an Anaconda environment (But I cannot find exact versions of packages I used since it was five years ago). 

Some code files may be encoded by GB2312 because of some Chinese contents. 

## Usage

I assume that users of the repository have basic knowledge about computational kinematics/dynamics so are familiar with constraint equations and solutions. 

### For kinematics

One can find two examples (INP0 and INP2 in `KinematicsExer`) of questions, their descriptions of constraints, detailed velocity and accelaration equations, flow chart, and results in `Kinematics/Report.docx`.

1. Use `create_INPfile.py` to generate a input file (such as `INP*.txt` in the repository) for rigid bodies and constraints. See the .docx file, `INP0.txt`, and `INP2.txt` for descriptions of constraints. 
2. Use `create_bodyshape.py` to generate a file for geometries of objects (such as `INP*_shape.txt` in the repository). 
3. Run `plot.py` to select the input file and geometry file. 
4. `plot.py` then calls `Kinematics.exe` complied from `KinematicsC++` that generates outputs, and plot the outputs, including positions, velocities, and accelarations of objects, charts, and a mp4 file. 

No space is allowed in filepaths.

### For dynamics

Two examples (INP4 and INP5 in `DynamicsExer`) are reported in `Dynamics/Report.docx` containing descriptions of equations and results. 

1. Write an input file (such as `INP*.txt` in the repository) for rigid bodies, constraints, and forces. See `Dynamics/Report.docx`, `INP4.txt`, and `INP5.txt` for examples. 

2. Use `create_bodyshape.py` to generate a file for geometries of objects (such as `INP*_shape.txt` in the repository). 
3. Run `main.py` to select the input file and geometry file.
4. `main.py` then calls `Dynamics.exe` complied from `DynamicsC++` that generates outputs, and plot the outputs, including positions, velocities, accelerations, and Lagrangian multipliers of objects, charts, and a mp4 file.
