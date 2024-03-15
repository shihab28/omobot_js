#!/usr/bin/python3

import numpy as np
from scipy.optimize import curve_fit
import pandas as pd
import matplotlib.pyplot as plt

# csvFilePath = 'src/mobot_pkg/extra/pwmX_vs_W_2.csv'
csvFilePath = 'src/mobot_pkg/extra/pwmX_vs_W.csv'

pwmSpeedIn = []
encSpeed_p = []
encSpeed_n = []

ab_p = []
var_p = []

ab_n = []
var_n = []

popt = []
pcov = []

SL = 70.6549
Rd = 39.5

csvDF = pd.read_csv(csvFilePath)
values = csvDF.values

def saveCsv2(filePath, x, y):
    tempData = ''
    for i in range(len(x)): 
        tempData += f"{x[i]},{y[i]}\t\t\t\t{y[i]},{x[i]}\n"
    
    with open(filePath, 'w') as wf:
        wf.write(tempData)

def fittingFunc(x, a, b):
    return b + a/x

def ReverseFitFunc(x, a, b):
    return a / (x - b)

def processData(csvFilePath):
    xp = []
    xn = []
    yp = [[],[],[],[]]
    yn = [[],[],[],[]]
    
    csvDF = pd.read_csv(csvFilePath)
    values = csvDF.values

    for i in range(len(values)):
        x_temp = values[i][0]

        if x_temp < 0:
            xn.append(x_temp)
            yn[0].append(values[i][1])
            yn[1].append(values[i][2])
            yn[2].append(values[i][3])
            yn[3].append(values[i][4])
        
        else:
            xp.append(x_temp)
            yp[0].append(values[i][1])
            yp[1].append(values[i][2])
            yp[2].append(values[i][3])
            yp[3].append(values[i][4])

    return xp, yp, xn, yn

def getParameters(fittingFunc, pwmSpeed_p, encSpeed_p, pwmSpeed_n, encSpeed_n):
    
    for i in range(4):
        pop, pcp = curve_fit(fittingFunc, pwmSpeed_p, encSpeed_p[i])
        # pop, pcp = curve_fit(ReverseFitFunc, encSpeed_p[i], pwmSpeed_p)
        # saveCsv2(f'x_yp{i+1}.csv', encSpeed_p[i], pwmSpeed_p)
        ab_p.append(np.around(pop, 5))
        var_p.append(np.around(pcp, 5))

        pon, pcn = curve_fit(fittingFunc, pwmSpeed_n, encSpeed_n[i])
        # pon, pcn = curve_fit(ReverseFitFunc, encSpeed_n[i], pwmSpeed_n)
        # saveCsv2(f'x_yn{i+1}.csv', encSpeed_n[i], pwmSpeed_n)
        ab_n.append(np.around(pon, 5))
        var_n.append(np.around(pcn, 5))
    
    return ab_p, var_p, ab_n, var_n


# print("ab_p  : ",  ab_p)
# print("ab_n  : ",  ab_n)
# print("var_p : ", var_p)
# print("var_n : ", var_n)

plots = []
pwmSpeed_p, encSpeed_p, pwmSpeed_n, encSpeed_n = processData(csvFilePath)
    # fitFunc = ReverseFitFunc
ab_p, var_p, ab_n, var_n = getParameters(fittingFunc, pwmSpeed_p, encSpeed_p, pwmSpeed_n, encSpeed_n)

equation = 'Forward : '
Display =True
for i in range(4):
    if Display:
        plot_ = plt.plot(encSpeed_p[i], pwmSpeed_p, 'b', label=f'Speed{i}')
        plots.append(plot_)
        fitY = ReverseFitFunc(encSpeed_p[i], *ab_p[i])
        plt.plot(encSpeed_p[i], fitY, 'r', label=f'fittedSpeed{i}')
        equation += f"\nyp_{i+1} = {ab_p[i][0]}/x + ({ab_p[i][1]})\n"


        plt.plot(encSpeed_n[i], pwmSpeed_n, 'b', label=f'Speed{i}')
        fitY =  ReverseFitFunc(encSpeed_n[i], *ab_n[i])

        plt.plot(encSpeed_n[i], fitY, 'r', label=f'fittedSpeed{i}')
        equation += f"yn_{i+1} = {ab_n[i][0]}/x + ({ab_n[i][1]})\n"


        plt.xticks( np.arange(-4, 4, .5))
        plt.yticks( np.arange(-260, 260, 50))
        plt.grid(True, which='both')

        plt.ylabel(f'PWM_w{i+1}', size = 16)
        plt.xlabel('encSpeed (mm/sec)', size = 16)
        plt.legend(['True Data', 'Fitted Data'], fontsize="12")
        plt.title(f'Model Fitting for motor {i+1}', size = 20)

        plt.show()
    else:
        fitY = ReverseFitFunc(encSpeed_p[i], *ab_p[i])
        equation += f"\nyp_{i+1} = {ab_p[i][0]}/x + ({ab_p[i][1]})\n"
        fitY =  ReverseFitFunc(encSpeed_n[i], *ab_n[i])
        equation += f"yn_{i+1} = {ab_n[i][0]}/x + ({ab_n[i][1]})\n"


equation += '\n\nReverse : '
for i in range(4):
    equation += f"\nyp_{i+1} = {ab_p[i][0]} / (x - ({ab_p[i][1]}))\n"
    equation += f"yn_{i+1} = {ab_n[i][0]} / (x - ({ab_n[i][1]}))\n"

equation += f'''\n
ap = [{ab_p[0][0]}, {ab_p[1][0]}, {ab_p[2][0]}, {ab_p[3][0]}]
bp = [{ab_p[0][1]}, {ab_p[1][1]}, {ab_p[2][1]}, {ab_p[3][1]}]
an = [{ab_n[0][0]}, {ab_n[1][0]}, {ab_n[2][0]}, {ab_n[3][0]}]
bn = [{ab_n[0][1]}, {ab_n[1][1]}, {ab_n[2][1]}, {ab_n[3][1]}]\n
'''

equation += f'''\n
ap1 = {ab_p[0][0]}, bp1 = {ab_p[0][1]}, an1 = {ab_n[0][0]}, bn1 = {ab_n[0][1]}, ap2 = {ab_p[1][0]}, bp2 = {ab_p[1][1]}, an2 = {ab_n[1][0]}, bn2 = {ab_n[1][1]}, ap3 = {ab_p[2][0]}, bp3 = {ab_p[2][1]}, an3 = {ab_n[2][0]}, bn3 = {ab_n[2][1]}, ap4 = {ab_p[3][0]}, bp4 = {ab_p[3][1]}, an4 = {ab_n[3][0]}, bn4 = {ab_n[3][1]};
'''

equation = equation.strip()




equation += f'''\n****Variance****
var_ap1 = {var_p[0][0]}, var_bp1 = {var_p[0][1]}, 
var_an1 = {var_n[0][0]}, var_bn1 = {var_n[0][1]}, 
var_ap2 = {var_p[1][0]}, var_bp2 = {var_p[1][1]}, 
var_an2 = {var_n[1][0]}, var_bn2 = {var_n[1][1]}, 
var_ap3 = {var_p[2][0]}, var_bp3 = {var_p[2][1]}, 
var_an3 = {var_n[2][0]}, var_bn3 = {var_n[2][1]}, 
var_ap4 = {var_p[3][0]}, var_bp4 = {var_p[3][1]}, 
var_an4 = {var_n[3][0]}, var_bn4 = {var_n[3][1]};
'''

print(equation)

with open('src/mobot_pkg/extra/equations.txt', 'w') as wf:
    wf.write(equation)


