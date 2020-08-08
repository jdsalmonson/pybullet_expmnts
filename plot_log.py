import os
import sys
import numpy as np
import pylab as plt

from robotlog import RobotLog

log_file_name = "LOG00010.txt" #"LOG_IK_0001.txt"
botNum = 1
'''
log_file_name = "LOG_IK_PANDA_0001.txt"
botNum = 7
'''

bot_log = RobotLog(log_file_name)

log = bot_log.log[botNum]

qNum = list(set(log['qNum']))[0]

numplots = qNum
cols = 3

rows, remainder = divmod(numplots, cols)
if remainder > 0:
    rows += 1

def ave(ary):
    return 0.5 * (ary[:-1] + ary[1:])

def smooth(x, window_len=11):
    """based on: https://scipy-cookbook.readthedocs.io/items/SignalSmooth.html"""
    if window_len<3:
        return x
    s=np.r_[x[window_len-1:0:-1],x,x[-2:-window_len-1:-1]]
    w=np.hanning(window_len)
    y=np.convolve(w/w.sum(),s,mode='valid')
    return y[int(np.floor(window_len/2-1)):-int(np.ceil(window_len/2))]

stp = np.array(log['stepCount'])
stpz = ave(stp)
tmstmp = np.array(log['timeStamp'])
tmstmpz = ave(tmstmp)

fig, axs = plt.subplots(rows, cols, figsize = (15, 8), sharex = True)
fig.suptitle("Position and Velocity of joints as a function of time stamp")
for iplt in range(numplots):

    rw, cl = divmod(iplt, cols)

    qn = np.array(log['q'+str(iplt)])
    un = np.array(log['u'+str(iplt)])
    dq = np.diff(qn) / np.diff(tmstmp)  # same as un

    # smooth velocity:
    un = smooth(un, window_len=16)

    acc = np.diff(un) / np.diff(tmstmp)

    acc = np.clip(acc, -200, 200)
    #stpz = 0.5 * (stp[:-1] + stp[1:])
    pla = []
    pla.extend( axs[rw, cl].plot(tmstmp, log['q'+str(iplt)], label = "q") )
    pla.extend( axs[rw, cl].plot(tmstmp, log['u'+str(iplt)], label = "u") )
    #pla.extend( axs[rw, cl].plot(tmstmp, un, label = "u smooth") )
    #pla.extend( axs[rw, cl].plot(tmstmpz, dq, label = "dq") )
    pla.extend( axs[rw, cl].plot(tmstmpz, acc / 10.,'--', label = "a/10") )
    torque_key = 't'+str(iplt)
    if torque_key in log.keys():
        #axs[rw, cl].plot(tmstmp, smooth(np.array(log[torque_key]), window_len=2)/10, label = "torque/10")
        #pla.extend( axs[rw, cl].plot(tmstmp, smooth(np.array(log[torque_key]), window_len=11), label = "smooth torque") )
        pla.extend( axs[rw, cl].plot(tmstmp, np.array(log[torque_key]), label = "torque") )

    # assemble list of plots & legends from first plot,
    # otherwise sometimes matplotlib doesn't get the plot legends
    if iplt == 0:
        pl_leg = []
        for pl in pla:
            pl_leg.append([pl, pl.get_label()])

    axs[rw, cl].set_title(f"Joint {iplt}")

plt.legend(*zip(*pl_leg))
#plt.tight_layout()
plt.show()
