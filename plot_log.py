import os
import sys
import numpy as np
import pylab as plt

from robotlog import RobotLog

log_file_name = "LOG_IK_0001.txt"
bot_log = RobotLog(log_file_name)

botNum = 1
log = bot_log.log[botNum]

qNum = list(set(log['qNum']))[0]

numplots = qNum
cols = 3

rows, remainder = divmod(numplots, cols)
if remainder > 0:
    rows += 1

stp = np.array(log['stepCount'])
stpz = 0.5 * (stp[:-1] + stp[1:])
tmstmp = np.array(log['timeStamp'])
tmstmpz = 0.5 * (tmstmp[:-1] + tmstmp[1:])

fig, axs = plt.subplots(rows, cols, figsize = (15, 8), sharex = True)
fig.suptitle("Position and Velocity of joints as a function of time stamp")
for iplt in range(numplots):

    rw, cl = divmod(iplt, cols)

    qn = np.array(log['q'+str(iplt)])
    un = np.array(log['u'+str(iplt)])
    dq = np.diff(qn) / np.diff(tmstmp)  # same as un
    acc = np.diff(un) / np.diff(tmstmp)
    acc = np.clip(acc, -200, 200)
    stpz = 0.5 * (stp[:-1] + stp[1:])
    axs[rw, cl].plot(tmstmp, log['q'+str(iplt)], label = "q")
    axs[rw, cl].plot(tmstmp, log['u'+str(iplt)], label = "u")
    axs[rw, cl].plot(tmstmpz, dq, label = "dq")
    axs[rw, cl].plot(tmstmpz, acc / 10.,'--', label = "a/10")
    axs[rw, cl].set_title(f"Joint {iplt}")

plt.legend()
#plt.tight_layout()
plt.show()
