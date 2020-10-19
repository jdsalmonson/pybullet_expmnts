#import pybullet as p
import os
import struct
from collections import defaultdict
import numpy as np

class RobotLog(object):
    def __init__(self, filename, verbose = True):
        self.filename = filename
        self.verbose = verbose

        self.log = self.readLogFile()
        self.jlog = self.readJointLogFile()

    def readLogFile(self):
        f = open(self.filename, 'rb')

        if self.verbose:
            print(f'Opened: {self.filename}')

        keys = f.readline().decode('utf8').rstrip('\n').split(',')
        fmt = f.readline().decode('utf8').rstrip('\n')

        # The byte number of one record
        sz = struct.calcsize(fmt)
        # The type number of one record
        ncols = len(fmt)

        if self.verbose:
            print(f'Keys: {keys}')
            print(f'Format: {fmt}')
            print(f'Size: {sz}')
            print(f'Columns: {ncols}')

        # Read data
        wholeFile = f.read()

        # split by alignment word
        chunks = wholeFile.split(b'\xaa\xbb')
        #log = list()
        log = defaultdict(lambda: defaultdict(list))
        for chunk in chunks:
            if len(chunk) == sz:
                values = struct.unpack(fmt, chunk)
                # sort botId by objectId, which is keys[2]
                botId = values[2]
                for k, v in zip(keys, values):
                    if k is not keys[2]:
                        log[botId][k].append(v)

        return log

    def readJointLogFile(self):
        jfilename = self.filename[:-5] + ".h5"

        if not os.path.isfile(jfilename):
            print(f"HDF5 file {jfilename} not found")
            return

        from qnd.h5f import openh5

        f = openh5(jfilename, "r")

        jlog = defaultdict(list)
        for rec in f.gotoit():
            for v in f:
                jlog[v].append(f[v])

        for v in jlog:
            jlog[v] = np.asarray(jlog[v])

        return jlog


        '''
        record = list()
        for i in range(ncols):
            record.append(values[i])
            log.append(record)

            if self.verbose:
                print(f"Robot IDs: {list(set([x[2] for x in log]))}")
                return log
        '''

if __name__ == "__main__":
    #log_file_name = "/home/jay/stash/bullet3/examples/pybullet/examples/data/block_grasp_log.bin"

    #log_file_stem = "LOG00010"
    #log_file_stem = "LOG_IK_0001"
    #log_file_stem = "kuka0002"
    log_file_stem = "kuka_ik_0012"
    log_file_name = f"logs/{log_file_stem}.slog"

    bot_log = RobotLog(log_file_name)

    print(len(bot_log.log[1]['q0']))
    if hasattr(bot_log, "jlog") and bot_log.jlog is not None:
        print(bot_log.jlog['q'].shape)
