#!/usr/bin/env python

#
# Written by Niels Joubert, Oct 28th 2014
#
# Dependencies: pymavlink (globally installed)
#

from pymavlink import DFReader
import os

def convertLog(infilename, outfilename):
    print "Converting %s to %s" % (infilename, outfilename)
    log = DFReader.DFReader_binary(infilename, 0)
    
    with open(outfilename, 'w') as f:
        while True:
            m = log.recv_msg()
            if m is None:
                break

            data = []
            for i in range(len(m.fmt.columns)):
                name = m.fmt.columns[i]
                data.append(m._d[name])

            f.write("%s, %s\n" % (m.get_type(), ', '.join([str(a) for a in data])))

def getOutFileName(infilename):
    partial_filename = os.path.splitext(infilename)[0]
    log = DFReader.DFReader_binary(infilename, 0)
    timestamp = None
    while True:
        m = log.recv_msg
        timestamp = log.timestamp
        if timestamp != None:
            break
    if timestamp != None:
        import datetime
        f = datetime.datetime.fromtimestamp(timestamp).strftime('%m-%d-%Y-%H-%M-%S')
        return partial_filename + "." + str(f) + ".log"
    return partial_filename + ".log"

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print "Usage: mavlogconvert.py <filename.bin> | <directory containing .bin>"
        sys.exit(1)
    argumentf = sys.argv[1]
    if os.path.isdir(argumentf):
        for fname in os.listdir(argumentf):
            if (os.path.splitext(fname)[1].lower() == ".bin"):
            	fullfname = os.path.join(argumentf, fname)
                convertLog(fullfname, os.path.join(argumentf, getOutFileName(fullfname)))
    else:
        convertLog(argumentf, getOutFileName(argumentf))