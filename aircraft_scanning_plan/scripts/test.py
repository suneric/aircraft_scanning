"""
discrete coverage path planning problem
with monte carlo tree search algorithm
"""
import time
import random
import numpy as np
import math
from map import *
from util import *
import sys
import os
import argparse

############################################################
# main
def getParameters():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mapwidth', type=int, default=100)
    parser.add_argument('--mapheight', type=int, default=100)
    parser.add_argument('--mapres', type=float, default=1.0) # grid resolution
    parser.add_argument('--mapseeds', type=int, default=1000) # number of seeds
    parser.add_argument('--viewdis',type=float,default=5.0) # working distance
    parser.add_argument('--vpres', type=float ,default=2.0)
    parser.add_argument('--vpmethod', type=str, default="uniform") # viewpoints generation method

    parser.add_argument('--nb', type=int, default=8)
    parser.add_argument('--cn', type=float, default=0.3)

    parser.add_argument('--save', type=str, default=None)
    parser.add_argument('--load', type=str, default=None)
    parser.add_argument('--mapfile', type=str, default=None)
    parser.add_argument('--vpsfile', type=str, default=None)
    parser.add_argument('--draw', type=int, default=0) # 0 no draw, 1, draw map, 2, draw map and ViewPoints, 3, draw map and trajectory
    return parser.parse_args()

if __name__ == "__main__":
    args = getParameters()
    # create map, viewpoints and save
    vps = []
    map = GridMap()
    if args.save:
        map.makeMap(width=args.mapwidth,height=args.mapheight,res=args.mapres,sn=args.mapseeds)
        map.saveMap(os.path.join(args.save, args.mapfile))
        vpGenerator = ViewPointGenerator(map=map, fov=(80.0,80.0), res=args.vpres, dist=args.viewdis, type=args.vpmethod)
        vps = vpGenerator.generateViewPoints()
        vpGenerator.save(os.path.join(args.save, args.vpsfile),vps)

    if args.load:
        map.loadMap(os.path.join(args.load, args.mapfile))
        vpGenerator = ViewPointGenerator(map=map, fov=(80.0,80.0), res=args.vpres, dist=args.viewdis, type=args.vpmethod)
        vps = vpGenerator.load(os.path.join(args.load, args.vpsfile))

    plotHelper = PlotHelper(width=16,height=12,map=map)
    if args.draw == 1: # map only
        plotHelper.plotMap()
    elif args.draw == 2: # map and viewpoints
        plotHelper.plotMap()
        plotHelper.plotViewPoints(vps)
    elif args.draw == 3: # map and trajectory
        plotHelper.plotMap()
        plotHelper.drawTrajectory(vps,speed=20)
    elif args.draw == 4: # draw neighbor
        startIdx = np.random.randint(len(vps))
        startVp = vps[startIdx]
        util = ViewPointUtil(vps=vps, actDim=args.nb, cn=args.cn)
        util.buildNeighborMap()
        nbvps = [vps[i] for i in util.neighbors(startVp.id)[0:args.nb]]
        nbvps.insert(0,startVp)
        plotHelper.drawTrajectory(nbvps,speed=10,drawline=False)
    plotHelper.show()
