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
from cpp_aco import *
from cpp_mcts import *
import pandas as pd

np.random.seed(124)

def testACO(vps,startIdx,tc,alpha,beta,rho,acIter,i,save):
    print("test ACO for {} configuration {:.2f} {} {} {} {:.2f}".format(i,tc,acIter,alpha,beta,rho))
    tspACO = ACO(vps=vps, startIdx=startIdx, ants=len(vps), alpha=alpha, beta=beta, rho=rho)
    progress, bestvps = tspACO.run(acIter)
    trajectory = "acobest_"+str(i+1)
    vpGenerator.save(os.path.join(save,trajectory+".txt"), bestvps)
    dict = {'Iteration': [j for j in range(len(progress))], 'Distance': progress}
    df = pd.DataFrame(dict)
    df.to_csv(os.path.join(save,trajectory+".csv"))
    return

def compareACO(vps,save,tc):
    configs = []
    # test with different configs (alpha, beta, rho)
    configs.append((1,3,0.05)) # 1
    # use set covering problem to find a minimun set of viewpoints
    startIdx = nearestViewpoint((0,0,0),vps)
    scp = SCPSolver(vps,startIdx,coverage=tc)
    minvps = scp.computeMinimumCoveringViewpoints(iter=1000)
    for i in range(len(configs)):
        testACO(minvps,minvps.index(vps[startIdx]),tc,configs[i][0],configs[i][1],configs[i][2],2000,i,save)
    return

def testMCTS(vps,tc,nb,cn,cp,fe,dr,iter,i,save):
    print("test MCTS for {} configuration {:.2f} {} {} {} {} {:.6f} {}".format(i, tc, nb, cn, cp, fe, dr, iter))
    util = ViewPointUtil(vps=vps, actDim=nb, cn=cn)
    util.buildNeighborMap()
    startIdx = 0 #nearestViewpoint((0,0,0),vps)
    startVp = util.viewpoints[startIdx]
    initState = initialState(util, startVp)
    root = MCTSNode(util,initState,parent=None)
    mcts = MonteCarloTreeSearch(util,root,cparam=cp,decay=dr,targetCoverage=tc)
    node, progress = mcts.search(iteration=iter,fe=fe)
    bestvps, coverage = mcts.test()
    trajectory = "mctsbest_"+str(i+1)
    vpGenerator.save(os.path.join(save,trajectory+".txt"),bestvps)
    dict = {'Iteration': [j for j in range(len(progress))], 'Coverage': progress}
    df = pd.DataFrame(dict)
    df.to_csv(os.path.join(save,trajectory+".csv"))
    return

def compareMCTS(vps,save,tc):
    configs = []
    configs.append((0.5,0.0,0.1,4)) # 3
    configs.append((0.5,0.1,0.1,4)) # 3
    configs.append((0.5,0.2,0.1,4)) # 3
    configs.append((0.5,0.3,0.1,4)) # 3
    configs.append((0.5,0.4,0.1,4)) # 3
    configs.append((0.5,0.5,0.1,4)) # 3
    configs.append((0.5,0.6,0.1,4)) # 3
    configs.append((0.5,0.7,0.1,4)) # 3
    configs.append((0.5,0.8,0.1,4)) # 3
    configs.append((0.5,0.9,0.1,4)) # 3
    configs.append((0.5,1.0,0.1,4)) # 3
    for i in range(len(configs)):
        testMCTS(vps,tc,configs[i][3],configs[i][0],configs[i][1],configs[i][2],0.999,100000,i,save)
    return


############################################################
def fov(s):
    try:
        h,v = map(float, s.split(','))
        return h, v
    except:
        raise argparse.ArgumentTypeError("FOV must be h,v")

def getParameters():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mapwidth', type=int, default=100, help="map width")
    parser.add_argument('--mapheight', type=int, default=100, help="map height")
    parser.add_argument('--mapres', type=float, default=1.0, help="map grid resolution")
    parser.add_argument('--mapseeds', type=int, default=1000, help="number of seeds for map")

    parser.add_argument('--vpdis',type=float,default=5.0, help="working distance of viewpoints")
    parser.add_argument('--vpres', type=float ,default=2.0, help="resolution of viewpoints")
    parser.add_argument('--distribution', type=str, default="uniform", help="distribution type of viewpoints")
    parser.add_argument('--fov', type=fov, default = None, help="field of view of sensor")
    parser.add_argument('--nb', type=int, default=8, help="number of neighbor viewpoints")
    parser.add_argument('--cn', type=float, default=0.3, help="control parameter of choosing neighbor viewpoints")

    parser.add_argument('--savemap', type=str, default=None, help="path to save map")
    parser.add_argument('--loadmap', type=str, default=None, help="path to load map")
    parser.add_argument('--mapfile', type=str, default=None, help="filename of map")

    parser.add_argument('--savevps', type=str, default=None, help="path to save viewpoints")
    parser.add_argument('--loadvps', type=str, default=None, help="path to load viewpoints")
    parser.add_argument('--vpsfile', type=str, default=None, help="filename of viewpoints")

    # 0 no draw, 1, draw map, 2, draw map and ViewPoints, 3, draw map and trajectory
    parser.add_argument('--draw', type=int, default=0, help="draw the map and viewpoints")

    parser.add_argument('--compare', type=str, default=None, help="compare type of test, ACO or MCTS")
    parser.add_argument('--coverage', type=float, default=0.92, help="target coverage of test")
    parser.add_argument('--savebest', type=str ,default=None, help="path to save best trajectory")

    parser.add_argument('--alter',type=bool, default=False)
    parser.add_argument('--mirror', type=bool, default=False)
    return parser.parse_args()

############################################################
if __name__ == "__main__":
    args = getParameters()

    # create map
    map = GridMap()
    if args.savemap:
        map.makeMap(width=args.mapwidth,height=args.mapheight,res=args.mapres,sn=args.mapseeds)
        map.saveMap(os.path.join(args.savemap, args.mapfile))
    if args.loadmap:
        map.loadMap(os.path.join(args.loadmap, args.mapfile))

    # generate candidate viewpoints
    vps = []
    if args.savevps:
        vpGenerator = ViewPointGenerator(map=map, fov=(args.fov), res=args.vpres, dist=args.vpdis, type=args.distribution)
        vps = vpGenerator.generateViewPoints()
        vpGenerator.save(os.path.join(args.savevps, args.vpsfile),vps)
    if args.loadvps:
        vpGenerator = ViewPointGenerator(map=map, fov=(args.fov), res=args.vpres, dist=args.vpdis, type=args.distribution)
        vps = vpGenerator.load(os.path.join(args.loadvps, args.vpsfile))

    # compare test
    if args.compare == "ACO":
        compareACO(vps, args.savebest, args.coverage)
    elif args.compare == "MCTS":
        compareMCTS(vps, args.savebest, args.coverage)

    if args.alter:
        alteredVps = alterTour(vps)
        vpGenerator.save(os.path.join(args.savebest,"altered.txt"), alteredVps)

    if args.mirror:
        mirrorVps = mirrorTour(vps)
        vpGenerator.save(os.path.join(args.savebest,"mirrored.txt"), mirrorVps)

    # draw
    plotHelper = PlotHelper(width=12,height=12,map=map)
    if args.draw == 1: # map only
        plotHelper.plotMap()
        plotHelper.show()
    elif args.draw == 2: # map and viewpoints
        plotHelper.plotMap()
        plotHelper.plotViewPoints(vps,type='point')
        plotHelper.show()
    elif args.draw == 3: # map and trajectory
        plotHelper.plotMap()
        plotHelper.drawTrajectory(vps,speed=20,drawline=True,markEnd=True,insertText=True)
        plotHelper.show()
    elif args.draw == 4: # draw neighbor
        startVp = vps[int(len(vps)/2)+10]
        util = ViewPointUtil(vps=vps, actDim=args.nb, cn=args.cn)
        util.buildNeighborMap()
        nbvps = [vps[i] for i in util.neighbors(startVp.id)[0:args.nb]]
        nbvps.insert(0,startVp)
        plotHelper.drawTrajectory(nbvps,speed=10,drawline=False,markEnd=False)
        plotHelper.show()
    elif args.draw == 5: # draw minimum covering set of viewpoints
        startIdx = nearestViewpoint((0,0,0),vps)
        scp = SCPSolver(vps,startIdx,coverage=args.coverage)
        minvps = scp.computeMinimumCoveringViewpoints(iter=1000)
        plotHelper.drawTrajectory(minvps,speed=10,drawline=False,markEnd=False,insertText=False)
        plotHelper.show()
