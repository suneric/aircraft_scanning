import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np
import argparse
import os

def smoothTriangle(data, degree):
    triangle=np.concatenate((np.arange(degree + 1), np.arange(degree)[::-1])) # up then down
    smoothed=[]
    for i in range(degree, len(data) - degree * 2):
        point=data[i:i + len(triangle)] * triangle
        smoothed.append(np.sum(point)/np.sum(triangle))
    # Handle boundaries
    smoothed=[smoothed[0]]*int(degree + degree/2) + smoothed
    while len(smoothed) < len(data):
        smoothed.append(smoothed[-1])
    return smoothed

def smoothExponential(data, weight):
    last = data[0]  # First value in the plot (first timestep)
    smoothed = []
    for point in data:
        smoothed_val = last * weight + (1 - weight) * point  # Calculate smoothed value
        smoothed.append(smoothed_val)                        # Save it
        last = smoothed_val                                  # Anchor the last smoothed value
    return smoothed

def plotStat(csvFile, type):
    df = pd.read_csv(csvFile)
    fig = go.Figure()
    value = "Distance"
    title = "MAX-MIN Ant System"
    if type == "MCTS":
        value = "Coverage"
        title = "Monte Carlo Tree Search"
    fig.add_trace(go.Scatter(x = df['Iteration'], y = smoothExponential(df[value],0.99),name="Coverage Path Planning", marker=dict(color='#0075DC')))
    fig.update_layout(
        title=title,
        xaxis_title="Iteration",
        yaxis_title=value,
        font=dict(
            family="Arial",
            size=20,
            color="Black"
        ),
        plot_bgcolor="rgb(255,255,255)"
    )
    fig.show()

def plotCompareACO(files,legends,colors,maxIter=None):
    dfs = []
    for file in files:
        dfs.append(pd.read_csv(file))

    fig = go.Figure()
    title = "MAX-MIN Ant System"
    for i in range(len(dfs)):
        df = dfs[i]
        if maxIter:
            df = dfs[i][0:maxIter]
        fig.add_trace(go.Scatter(x = df['Iteration'], y = smoothExponential(df["Distance"],0.9),name=legends[i], marker=dict(color=colors[i])))

    fig.update_layout(
        title=title,
        xaxis_title="Iteration",
        yaxis_title="Distance",
        legend=dict(
            x=0.7,
            y=0.9,
            font=dict(
                family="Arial",
                size=18,
                color="Black"
            )
        ),
        font=dict(
            family="Arial",
            size=20,
            color="Black"
        ),
        plot_bgcolor="rgb(255,255,255)"
    )
    fig.show()
    return

def plotCompareMCTS(files,legends,colors,maxIter=None):
    dfs = []
    for file in files:
        dfs.append(pd.read_csv(file))

    fig = go.Figure()
    title = "Monte Carlo Tree Search"
    for i in range(len(dfs)):
        df = dfs[i]
        if maxIter:
            df = dfs[i][0:maxIter]
        fig.add_trace(go.Scatter(x = df['Iteration'], y = smoothExponential(df["Coverage"],0.999),name=legends[i], marker=dict(color=colors[i])))

    fig.update_layout(
        title=title,
        xaxis_title="Iteration",
        yaxis_title="Coverage",
        legend=dict(
            x=0.6,
            y=0.1,
            font=dict(
                family="Arial",
                size=17,
                color="Black"
            )
        ),
        font=dict(
            family="Arial",
            size=20,
            color="Black"
        ),
        plot_bgcolor="rgb(255,255,255)"
    )
    fig.show()
    return

def plotCompareMCTS_RC():
    rc = [0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0]
    coverage = [94.56,91.00,96.44,99.56,100.00,99.56,100.0,99.56,100.00,100.00,100.00]
    vpcount = [25,23,24,25,25,25,25,25,25,25,25]
    travel_dist = [168.07,154.07,161.07,168.07,168.00,168.07,168.00,168.07,168.00,168.00,168.00]

    fig = go.Figure()
    title = "Monte Carlo Tree Search Reward Control Parameter"
    fig.add_trace(go.Scatter(x = rc, y = coverage, name="coverage (%)", marker=dict(color="#ff5511")))
    fig.add_trace(go.Scatter(x = rc, y = vpcount, name="number of viewpoints", marker=dict(color="#33ee22")))
    fig.add_trace(go.Scatter(x = rc, y = travel_dist, name="travel distance (meters)", marker=dict(color="#1155ff")))

    fig.update_layout(
        title=title,
        xaxis_title="Reward Control Parameter ($\sigma$)",
        yaxis_title="",
        legend=dict(
            x=0.6,
            y=0.1,
            font=dict(
                family="Arial",
                size=17,
                color="Black"
            )
        ),
        font=dict(
            family="Arial",
            size=20,
            color="Black"
        ),
        plot_bgcolor="rgb(255,255,255)"
    )
    fig.show()
    return

def names(s):
    try:
        names = map(str, s.split(','))
        return names
    except:
        raise argparse.ArgumentTypeError("files")

# main loop
def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', type=names, default=None)
    parser.add_argument('--legend', type=names,default=None)
    parser.add_argument('--load', type=str, default=None)
    parser.add_argument('--type', type=str, default='ACO')
    parser.add_argument('--maxIter', type=int, default=None)
    return parser.parse_args()

if __name__ == "__main__":
    args = get_args()
    #plotStat(os.path.join(args.load,args.csv),type=args.type)
    colors = ['#ff5511','#33ee22','#1155ff','#552233','#ff22dd','#ddc3ee','#906c6d','#147a6f','#a2cab9','#ffaeae']
    filenames = args.csv
    legends = args.legend
    files = []
    for filename in filenames:
        files.append(os.path.join(args.load,filename+".csv"))
    print(files)

    if args.type == "ACO":
        plotCompareACO(files,legends,colors,args.maxIter)
    if args.type == "MCTS":
        plotCompareMCTS(files,legends,colors,args.maxIter)
    if args.type == "MCTS_RC":
        plotCompareMCTS_RC()
