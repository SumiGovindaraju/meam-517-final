#!/usr/bin/env python3
import traci
import time
from random import randrange
import pandas as pd

sumoCmd = ["sumo", "-c", "osm.sumocfg"]
traci.start(sumoCmd)

packVehicleData = []
packTLSData = []
packBigData = []

vehToLinkIDMap = {}
linkIDToInflows = {}
linkIDToOutflows = {}

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()

    vehicles = traci.vehicle.getIDList()
    trafficlights = traci.trafficlight.getIDList()

    tmpLinkToInflow = {}
    tmpLinkToOutflow = {}

    for i in range(0, len(vehicles)):
        vehid = vehicles[i]
        link = traci.vehicle.getRoadID(vehicles[i])

        if vehid in vehToLinkIDMap:
            oldLink = vehToLinkIDMap[vehid]

            if oldLink != link:
                tmpLinkToOutflow[oldLink] = tmpLinkToOutflow.get(oldLink, 0) + 1
                tmpLinkToInflow[link] = tmpLinkToInflow.get(link, 0) + 1
                vehToLinkIDMap[vehid] = link
        else:
            vehToLinkIDMap[vehid] = link
    
    for link in traci.edge.getIDList():
        if link not in linkIDToInflows:
            linkIDToInflows[link] = list()
        if link not in linkIDToOutflows:
            linkIDToOutflows[link] = list()

        linkIDToInflows[link].append(tmpLinkToInflow.get(link, 0))
        linkIDToOutflows[link].append(tmpLinkToOutflow.get(link, 0))

traci.close()

def avg(ls):
    return sum(ls) / len(ls)

data = []
for link in linkIDToInflows.keys():
    data.append({'link': link, 'inflow': avg(linkIDToInflows[link]), 'outflow': avg(linkIDToOutflows[link])})

df = pd.DataFrame(data)
df.to_csv("linkToFlows.csv", index=False)