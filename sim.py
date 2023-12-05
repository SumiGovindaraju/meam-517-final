#!/usr/bin/env python3
import traci
import time
from random import randrange
import pandas as pd


######## CONFIG ########

STEP_LENGTH = 0.1  # seconds
CYCLE_LENGTH = 60  # seconds

######## ###### ########

sumoCmd = ["sumo", "-c", "grid.sumocfg",
                   "--step-length", str(STEP_LENGTH)]
traci.start(sumoCmd)

for tlsid in traci.trafficlight.getIDList():
    traci.trafficlight.setParameter(tlsid, "cycleTime", str(CYCLE_LENGTH))

packVehicleData = []
packTLSData = []
packBigData = []

vehToLinkIDMap = {}
linkIDToInflows = {}
linkIDToOutflows = {}
linkIDToSpawns = {}
linkIDToDespawns = {}

active_vehicles = []
previous_active_vehicles = []

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()

    t = traci.simulation.getTime()  # seconds

    active_vehicles = traci.vehicle.getIDList()
    trafficlights = traci.trafficlight.getIDList()

    tmpLinkToInflow = {}
    tmpLinkToOutflow = {}
    tmpLinkToSpawns = {}
    tmpLinkToDespawns = {}

    for vehid in active_vehicles:
        link = traci.vehicle.getRoadID(vehid)

        if vehid in vehToLinkIDMap:
            oldLink = vehToLinkIDMap[vehid]

            if oldLink != link:
                tmpLinkToOutflow[oldLink] = tmpLinkToOutflow.get(oldLink, 0) + 1
                tmpLinkToInflow[link] = tmpLinkToInflow.get(link, 0) + 1
                vehToLinkIDMap[vehid] = link
        else:
            vehToLinkIDMap[vehid] = link
            tmpLinkToSpawns[link] = tmpLinkToSpawns.get(link, 0) + 1
    
    for vehid in set(previous_active_vehicles) - set(active_vehicles):
        link = vehToLinkIDMap[vehid]
        tmpLinkToDespawns[link] = tmpLinkToDespawns.get(link, 0) + 1

    for link in traci.edge.getIDList():
        if link not in linkIDToInflows:
            linkIDToInflows[link] = list()
        if link not in linkIDToOutflows:
            linkIDToOutflows[link] = list()
        if link not in linkIDToSpawns:
            linkIDToSpawns[link] = list()
        if link not in linkIDToDespawns:
            linkIDToDespawns[link] = list()

        linkIDToInflows[link].append(tmpLinkToInflow.get(link, 0))
        linkIDToOutflows[link].append(tmpLinkToOutflow.get(link, 0))
        linkIDToSpawns[link].append(tmpLinkToSpawns.get(link, 0))
        linkIDToDespawns[link].append(tmpLinkToDespawns.get(link, 0))

    previous_active_vehicles = active_vehicles

traci.close()

def avg(ls):
    return sum(ls) / len(ls)

data = []
for link in linkIDToInflows.keys():
    data.append({
        'link': link, 
        'inflow': avg(linkIDToInflows[link]) / STEP_LENGTH, 
        'outflow': avg(linkIDToOutflows[link]) / STEP_LENGTH,
        'spawns': avg(linkIDToSpawns[link]) / STEP_LENGTH,
        'despawns': avg(linkIDToDespawns[link]) / STEP_LENGTH
    })

df = pd.DataFrame(data)
df.to_csv("linkToFlows.csv", index=False)

print("***** DONE *****")