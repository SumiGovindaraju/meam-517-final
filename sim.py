#!/usr/bin/env python3
import traci
import time
from random import randrange
import pandas as pd


######## CONFIG ########

STEP_LENGTH = 1  # seconds
CYCLE_LENGTH = 60  # seconds

VEHICLE_LENGTH = 5 # meters
VEHICLE_MIN_GAP = 2.5 # meters

######## ###### ########

# Edge Indexing
edgeToLinkIdx = {}
tflToJunctionIdx = {}

def getLinkIdx(edge):
    if ":J" in edge:
        return -1

    if edge not in edgeToLinkIdx:
        edgeToLinkIdx[edge] = int(len(edgeToLinkIdx))

    return edgeToLinkIdx[edge]

def getJunctionIdx(tflid):
    if tflid not in tflToJunctionIdx:
        tflToJunctionIdx[tflid] = int(len(tflToJunctionIdx))

    return tflToJunctionIdx[tflid]

# Start sumo cmd
sumoCmd = ["sumo", "-c", "grid.sumocfg",
                   "--step-length", str(STEP_LENGTH)]
traci.start(sumoCmd)

# Configure cycle time
for tlsid in traci.trafficlight.getIDList():
    traci.trafficlight.setParameter(tlsid, "cycleTime", str(CYCLE_LENGTH))

# Estimate max queue length
linkToMaxQueueLength = {}
for laneid in traci.lane.getIDList():
    link = traci.lane.getEdgeID(laneid)
    length = traci.lane.getLength(laneid)

    if length > VEHICLE_LENGTH:
        linkToMaxQueueLength[link] = 1 + (length - VEHICLE_LENGTH) / (VEHICLE_LENGTH + VEHICLE_MIN_GAP)
    else:
        linkToMaxQueueLength[link] = 0

# Get junctions
linkToJunctionInfo = {}
for tlsid in traci.trafficlight.getIDList():
    junctionIdx = getJunctionIdx(tlsid)

    for ls in traci.trafficlight.getControlledLinks(tlsid):
        for incoming_link, _, _ in ls:     
            incoming_link = incoming_link.rsplit('_', 1)[0]
            orientation = int(0 if traci.edge.getAngle(incoming_link) % 180 == 0 else 1)
            linkToJunctionInfo[incoming_link] = (junctionIdx, tlsid, orientation)

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
            # Config vehicle
            traci.vehicle.setLength(vehid, VEHICLE_LENGTH)
            traci.vehicle.setMinGap(vehid, VEHICLE_MIN_GAP)

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
    idx = getLinkIdx(link)
    if idx == -1:
        continue

    junctionIdx, junctionName, orientation = None, None, None
    if link in linkToJunctionInfo:
        junctionIdx, junctionName, orientation = linkToJunctionInfo[link]

    data.append({
        'id': int(idx),
        'linkName': link, 
        'inflow': avg(linkIDToInflows[link]) / STEP_LENGTH, 
        'outflow': avg(linkIDToOutflows[link]) / STEP_LENGTH,
        'spawns': avg(linkIDToSpawns[link]) / STEP_LENGTH,
        'despawns': avg(linkIDToDespawns[link]) / STEP_LENGTH,
        'maxQueueLength': linkToMaxQueueLength.get(link, 0),
        'junctionId': int(junctionIdx) if junctionIdx is not None else None,
        'junctionName': junctionName,
        'orientation': int(orientation) if orientation is not None else None,
    })

df = pd.DataFrame(data)
df.to_csv("linkToFlows.csv", index=False)

print("***** DONE *****")