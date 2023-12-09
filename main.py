#!/usr/bin/env python3

######## CONFIG ########
USE_CONTROLLER = True

CONFIG_FILE = "linkToFlows.csv"
DATA_FILE = "sim_data.csv" if not USE_CONTROLLER else "controller_data.csv"

MAX_CYCLES = 100  # number of cycles to run the simulation
TIME_HORIZON = 5  # number of cycles between recalibrating dynamics

STEP_LENGTH = 1  # seconds
CYCLE_LENGTH = 60  # seconds

VEHICLE_LENGTH = 5  # meters
VEHICLE_MIN_GAP = 2.5  # meters

SHOW_GUI = False  # set to true if you want to show the GUI
######## ###### ########

import csv
import numpy as np
import pandas as pd
import traci

from traffic import Traffic

######## UTIL ########
def avg(ls):
    return sum(ls) / len(ls) if len(ls) > 0 else 0

def trim_link_name(link):
    return link.rsplit('_', 1)[0]
######## #### ########

######## EDGE/JUNCTION INDEXING ########
edgeToLinkIdx = {}
tflToJunctionIdx = {}

def getLinkIdx(edge):
    if ":J" in edge:
        return -1

    if edge not in edgeToLinkIdx:
        edgeToLinkIdx[edge] = int(len(edgeToLinkIdx))
        print(edge)

    return edgeToLinkIdx[edge]

def getJunctionIdx(tflid):
    if tflid not in tflToJunctionIdx:
        tflToJunctionIdx[tflid] = int(len(tflToJunctionIdx))
        print(tflid)

    return tflToJunctionIdx[tflid]

def init_indices_from_file(file_name):
    with open(file_name, mode='r') as file:
        csvFile = csv.DictReader(file)

        for line in csvFile:
            edgeToLinkIdx[line['linkName']] = int(float(line['id']))

            if len(line['junctionId']) != 0:
                tflToJunctionIdx[line['junctionName']] = int(float(line['junctionId']))

init_indices_from_file(CONFIG_FILE)

######## ######################## ########

######## START SUMO AND CONFIG ########
sumoCmd = ["sumo-gui" if SHOW_GUI else "sumo",
           "-c", "grid.sumocfg",
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
            incoming_link = trim_link_name(incoming_link)
            orientation = int(0 if traci.edge.getAngle(incoming_link) % 180 == 0 else 1)
            linkToJunctionInfo[incoming_link] = (junctionIdx, tlsid, orientation)
######## ##################### ########

# Initialize dynamics data structures
vehToLinkIDMap = {}
linkIDToInflows = {}
linkIDToOutflows = {}
linkIDToSpawns = {}
linkIDToDespawns = {}

active_vehicles = []
previous_active_vehicles = []

def reset_dynamics_data_structs():
    vehToLinkIDMap.clear()
    linkIDToInflows.clear()
    linkIDToOutflows.clear()
    linkIDToSpawns.clear()
    linkIDToDespawns.clear()
    active_vehicles.clear()
    previous_active_vehicles.clear()

# Default linkToFlows
traffic = Traffic(sumo_file=CONFIG_FILE, cycle_time=CYCLE_LENGTH)

logged_data = []

for i in range(MAX_CYCLES):
    # update linkToFlows and update dynamics (if necessary)
    if i > 0 and i % TIME_HORIZON == 0:
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

        reset_dynamics_data_structs()
        linkToFlows = pd.DataFrame(data)
        traffic = Traffic(sumo_df=linkToFlows, cycle_time=CYCLE_LENGTH)
    
    # compute state and MPC feedback
    x = np.zeros((traffic.N, 1))
    for vehid in active_vehicles:
        link = traci.vehicle.getRoadID(vehid)
        x[getLinkIdx(link)] += 1

    u, _ = traffic.compute_MPC_feedback(x)
    print(u)

    junctionToActuation = {}  # junction -> (switch time, first orientation)
    for tlsid in traci.trafficlight.getIDList():
        for ls in traci.trafficlight.getControlledLinks(tlsid):
            for incoming_link, _, _ in ls:
                incoming_link = trim_link_name(incoming_link)
                _, junction, orientation = linkToJunctionInfo[incoming_link]

                if junction not in junctionToActuation:
                    green_time = u[getLinkIdx(incoming_link)]
                    if orientation == 1:
                        green_time = CYCLE_LENGTH - green_time

                    junctionToActuation[junction] = (green_time, 0)

    # iterate through 1 cycle and apply green time control input into SUMO, while collecting data
    for j in range(int(CYCLE_LENGTH / STEP_LENGTH)):
        traci.simulationStep()
        
        t = traci.simulation.getTime()
        
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
        
        # apply controller
        for tlsid in traci.trafficlight.getIDList():
            for tl_link_idx, ls in enumerate(traci.trafficlight.getControlledLinks(tlsid)):
                for incoming_link, _, _ in ls:
                    incoming_link = trim_link_name(incoming_link)

                    switchTime, firstOrientation = junctionToActuation[junction]
                    orientation = int(0 if traci.edge.getAngle(incoming_link) % 180 == 0 else 1)

                    green = (t < switchTime and orientation == firstOrientation) or (t >= switchTime and orientation != firstOrientation)

                    if USE_CONTROLLER:
                        traci.trafficlight.setLinkState(tlsid, tl_link_idx, 'G' if green else 'R')

        # TODO: Log some sort of metrics (mean queue length, vehicle speed, emissions, etc.)

    # end early if no more cars
    if traci.simulation.getMinExpectedNumber() == 0:
        break

traci.close()

pd.DataFrame(logged_data).to_csv(DATA_FILE, index=False)
print("**** DONE! ****")