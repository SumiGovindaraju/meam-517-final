import traci
import time
import pandas as pd


linkControls = {}
junctionPhases = {}
vehicleEmissions = {}
vehicleWaitingTimes = {}
maxVehicleWaitingTimes = {}
maxWaitingTimeDuringPeriod = {}

default_green_time = 42  
yellowDuration = 3
state = ' '

def generatePhaseString(phaseIndex):
    trafficPhases = ["GGgrrrGGgrrr", "yyyrrryyyrrr", "rrrGGgrrrGGg", "rrryyyrrryyy"]
    return trafficPhases[phaseIndex]

# === IMPORTING CSV ===

csv_file = "greenLinkTimes.csv"  

# The keys are traffic light IDs, and the values are the corresponding green durations
df = pd.read_csv(csv_file)
df = df.dropna(subset=['junctionId', 'junctionName', 'greenTime'])


for _, row in df.iterrows():
    link_name = row['linkName']
    junction_name = row['junctionName']
    orientation = row['orientation']
    green_time = row['greenTime']
    
    linkControls[link_name] = {'junctionName': junction_name, 'orientation': orientation, 
                               'greenTime': green_time, 'elapsedTime': 0, 'currentPhase': 'G'}

    if junction_name not in junctionPhases:
        junctionPhases[junction_name] = {'greenTimes': {}, 'currentPhaseIndex': 0, 'elapsedTime': 0}
    junctionPhases[junction_name]['greenTimes'][orientation] = green_time


# === BOOTING UP SUMO ===
# Use sumo-gui to run GUI simulation, just sumo without GUI
sumoCmd = ["sumo-gui", "-c", "grid.sumocfg"]
traci.start(sumoCmd)


currentPhaseIndex = 0
elapsedTime = 0  # Time elapsed in the current phase in seconds

activeVehicles = set()

# === SIMULATION LOOP ===
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()

    # TRAFFIC LIGHT CONTROL
    for junction, phases in junctionPhases.items():
        currentPhaseIndex = phases['currentPhaseIndex']
        phaseString = generatePhaseString(currentPhaseIndex)
        traci.trafficlight.setRedYellowGreenState(junction, phaseString)

        totalCycleTime = sum(phases['greenTimes'].values()) + yellowDuration * len(phases['greenTimes'])

        if currentPhaseIndex % 2 == 0:  # Green phase
            if phases['elapsedTime'] >= phases['greenTimes'][currentPhaseIndex // 2]:
                phases['elapsedTime'] = 0
                phases['currentPhaseIndex'] = (currentPhaseIndex + 1) % 4
        else:  # Yellow phase
            if phases['elapsedTime'] >= yellowDuration:
                phases['elapsedTime'] = 0
                phases['currentPhaseIndex'] = (currentPhaseIndex + 1) % 4

        if phases['elapsedTime'] < totalCycleTime:
            phases['elapsedTime'] += 1
        else:
            phases['elapsedTime'] = 0  # Reset cycle
        
    # DATA TRACKING
        
        #print(f"Traffic Light {tfl_id}, Current Phase Index: {control['currentPhaseIndex']}")
        currentVehicles = set(traci.vehicle.getIDList())
        # Update the set of active vehicles
        vehiclesLeft = activeVehicles - currentVehicles
        vehiclesEntered = currentVehicles - activeVehicles
        activeVehicles -= vehiclesLeft
        activeVehicles |= vehiclesEntered

        for vehID in activeVehicles:
            # Update total emissions for each vehicle
            co2Emission = traci.vehicle.getCO2Emission(vehID)
           
            if vehID not in vehicleEmissions:
                vehicleEmissions[vehID] = 0
            vehicleEmissions[vehID] += co2Emission

            # Update total and max waiting time for each vehicle
            currentWaitingTime = traci.vehicle.getWaitingTime(vehID)
            if vehID not in maxVehicleWaitingTimes:
                maxVehicleWaitingTimes[vehID] = 0
                maxWaitingTimeDuringPeriod[vehID] = 0

            if currentWaitingTime > 0:
                # Update max waiting time for the current period
                if currentWaitingTime > maxWaitingTimeDuringPeriod[vehID]:
                    maxWaitingTimeDuringPeriod[vehID] = currentWaitingTime
            else:
                # Waiting time is zero, add the max waiting time of this period to the total and reset
                maxVehicleWaitingTimes[vehID] += maxWaitingTimeDuringPeriod[vehID]
                maxWaitingTimeDuringPeriod[vehID] = 0
              

traci.close()
emissions_df = pd.DataFrame(list(vehicleEmissions.items()), columns=['VehicleID', 'TotalCO2Emissions'])
# Create a DataFrame for total max waiting times
max_waiting_times_df = pd.DataFrame(list(maxVehicleWaitingTimes.items()), columns=['VehicleID', 'TotalMaxWaitingTime'])

# Combine the DataFrames
combined_df = pd.merge(emissions_df, max_waiting_times_df, on='VehicleID')

# Save to Excel
combined_df.to_csv("emissionsData.csv", index=False)

time.sleep(5)