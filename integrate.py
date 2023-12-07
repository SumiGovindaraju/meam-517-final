import traci
import time
import pandas as pd


csv_file = "greenTimeInputs.csv"  

# The keys are traffic light IDs, and the values are the corresponding green durations
df = pd.read_csv(csv_file)

# Each traffic light will have its own phase index, elapsed time, and green duration
# Read CSV and create a dictionary with additional control keys
tflControls = {tfl_id: {'greenTime1': row['greenTime1'], 'greenTime2': row['greenTime2'], 
                        'currentPhaseIndex': 0, 'elapsedTime': 0}
               for tfl_id, row in df.set_index('tfl_id').iterrows()}


# Use sumo-gui to run GUI simulation, just sumo without GUI
sumoCmd = ["sumo-gui", "-c", "grid.sumocfg"]
traci.start(sumoCmd)

vehicleEmissions = {}
vehicleWaitingTimes = {}

yellowDuration = 3

trafficPhases = ["GGgrrrGGgrrr", "yyyrrryyyrrr", "rrrGGgrrrGGg", "rrryyyrrryyy"]

currentPhaseIndex = 0
elapsedTime = 0  # Time elapsed in the current phase in seconds

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()

    for tfl_id, control in tflControls.items():
        # Determine the duration for the current phase
        if control['currentPhaseIndex'] in [0, 2]:  # Green phases
            phaseDuration = control['greenTime1'] if control['currentPhaseIndex'] == 0 else control['greenTime2']
        else:  # Yellow phases
            phaseDuration = yellowDuration

        # Check if it's time to change the phase
        if control['elapsedTime'] >= phaseDuration:
            newPhaseIndex = (control['currentPhaseIndex'] + 1) % len(trafficPhases)
            traci.trafficlight.setRedYellowGreenState(tfl_id, trafficPhases[newPhaseIndex])
            control['currentPhaseIndex'] = newPhaseIndex
            control['elapsedTime'] = 0
        else:
            control['elapsedTime'] += 1

        #print(f"Traffic Light {tfl_id}, Current Phase Index: {control['currentPhaseIndex']}")

        vehicles=traci.vehicle.getIDList()

        for vehID in vehicles:
            # Update total emissions for each vehicle
            co2Emission = traci.vehicle.getCO2Emission(vehID)
            if vehID not in vehicleEmissions:
                vehicleEmissions[vehID] = 0
            vehicleEmissions[vehID] += co2Emission

            # Update total waiting time for each vehicle
            waitingTime = traci.vehicle.getWaitingTime(vehID)
            if vehID not in vehicleWaitingTimes:
                vehicleWaitingTimes[vehID] = 0
            vehicleWaitingTimes[vehID] += waitingTime
              

traci.close()

emissions_df = pd.DataFrame(list(vehicleEmissions.items()), columns=['VehicleID', 'TotalCO2Emissions'])
waiting_times_df = pd.DataFrame(list(vehicleWaitingTimes.items()), columns=['VehicleID', 'TotalWaitingTime'])

combined_df = pd.merge(emissions_df, waiting_times_df, on='VehicleID')

# Save to Excel
combined_df.to_csv("emissionsData.csv", index=False)

time.sleep(5)