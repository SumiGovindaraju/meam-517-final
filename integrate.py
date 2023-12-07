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
sumoCmd = ["sumo", "-c", "grid.sumocfg"]
traci.start(sumoCmd)

vehicleEmissions = {}
vehicleWaitingTimes = {}
maxVehicleWaitingTimes = {}
maxWaitingTimeDuringPeriod = {}

yellowDuration = 3

trafficPhases = ["GGgrrrGGgrrr", "yyyrrryyyrrr", "rrrGGgrrrGGg", "rrryyyrrryyy"]

currentPhaseIndex = 0
elapsedTime = 0  # Time elapsed in the current phase in seconds

activeVehicles = set()

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