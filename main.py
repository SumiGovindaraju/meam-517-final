from traffic import Traffic


num_cycles = 100

for i in range(num_cycles):
 
    # TODO: update linkToFlows

    # TODO: parse linkToFlows and update dynamics

    # TODO: compute MPC feedback

    # TODO: apply green time control input into SUMO
    
    # TODO: wait one cycle time and let the above control scheme play out

    # TODO: Log some sort of metric (mean queue length, vehicle speed, emissions, etc.)