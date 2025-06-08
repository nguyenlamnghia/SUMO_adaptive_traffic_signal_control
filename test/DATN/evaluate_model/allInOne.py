import traci
import numpy as np
import pandas as pd

# Kết nối với SUMO
sumo_cmd = ["sumo", "-c", "../datn.sumocfg", "--tripinfo-output", "tripinfo.xml"]
traci.start(sumo_cmd)

waiting_times = []
travel_times = []
speeds = []
vehicle_data = {}

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()
    vehicles = traci.vehicle.getIDList()

    for veh_id in vehicles:
        if veh_id not in vehicle_data:
            vehicle_data[veh_id] = {"depart_time": current_time, "distance": 0}
        waiting_times.append(traci.vehicle.getAccumulatedWaitingTime(veh_id))
        vehicle_data[veh_id]["distance"] = traci.vehicle.getDistance(veh_id)

    arrived_vehicles = traci.simulation.getArrivedIDList()
    for veh_id in arrived_vehicles:
        if veh_id in vehicle_data:
            travel_time = current_time - vehicle_data[veh_id]["depart_time"]
            distance = vehicle_data[veh_id]["distance"]
            if travel_time > 0:
                travel_times.append(travel_time)
                speed = (distance / 1000) / (travel_time / 3600)  # km/h
                speeds.append(speed)
            del vehicle_data[veh_id]

traci.close()

# Tính các tiêu chí
avg_waiting_time = np.mean(waiting_times) if waiting_times else 0
avg_travel_time = np.mean(travel_times) if travel_times else 0
avg_speed = np.mean(speeds) if speeds else 0

print(f"Thời gian chờ trung bình: {avg_waiting_time:.2f} giây")
print(f"Thời gian di chuyển trung bình: {avg_travel_time:.2f} giây")
print(f"Tốc độ trung bình: {avg_speed:.2f} km/h")