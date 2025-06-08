import traci
import numpy as np

# Kết nối với SUMO
sumo_cmd = ["sumo", "-c", "../datn.sumocfg"]
traci.start(sumo_cmd)

vehicle_data = {}  # Lưu quãng đường và thời gian
speeds = []

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()
    vehicles = traci.vehicle.getIDList()

    for veh_id in vehicles:
        if veh_id not in vehicle_data:
            vehicle_data[veh_id] = {"depart_time": current_time, "distance": 0}
        vehicle_data[veh_id]["distance"] = traci.vehicle.getDistance(veh_id)

    # Kiểm tra phương tiện đã đến đích
    arrived_vehicles = traci.simulation.getArrivedIDList()
    for veh_id in arrived_vehicles:
        if veh_id in vehicle_data:
            distance = vehicle_data[veh_id]["distance"]
            travel_time = current_time - vehicle_data[veh_id]["depart_time"]
            if travel_time > 0:
                speed = (distance / 1000) / (travel_time / 3600)  # km/h
                speeds.append(speed)
            del vehicle_data[veh_id]

traci.close()

# Tính tốc độ trung bình
avg_speed = np.mean(speeds) if speeds else 0
print(f"Tốc độ trung bình: {avg_speed:.2f} km/h")