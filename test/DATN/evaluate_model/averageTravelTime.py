import traci
import numpy as np

# Kết nối với SUMO
sumo_cmd = ["sumo", "-c", "../datn.sumocfg"]
traci.start(sumo_cmd)

vehicle_times = {}  # Lưu thời gian xuất phát và đến
travel_times = []

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    # Lấy danh sách phương tiện
    vehicles = traci.vehicle.getIDList()
    current_time = traci.simulation.getTime()

    for veh_id in vehicles:
        if veh_id not in vehicle_times:
            vehicle_times[veh_id] = {"depart": current_time}

    # Kiểm tra phương tiện đã đến đích
    arrived_vehicles = traci.simulation.getArrivedIDList()
    for veh_id in arrived_vehicles:
        if veh_id in vehicle_times:
            travel_time = current_time - vehicle_times[veh_id]["depart"]
            travel_times.append(travel_time)
            del vehicle_times[veh_id]

traci.close()

# Tính thời gian di chuyển trung bình
avg_travel_time = np.mean(travel_times) if travel_times else 0
print(f"Thời gian di chuyển trung bình: {avg_travel_time:.2f} giây")