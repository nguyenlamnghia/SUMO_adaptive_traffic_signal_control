import traci
import numpy as np

# Kết nối với SUMO
sumo_cmd = ["sumo", "-c", "../datn.sumocfg"]
traci.start(sumo_cmd)

total_waiting_time = 0
vehicle_count = 0

# Chạy mô phỏng
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    # Lấy danh sách ID phương tiện hiện tại
    vehicles = traci.vehicle.getIDList()
    for veh_id in vehicles:
        waiting_time = traci.vehicle.getAccumulatedWaitingTime(veh_id)
        total_waiting_time += waiting_time
        vehicle_count += 1

traci.close()

# Tính thời gian chờ trung bình
if vehicle_count > 0:
    avg_waiting_time = total_waiting_time / vehicle_count
    print(f"Thời gian chờ trung bình: {avg_waiting_time:.2f} giây")
else:
    print("Không có phương tiện nào trong mô phỏng.")