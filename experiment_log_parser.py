import os

logs_dir = "ExperimentLogs"
experiment_name = "E2E_vs_MCTS_LQR_Complex"
wins = {}
dnfs = {}
lap_times = {}
with open(os.path.join(logs_dir, experiment_name + ".txt")) as exp_log:
    current_dnfs = set()
    current_laps = {}
    for line in exp_log:
        if line.startswith("Experiment"):
            if len(list(filter(lambda t: t not in current_dnfs, current_laps.keys()))):
                current_winner = min(filter(lambda t: t not in current_dnfs, current_laps.keys()), key=lambda t: current_laps[t])
            else:
                current_winner = ""
            if len(current_winner) and current_winner not in current_dnfs:
                wins[current_winner] = wins.get(current_winner, 0) + 1
            for d in current_dnfs:
                dnfs[d] = dnfs.get(d, 0) + 1
            for l in current_laps:
                if l not in current_dnfs:
                    if l not in lap_times: lap_times[l] = []
                    lap_times.get(l).append(current_laps[l])
            current_dnfs = set()
            current_laps = {}
        else:
            agent_type = line.split(" ")[0]
            if "Total Time" in line or "Overall Time" in line:
                current_laps[agent_type] = float(line.rsplit(" ")[3])
            if "Laps Completed" in line:
                lap_details = line.split(" ")[-1]
                lap_f, lap_t = lap_details.split("/")
                laps = int(lap_t)
                if int(lap_f)/int(lap_t) != 1:
                    current_dnfs.add(agent_type)
    if len(list(filter(lambda t: t not in current_dnfs, current_laps.keys()))):
        current_winner = min(filter(lambda t: t not in current_dnfs, current_laps.keys()),
                             key=lambda t: current_laps[t])
    else:
        current_winner = ""
    if len(current_winner) and current_winner not in current_dnfs:
        wins[current_winner] = wins.get(current_winner, 0) + 1
    for d in current_dnfs:
        dnfs[d] = dnfs.get(d, 0) + 1
    for l in current_laps:
        if l not in current_dnfs:
            if l not in lap_times: lap_times[l] = []
            lap_times.get(l).append(current_laps[l])
avg_lap_times = {l: sum(lap_times[l])/(len(lap_times[l]*laps)) for l in lap_times}

print("Wins", wins)
print("DNFs", dnfs)
print("Avg Lap Times", avg_lap_times)