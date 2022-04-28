import os

logs_dir = "ExperimentLogs"
points_per_position = [10, 7.5, 6, 4]
def summarize_experiment(experiment_name):
    print(experiment_name)
    wins = {}
    dnfs = {}
    lap_times = {}
    win_margins = {}
    collisions = {}
    illegal_changes = {}
    lane_differences = {}
    vel_differences = {}
    points = {}
    with open(os.path.join(logs_dir, experiment_name + ".txt")) as exp_log:
        current_dnfs = set()
        current_laps = {}
        for line in exp_log:
            if line.startswith("Experiment"):
                finishers = list(filter(lambda t: t not in current_dnfs, current_laps.keys()))
                if len(finishers):
                    current_winner = min(finishers, key=lambda t: current_laps[t])
                    ordering = list(sorted(finishers, key=lambda t: current_laps[t]))
                else:
                    current_winner = ""
                    ordering = []
                if len(current_winner) and current_winner not in current_dnfs:
                    l = current_winner.split("(")[0]
                    wins[l] = wins.get(l, 0) + 1
                for d in current_dnfs:
                    l = d.split("(")[0]
                    dnfs[l] = dnfs.get(l, 0) + 1
                if len(finishers) > 1 and current_winner != "":
                    l = current_winner.split("(")[0]
                    if l not in win_margins:
                        win_margins[l] = []
                    win_margins[l].append(current_laps[max(finishers, key=lambda t: current_laps[t])] - current_laps[min(finishers, key=lambda t: current_laps[t])])
                for a in current_laps:
                    agent_type = a.split("(")[0]
                    l = agent_type
                    if l not in points: points[l] = []
                    if l not in collisions: collisions[l] = []
                    if l not in illegal_changes: illegal_changes[l] = []
                    if l not in current_dnfs:
                        if l not in lap_times: lap_times[l] = []
                        if l not in lane_differences: lane_differences[l] = []
                        if l not in vel_differences: vel_differences[l] = []
                        lap_times.get(l).append(current_laps[a])
                        lane_differences.get(l).append(current_tld[a])
                        vel_differences.get(l).append(current_tvd[a])
                    collisions.get(l).append(current_collisions[a])
                    illegal_changes.get(l).append(current_lcs[a])
                    points[l].append(0)
                    for idx, finisher in enumerate(ordering):
                        if finisher.startswith(l):
                            points[l][-1] += points_per_position[idx]
                current_dnfs = set()
                current_collisions = {}
                current_lcs = {}
                current_tld = {}
                current_tvd = {}
                current_laps = {}
            else:
                agent_name = line.split(" ")[0]
                # agent_type = line.split(" ")[0][:-3]
                if "Total Time" in line or "Overall Time" in line:
                    current_laps[agent_name] = float(line.rsplit(" ")[-1])
                if "Collisions" in line:
                    current_collisions[agent_name] = float(line.rsplit(" ")[-1])
                if "Illegal Lane Changes" in line:
                    current_lcs[agent_name] = float(line.rsplit(" ")[-1])
                if "Avg Target Lane Difference" in line:
                    current_tld[agent_name] = float(line.rsplit(" ")[-1])
                if "Avg Target Vel Difference" in line:
                    current_tvd[agent_name] = float(line.rsplit(" ")[-1])
                if "Laps Completed" in line:
                    lap_details = line.split(" ")[-1]
                    lap_f, lap_t = lap_details.split("/")
                    laps = int(lap_t)
                    if int(lap_f)/int(lap_t) != 1:
                        current_dnfs.add(agent_name)
                if "Collisions" in line:
                    lap_details = line.split(" ")[-1]
        
        finishers = list(filter(lambda t: t not in current_dnfs, current_laps.keys()))
        if len(finishers):
            current_winner = min(finishers, key=lambda t: current_laps[t])
            ordering = list(sorted(finishers, key=lambda t: current_laps[t]))
        else:
            current_winner = ""
            ordering = []
        if len(current_winner) and current_winner not in current_dnfs:
            l = current_winner.split("(")[0]
            wins[l] = wins.get(l, 0) + 1
        for d in current_dnfs:
            l = d.split("(")[0]
            dnfs[l] = dnfs.get(l, 0) + 1
        if len(finishers) > 1 and current_winner != "":
            l = current_winner.split("(")[0]
            if l not in win_margins:
                win_margins[l] = []
            win_margins[l].append(current_laps[max(finishers, key=lambda t: current_laps[t])] - current_laps[min(finishers, key=lambda t: current_laps[t])])
        for a in current_laps:
            agent_type = a.split("(")[0]
            l = agent_type
            if l not in points: points[l] = []
            if l not in collisions: collisions[l] = []
            if l not in illegal_changes: illegal_changes[l] = []
            if l not in current_dnfs:
                if l not in lap_times: lap_times[l] = []
                if l not in lane_differences: lane_differences[l] = []
                if l not in vel_differences: vel_differences[l] = []
                lap_times.get(l).append(current_laps[a])
                lane_differences.get(l).append(current_tld[a])
                vel_differences.get(l).append(current_tvd[a])
            collisions.get(l).append(current_collisions[a])
            illegal_changes.get(l).append(current_lcs[a])
            points[l].append(0)
            for idx, finisher in enumerate(ordering):
                if finisher.startswith(l):
                    points[l][-1] += points_per_position[idx]
    avg_lap_times = {l: sum(lap_times[l])/(len(lap_times[l]*laps)) for l in lap_times}
    avg_win_margins = {l: sum(win_margins[l])/(len(win_margins[l])) for l in win_margins}
    avg_colls = {l: sum(collisions[l])/(len(collisions[l])) for l in collisions}
    avg_lcs = {l: sum(illegal_changes[l])/(len(illegal_changes[l])) for l in illegal_changes}
    avg_tld = {l: sum(lane_differences[l])/(len(lane_differences[l])) for l in lane_differences}
    avg_tvd = {l: sum(vel_differences[l])/(len(vel_differences[l])) for l in vel_differences}
    avg_points = {l : sum(points[l])/len(points[l]) for l in points}
    avg_safety_score = {l: (sum(collisions[l]+illegal_changes[l]))/len(collisions[l]) for l in collisions}
    stddv_safety_score = {l: (sum([((illegal_changes[l][i]+collisions[l][i])-avg_safety_score[l])**2 for i in range(len(collisions[l]))])/len(collisions[l]))**0.5 for l in collisions}

    print("Wins", wins)
    print("DNFs", dnfs)
    # print("Avg Lap Times", avg_lap_times)
    # print ("Avg Win Margins", avg_win_margins)
    print ("Avg Collisions", avg_colls)
    # print ("Avg Illegal Lane Changes", avg_lcs)
    # print ("Avg Target Lane Distance", avg_tld)
    # print("Avg Target Vel Diff", avg_tvd)
    print("Avg Points Per Race", avg_points)
    print("Avg Safety Score", avg_safety_score)
    # print("Std Dev safety Score", stddv_safety_score)
    print()

def summarize_multiple_experiments(experiments_list):
    wins = {}
    dnfs = {}
    lap_times = {}
    win_margins = {}
    collisions = {}
    illegal_changes = {}
    lane_differences = {}
    vel_differences = {}
    points = {}
    for experiment_name in experiments_list:
        with open(os.path.join(logs_dir, experiment_name + ".txt")) as exp_log:
            current_dnfs = set()
            current_laps = {}
            for line in exp_log:
                if line.startswith("Experiment"):
                    finishers = list(filter(lambda t: t not in current_dnfs, current_laps.keys()))
                    if len(finishers):
                        current_winner = min(finishers, key=lambda t: current_laps[t])
                        ordering = list(sorted(finishers, key=lambda t: current_laps[t]))
                    else:
                        current_winner = ""
                        ordering = []
                    if len(current_winner) and current_winner not in current_dnfs:
                        l = current_winner.split("(")[0]
                        wins[l] = wins.get(l, 0) + 1
                    for d in current_dnfs:
                        l = d.split("(")[0]
                        dnfs[l] = dnfs.get(l, 0) + 1
                    if len(finishers) > 1 and current_winner != "":
                        l = current_winner.split("(")[0]
                        if l not in win_margins:
                            win_margins[l] = []
                        win_margins[l].append(current_laps[max(finishers, key=lambda t: current_laps[t])] - current_laps[min(finishers, key=lambda t: current_laps[t])])
                    for a in current_laps:
                        agent_type = a.split("(")[0]
                        l = agent_type
                        if l not in points: points[l] = []
                        if l not in collisions: collisions[l] = []
                        if l not in illegal_changes: illegal_changes[l] = []
                        if l not in current_dnfs:
                            if l not in lap_times: lap_times[l] = []
                            if l not in lane_differences: lane_differences[l] = []
                            if l not in vel_differences: vel_differences[l] = []
                            lap_times.get(l).append(current_laps[a])
                            lane_differences.get(l).append(current_tld[a])
                            vel_differences.get(l).append(current_tvd[a])
                        collisions.get(l).append(current_collisions[a])
                        illegal_changes.get(l).append(current_lcs[a])
                        points[l].append(0)
                        for idx, finisher in enumerate(ordering):
                            if finisher.startswith(l):
                                points[l][-1] += points_per_position[idx]
                    current_dnfs = set()
                    current_collisions = {}
                    current_lcs = {}
                    current_tld = {}
                    current_tvd = {}
                    current_laps = {}
                else:
                    agent_name = line.split(" ")[0]
                    # agent_type = line.split(" ")[0][:-3]
                    if "Total Time" in line or "Overall Time" in line:
                        current_laps[agent_name] = float(line.rsplit(" ")[-1])
                    if "Collisions" in line:
                        current_collisions[agent_name] = float(line.rsplit(" ")[-1])
                    if "Illegal Lane Changes" in line:
                        current_lcs[agent_name] = float(line.rsplit(" ")[-1])
                    if "Avg Target Lane Difference" in line:
                        current_tld[agent_name] = float(line.rsplit(" ")[-1])
                    if "Avg Target Vel Difference" in line:
                        current_tvd[agent_name] = float(line.rsplit(" ")[-1])
                    if "Laps Completed" in line:
                        lap_details = line.split(" ")[-1]
                        lap_f, lap_t = lap_details.split("/")
                        laps = int(lap_t)
                        if int(lap_f)/int(lap_t) != 1:
                            current_dnfs.add(agent_name)
                    if "Collisions" in line:
                        lap_details = line.split(" ")[-1]
            
            finishers = list(filter(lambda t: t not in current_dnfs, current_laps.keys()))
            if len(finishers):
                current_winner = min(finishers, key=lambda t: current_laps[t])
                ordering = list(sorted(finishers, key=lambda t: current_laps[t]))
            else:
                current_winner = ""
                ordering = []
            if len(current_winner) and current_winner not in current_dnfs:
                l = current_winner.split("(")[0]
                wins[l] = wins.get(l, 0) + 1
            for d in current_dnfs:
                l = d.split("(")[0]
                dnfs[l] = dnfs.get(l, 0) + 1
            if len(finishers) > 1 and current_winner != "":
                l = current_winner.split("(")[0]
                if l not in win_margins:
                    win_margins[l] = []
                win_margins[l].append(current_laps[max(finishers, key=lambda t: current_laps[t])] - current_laps[min(finishers, key=lambda t: current_laps[t])])
            for a in current_laps:
                agent_type = a.split("(")[0]
                l = agent_type
                if l not in points: points[l] = []
                if l not in collisions: collisions[l] = []
                if l not in illegal_changes: illegal_changes[l] = []
                if l not in current_dnfs:
                    if l not in lap_times: lap_times[l] = []
                    if l not in lane_differences: lane_differences[l] = []
                    if l not in vel_differences: vel_differences[l] = []
                    lap_times.get(l).append(current_laps[a])
                    lane_differences.get(l).append(current_tld[a])
                    vel_differences.get(l).append(current_tvd[a])
                collisions.get(l).append(current_collisions[a])
                illegal_changes.get(l).append(current_lcs[a])
                points[l].append(0)
                for idx, finisher in enumerate(ordering):
                    if finisher.startswith(l):
                        points[l][-1] += points_per_position[idx]
    avg_lap_times = {l: sum(lap_times[l])/(len(lap_times[l]*laps)) for l in lap_times}
    avg_win_margins = {l: sum(win_margins[l])/(len(win_margins[l])) for l in win_margins}
    avg_colls = {l: sum(collisions[l])/(len(collisions[l])) for l in collisions}
    avg_lcs = {l: sum(illegal_changes[l])/(len(illegal_changes[l])) for l in illegal_changes}
    avg_tld = {l: sum(lane_differences[l])/(len(lane_differences[l])) for l in lane_differences}
    avg_tvd = {l: sum(vel_differences[l])/(len(vel_differences[l])) for l in vel_differences}
    avg_points = {l : sum(points[l])/len(points[l]) for l in points}
    avg_safety_score = {l: (sum(collisions[l]+illegal_changes[l]))/len(collisions[l]) for l in collisions}
    stddv_safety_score = {l: (sum([((illegal_changes[l][i]+collisions[l][i])-avg_safety_score[l])**2 for i in range(len(collisions[l]))])/len(collisions[l]))**0.5 for l in collisions}

    print("Wins", wins)
    print("DNFs", dnfs)
    # print("Avg Lap Times", avg_lap_times)
    # print ("Avg Win Margins", avg_win_margins)
    print ("Avg Collisions", avg_colls)
    # print ("Avg Illegal Lane Changes", avg_lcs)
    print ("Avg Target Lane Distance", avg_tld)
    # print("Avg Target Vel Diff", avg_tvd)
    print("Avg Points Per Race", avg_points)
    print("Avg Safety Score", avg_safety_score)
    # print("Std Dev safety Score", stddv_safety_score)
    print()

oval_experiments = [
    "MCTS_LQR_vs_Fixed_LQR_Oval2",
    "Fixed_RL_vs_MCTS_LQR_Oval2", 
    "Fixed_RL_vs_Fixed_LQR_Oval2", 
    "E2E_vs_Fixed_RL_Oval2",
    "E2E_vs_MCTS_LQR_Oval2", 
    "E2E_vs_Fixed_LQR_Oval2", 
    "MCTS_RL_vs_E2E_Oval2", 
    "MCTS_RL_vs_Fixed_RL_Oval2", 
    "MCTS_RL_vs_MCTS_LQR_Oval2", 
    "MCTS_RL_vs_Fixed_LQR_Oval2",
]

complex_experiments = [
    "MCTS_LQR_vs_Fixed_LQR_Complex3",
    "Fixed_RL_vs_MCTS_LQR_Complex2", 
    "Fixed_RL_vs_Fixed_LQR_Complex2", 
    "E2E_vs_Fixed_LQR_Complex2", 
    "E2E_vs_MCTS_LQR_Complex2", 
    "E2E_vs_Fixed_RL_Complex2",
    "MCTS_RL_vs_MCTS_LQR_Complex2", 
    "MCTS_RL_vs_Fixed_LQR_Complex2",
    "MCTS_RL_vs_Fixed_RL_Complex2", 
    "MCTS_RL_vs_E2E_Complex2", 
]

for exp in oval_experiments:
    summarize_experiment(exp)

for exp in complex_experiments:
    summarize_experiment(exp)


print("Oval Experiments Aggregated:")
summarize_multiple_experiments(oval_experiments)
print("Complex Experiments Aggregated:")
summarize_multiple_experiments(complex_experiments)
print("All Experiments Aggregated:")
summarize_multiple_experiments(oval_experiments + complex_experiments)

print()
print()

duo_oval_experiments = [
    "MCTS_LQR_vs_Fixed_LQR_OvalDuos2",
    "Fixed_RL_vs_MCTS_LQR_OvalDuos2", 
    "Fixed_RL_vs_Fixed_LQR_OvalDuos2", 
    "E2E_vs_Fixed_RL_OvalDuos",
    "E2E_vs_MCTS_LQR_OvalDuos2", 
    "E2E_vs_Fixed_LQR_OvalDuos2", 
    "MCTS_RL_vs_E2E_OvalDuos", 
    "MCTS_RL_vs_Fixed_RL_OvalDuos", 
    "MCTS_RL_vs_MCTS_LQR_OvalDuos2", 
    "MCTS_RL_vs_Fixed_LQR_OvalDuos2",
]

duo_complex_experiments = [
    "MCTS_LQR_vs_Fixed_LQR_ComplexDuos2",
    "Fixed_RL_vs_MCTS_LQR_ComplexDuos2", 
    "Fixed_RL_vs_Fixed_LQR_ComplexDuos2", 
    "E2E_vs_Fixed_LQR_ComplexDuos2", 
    "E2E_vs_MCTS_LQR_ComplexDuos2", 
    "E2E_vs_Fixed_RL_ComplexDuos",
    "MCTS_RL_vs_MCTS_LQR_ComplexDuos2", 
    "MCTS_RL_vs_Fixed_LQR_ComplexDuos2",
    "MCTS_RL_vs_Fixed_RL_ComplexDuos", 
    "MCTS_RL_vs_E2E_ComplexDuos", 
]

for exp in duo_oval_experiments:
    summarize_experiment(exp)

for exp in duo_complex_experiments:
    summarize_experiment(exp)


print("Duo Oval Experiments Aggregated:")
summarize_multiple_experiments(duo_oval_experiments)
print("Duo Complex Experiments Aggregated:")
summarize_multiple_experiments(duo_complex_experiments)
print("Duo All Experiments Aggregated:")
summarize_multiple_experiments(duo_oval_experiments + duo_complex_experiments)