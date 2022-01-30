import os

logs_dir = "ExperimentLogs"

def summarize_experiment(experiment_name):
    wins = {}
    dnfs = {}
    lap_times = {}
    win_margins = {}
    collisions = {}
    illegal_changes = {}
    with open(os.path.join(logs_dir, experiment_name + ".txt")) as exp_log:
        current_dnfs = set()
        current_laps = {}
        for line in exp_log:
            if line.startswith("Experiment"):
                finishers = list(filter(lambda t: t not in current_dnfs, current_laps.keys()))
                if len(finishers):
                    current_winner = min(finishers, key=lambda t: current_laps[t])
                else:
                    current_winner = ""
                if len(current_winner) and current_winner not in current_dnfs:
                    wins[current_winner] = wins.get(current_winner, 0) + 1
                for d in current_dnfs:
                    dnfs[d] = dnfs.get(d, 0) + 1
                if len(finishers) > 1 and current_winner != "":
                    if current_winner not in win_margins:
                        win_margins[current_winner] = []
                    win_margins[current_winner].append(current_laps[max(finishers, key=lambda t: current_laps[t])] - current_laps[min(finishers, key=lambda t: current_laps[t])])
                for l in current_laps:
                    if l not in current_dnfs:
                        if l not in lap_times: lap_times[l] = []
                        if l not in collisions: collisions[l] = []
                        if l not in illegal_changes: illegal_changes[l] = []
                        lap_times.get(l).append(current_laps[l])
                        collisions.get(l).append(current_collisions[l])
                        illegal_changes.get(l).append(current_lcs[l])
                current_dnfs = set()
                current_collisions = {}
                current_lcs = {}
                current_laps = {}
            else:
                agent_type = line.split(" ")[0]
                if "Total Time" in line or "Overall Time" in line:
                    current_laps[agent_type] = float(line.rsplit(" ")[-1])
                if "Collisions" in line:
                    current_collisions[agent_type] = float(line.rsplit(" ")[-1])
                if "Illegal Lane Changes" in line:
                    current_lcs[agent_type] = float(line.rsplit(" ")[-1])
                if "Laps Completed" in line:
                    lap_details = line.split(" ")[-1]
                    lap_f, lap_t = lap_details.split("/")
                    laps = int(lap_t)
                    if int(lap_f)/int(lap_t) != 1:
                        current_dnfs.add(agent_type)
                if "Collisions" in line:
                    lap_details = line.split(" ")[-1]
        
        finishers = list(filter(lambda t: t not in current_dnfs, current_laps.keys()))          
        if len(finishers):
            current_winner = min(finishers,
                                key=lambda t: current_laps[t])
        else:
            current_winner = ""
        if len(current_winner) and current_winner not in current_dnfs:
            wins[current_winner] = wins.get(current_winner, 0) + 1
        for d in current_dnfs:
            dnfs[d] = dnfs.get(d, 0) + 1
        if len(finishers) > 1 and current_winner != "":
                    if current_winner not in win_margins:
                        win_margins[current_winner] = []
                    win_margins[current_winner].append(current_laps[max(finishers, key=lambda t: current_laps[t])] - current_laps[min(finishers, key=lambda t: current_laps[t])])
        for l in current_laps:
            if l not in current_dnfs:
                if l not in lap_times: lap_times[l] = []
                if l not in collisions: collisions[l] = []
                if l not in illegal_changes: illegal_changes[l] = []
                lap_times.get(l).append(current_laps[l])
                collisions.get(l).append(current_collisions[l])
                illegal_changes.get(l).append(current_lcs[l])
    avg_lap_times = {l: sum(lap_times[l])/(len(lap_times[l]*laps)) for l in lap_times}
    avg_win_margins = {l: sum(win_margins[l])/(len(win_margins[l])) for l in win_margins}
    avg_colls = {l: sum(collisions[l])/(len(collisions[l])) for l in collisions}
    avg_lcs = {l: sum(illegal_changes[l])/(len(illegal_changes[l])) for l in illegal_changes}

    print("Wins", wins)
    print("DNFs", dnfs)
    print("Avg Lap Times", avg_lap_times)
    print ("Avg Win Margins", avg_win_margins)
    print ("Avg Collisions", avg_colls)
    print ("Avg Illegal Lane Changes", avg_lcs)
    print()

summarize_experiment("MCTS_LQR_vs_Fixed_LQR_Complex")

summarize_experiment("Fixed_RL_vs_Fixed_LQR_Complex")
summarize_experiment("Fixed_RL_vs_MCTS_LQR_Complex")

summarize_experiment("E2E_vs_Fixed_RL_Complex")
summarize_experiment("E2E_vs_Fixed_LQR_Complex")
summarize_experiment("E2E_vs_MCTS_LQR_Complex")

summarize_experiment("MCTS_RL_vs_E2E_Complex")
summarize_experiment("MCTS_RL_vs_Fixed_RL_Complex")
summarize_experiment("MCTS_RL_vs_Fixed_LQR_Complex")
summarize_experiment("MCTS_RL_vs_MCTS_LQR_Complex")

summarize_experiment("MCTS_LQR_vs_Fixed_LQR_Oval")

summarize_experiment("Fixed_RL_vs_MCTS_LQR_Oval")
summarize_experiment("Fixed_RL_vs_Fixed_LQR_Oval")

summarize_experiment("E2E_vs_Fixed_RL_Oval")
summarize_experiment("E2E_vs_MCTS_LQR_Oval")
summarize_experiment("E2E_vs_Fixed_LQR_Oval")

summarize_experiment("MCTS_RL_vs_E2E_Oval")
summarize_experiment("MCTS_RL_vs_Fixed_RL_Oval")
summarize_experiment("MCTS_RL_vs_MCTS_LQR_Oval")
summarize_experiment("MCTS_RL_vs_Fixed_LQR_Oval")
