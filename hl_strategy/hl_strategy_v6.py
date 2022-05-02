import itertools
import math
from datetime import datetime
from enum import Enum
from functools import lru_cache
from typing import Dict, List, TextIO

import numpy as np
from scipy.optimize import NonlinearConstraint, Bounds, minimize

from util import _write_with_newline_and_sc, neg_to_str, is_greater_than

MAX_TIRE_AGE = 100
RUN_STORMPY = False

@lru_cache(False)
def calc_min_time(u, v, s, a_min, a_max, v_max):
    constraints = []

    def opt(x):
        return x[3]

    def time_variables_constraint(x):
        return x[1] - x[0]
    nlc = NonlinearConstraint(time_variables_constraint, 0, np.inf)
    constraints.append(nlc)

    def time_variables_constraint2(x):
        return x[2] - x[1]
    nlc = NonlinearConstraint(time_variables_constraint2, 0, np.inf)
    constraints.append(nlc)

    def time_variables_constraint3(x):
        return x[3] - x[2]
    nlc = NonlinearConstraint(time_variables_constraint3, 0, np.inf)
    constraints.append(nlc)

    def distance_constraint(x):
        iv = u + a_max*x[0]
        dist = u*x[0] + 0.5*a_max*x[0]*x[0]
        dist += (x[1]-x[0])*iv
        fv = iv + -abs(a_min)*(x[2]-x[1])
        dist += (iv + fv)*(x[2]-x[1])/2
        dist += (x[3]-x[2])*fv
        return dist
    nlc = NonlinearConstraint(distance_constraint, s, s)
    constraints.append(nlc)

    def final_v_constraint(x):
        iv = u + a_max * x[0]
        dist = u * x[0] + a_max * x[0]
        dist += (x[1] - x[0]) * iv
        fv = iv + -abs(a_min) * (x[2] - x[1])
        return fv
    nlc = NonlinearConstraint(final_v_constraint, v, v)
    constraints.append(nlc)

    def max_vel_constraint(x):
        iv = u+ a_max*x[0]
        return iv
    nlc = NonlinearConstraint(max_vel_constraint, 0, v_max)
    constraints.append(nlc)

    lb = [0, 0, 0, 0]
    ub = [100, 100, 100, 100]
    bounds = Bounds(lb, ub)
    x0 = [0.1,0.2,0.3,0.4]
    result = minimize(opt, x0, constraints=constraints, bounds=bounds)
    if (is_greater_than(max_vel_constraint(result.x), v_max, abs_tol=1e-4) or not math.isclose(distance_constraint(result.x), s, abs_tol=1e-4)
            or not math.isclose(final_v_constraint(result.x), v, abs_tol=1e-4)
            or result.x[0] <= -1e-5 or result.x[1] <= -1e-5):
        # print(max_vel_constraint(result.x), distance_constraint(result.x), s, u, v, result.x)
        # print()
        return None
    return result.fun


class CarDef:
    def __init__(self, min_velocity, max_velocity, main_velocity_step, init_velocity_step, min_gs, max_gs, max_braking, max_acceleration, tire_wear_factor,
                 init_tire, init_time, init_line, init_velocity, init_position):
        self.min_v = min_velocity
        self.max_v = max_velocity
        self.mvs = main_velocity_step
        self.ivs = init_velocity_step
        self.max_braking = -abs(max_braking)
        self.max_acceleration = max_acceleration
        self.min_gs = min_gs
        self.max_gs = max_gs
        self.init_tire = init_tire
        self.init_time = init_time
        self.init_line = init_line
        self.init_velocity = init_velocity
        self.init_position = init_position
        self.tire_wear_factor = tire_wear_factor


class TrackComponent:
    def __init__(self, lengths, min_v, max_v):
        self.lengths = lengths
        self.min_v = min_v
        self.max_v = max_v

    def is_v_feasible(self, velocity, line, tire_wear, min_cornering_gs, max_cornering_gs):
        raise NotImplementedError("Must be implemented by child classes")

    def tire_wear(self, velocity, line, tire_wear_factor):
        raise NotImplementedError("Must be implemented by child classes")


class TrackStraight(TrackComponent):
    def __init__(self, length, num_lines, min_v, max_v, component_wear_factor=.0125):
        super().__init__([length] * num_lines, min_v, max_v)
        self.cwf = component_wear_factor

    def is_v_feasible(self, velocity, line, tire_wear, min_cornering_gs, max_cornering_gs):
        return True

    def tire_wear(self, velocity, line, tire_wear_factor):
        if velocity == 0: return 0
        return int(self.lengths[line]*velocity*tire_wear_factor*self.cwf)

class Entry(TrackComponent):
    def __init__(self, lengths, turn_radius, width, num_lines, min_v, max_v, left_turn=True, component_wear_factor=.075):
        super().__init__(lengths, min_v, max_v)
        order = range(num_lines) if left_turn else reversed(range(num_lines))
        self.tr = list(map(lambda l: turn_radius + l*width/num_lines, order))
        self.cwf = component_wear_factor

    def is_v_feasible(self, velocity, line, tire_wear, min_cornering_gs, max_cornering_gs):
        gs = (velocity**2)/self.tr[line]
        g_diff = (max_cornering_gs-min_cornering_gs)*(tire_wear/MAX_TIRE_AGE)
        return gs <= max_cornering_gs-g_diff

    def tire_wear(self, velocity, line, tire_wear_factor):
        gs = (velocity**2)/self.tr[line]
        return int((math.ceil(gs) * self.lengths[line]/velocity)*tire_wear_factor * self.cwf)


class Mid(TrackComponent):
    def __init__(self, lengths, turn_radius, width, num_lines,min_v, max_v, left_turn=True, component_wear_factor=.075):
        super().__init__(lengths, min_v, max_v)
        order = range(num_lines) if left_turn else reversed(range(num_lines))
        self.tr = list(map(lambda l: turn_radius + l*width/num_lines, order))
        self.cwf = component_wear_factor

    def is_v_feasible(self, velocity, line, tire_wear, min_cornering_gs, max_cornering_gs):
        gs = (velocity**2)/self.tr[line]
        g_diff = (max_cornering_gs-min_cornering_gs)*(tire_wear/MAX_TIRE_AGE)
        return gs <= max_cornering_gs-g_diff

    def tire_wear(self, velocity, line, tire_wear_factor):
        gs = (velocity**2)/self.tr[line]
        return int((math.ceil(gs) * self.lengths[line]/velocity)*tire_wear_factor * self.cwf)


class TrackCorner:
    def __init__(self, num_lines, width, inside_turn_radius, degrees, exit_length, min_v, max_v, left_turn=True):
        order = range(num_lines) if left_turn else reversed(range(num_lines))
        turn_lengths = [math.radians(degrees) * radius for radius in map(lambda l: inside_turn_radius + l*width/num_lines, order)]
        self.entry = Entry([turn_length/2 for turn_length in turn_lengths], inside_turn_radius, width, num_lines, min_v[0], max_v[0], left_turn)
        self.mid = Mid([turn_length/2 for turn_length in turn_lengths], inside_turn_radius, width, num_lines, min_v[1], max_v[1], left_turn)
        self.exit = TrackStraight(exit_length, num_lines, min_v[2], max_v[2])


class TrackChicane:
    def __init__(self, num_lines, width, inside_turn_radius, degrees, exit_length, min_v, max_v, left_turn_first=True):
        order = list(range(num_lines)) if left_turn_first else list(reversed(range(num_lines)))
        turn_lengths_entry = [math.radians(degrees) * radius for radius in map(lambda l: inside_turn_radius + l*width/num_lines, order)]
        turn_lengths_mid = [math.radians(degrees) * radius for radius in map(lambda l: inside_turn_radius + l*width/num_lines, reversed(order))]
        self.entry = Entry([turn_length for turn_length in turn_lengths_entry], inside_turn_radius, width, num_lines, min_v[0], max_v[0], left_turn_first)
        self.mid = Mid([turn_length for turn_length in turn_lengths_mid], inside_turn_radius, width, num_lines, min_v[1], max_v[1], not left_turn_first)
        self.exit = TrackStraight(exit_length, num_lines, min_v[2], max_v[2])


class TrackDef:
    def __init__(self, track_landmarks, width, num_lanes, pit_exit_position, pit_exit_velocity, pit_exit_line, pit_time):
        self.num_lanes = num_lanes
        self.width = width
        self.pit_exit_p = pit_exit_position
        self.pit_exit_l = pit_exit_line
        self.pit_exit_v = pit_exit_velocity
        self.pit_time = pit_time
        self.landmarks = self._process_landmarks(track_landmarks)
        self.track_points= len(self.landmarks)

    def _process_landmarks(self, input_landmarks):
        output = []
        for l in input_landmarks:
            if type(l) == TrackStraight:
                output.append(l)
            elif type(l) == TrackCorner or type(l) == TrackChicane:
                output.append(l.entry)
                output.append(l.mid)
                output.append(l.exit)
        return output


class TimePrecision(Enum):
    Hundredths = 100
    Tenths = 10
    Seconds = 1


def generate_modules(output_file, total_seconds, laps, track_definition, car_definitions, time_precision, crash_tolerance=.5, is_game=False, game_type='smg', allow_worn_progress=True):
    with open(output_file, "w+") as output:
        if not is_game:
            _write_with_newline_and_sc("mdp\n", output, False)
        else:
            _write_with_newline_and_sc(f"{game_type}\n", output, False)

        tps = track_definition.track_points
        tls = track_definition.num_lanes
        pit_out = track_definition.pit_exit_p
        pit_out_v = track_definition.pit_exit_v
        pit_out_l = track_definition.pit_exit_l
        max_time = total_seconds * time_precision.value
        pit_time = track_definition.pit_time * time_precision.value
        _write_with_newline_and_sc(f"const int max_time", output)
        _write_with_newline_and_sc(f"const int num_laps", output)
        player_action_str = []
        final_player_action_str = {i: set() for i in range(len(car_definitions))}
        active_section_strings = {}
        for i in range(tps):
            active_section_strings[i] = f"section{i}_active"
            _write_with_newline_and_sc(f"formula {active_section_strings[i]} = track_pos={i} ? 1 : 0", output)

        for idx, car_definition in enumerate(car_definitions):
            if idx == 0: _write_with_newline_and_sc(f"const int p{idx}_init_tg", output)
            _write_with_newline_and_sc(f"const int p{idx}_init_ta", output)
            _write_with_newline_and_sc(f"const int p{idx}_init_v", output)

            max_v = car_definition.max_v
            init_tire = car_definition.init_tire
            init_time = car_definition.init_time
            init_line = car_definition.init_line
            init_v = car_definition.init_velocity
            init_pos = car_definition.init_position
            velocity_step = car_definition.mvs

            # Define Fixed Action Set
            action_set = {}
            for velocity in range(car_definition.min_v, max_v+math.ceil(car_definition.mvs/2), car_definition.mvs):
                ub = min(max_v+1, velocity+car_definition.mvs)
                for lane in range(tls):
                    action_set[(velocity, ub, lane)] = f"[step{idx}_b{velocity}_a{ub}_l{lane}]"

            not_allowed_actions = {i: set() for i in range(tps)}
            # Define Track Section Formulas

            for action in list(action_set.keys()):
                never_allowed = True
                for i in range(tps):
                    avg_v = (action[0] + min(action[1]-1, max_v))/2
                    target_section = track_definition.landmarks[(i)%tps]
                    for ta in range(0, MAX_TIRE_AGE+1):
                        if not target_section.is_v_feasible(avg_v, action[2], ta, car_definition.min_gs, car_definition.max_gs): break
                    # print(target_section.max_v, target_section.min_v, action[0], action[1])
                    if ta == 0 or (action[0] > target_section.max_v) or (action[1] < target_section.min_v):
                        not_allowed_actions[i].add(action)
                    else:
                        never_allowed = False
                    # if ta == 0: not_allowed_actions[i].add(action)
                    _write_with_newline_and_sc(f"formula sec{i}_b{action[0]}_a{action[1]}_l{action[2]}_c{idx} = tire_age{idx} < {ta} & track_pos={i} & lap<num_laps", output)
                if never_allowed:
                    del action_set[action]
            player_action_str.append(list(action_set.values()))

            # Define Formulas for allowed actions
            action_allowed_strings = {}
            for action in action_set:
                action_allowed_strings[action] = f"b{action[0]}_a{action[1]}_l{action[2]}_c{idx}"
                _write_with_newline_and_sc(f"formula b{action[0]}_a{action[1]}_l{action[2]}_c{idx} = {' | '.join(map(lambda i:f'sec{i}_b{action[0]}_a{action[1]}_l{action[2]}_c{idx}', range(tps)))}", output)

            # Car Module
            _write_with_newline_and_sc(f'module racecar{idx}\n', output, False)
            _write_with_newline_and_sc(f"t{idx} : [0..max_time] init {init_time if idx!=0 else f'p{idx}_init_tg'}", output)
            _write_with_newline_and_sc(f'track_lane{idx} : [0..{max(1, tls-1)}] init {init_line}', output)
            _write_with_newline_and_sc(f'lane_changes{idx} : [0..2] init 0', output)
            _write_with_newline_and_sc(f'velocity{idx} : [1..{max_v}] init p{idx}_init_v', output)
            _write_with_newline_and_sc(f'reached{idx} : bool init false', output)
            _write_with_newline_and_sc(f'worn{idx} : bool init false', output)
            for cur_lane in range(tls):
                for cur_v in range(car_definition.min_v, car_definition.max_v+math.ceil(car_definition.ivs/2), car_definition.ivs):
                    for action, action_string in action_set.items():
                        action_str = f"{action_string}"
                        avg_init_v = (cur_v + min(cur_v+car_definition.ivs-1, max_v))/2
                        max_dt = 0
                        updates = []
                        t_updates = []
                        always_allowed=True
                        for v in range(action[0], action[1]):
                            t_update_str = f"t{idx}"
                            pos_guard_str = f"false"
                            lane_change_count_update = f"lane_changes{idx}'=lane_changes{idx}"
                            for section_idx, section in enumerate(track_definition.landmarks):
                                if action in not_allowed_actions[section_idx] or v > section.max_v or v < section.min_v:
                                    # print(action, v, section.min_v, section.max_v)
                                    always_allowed = False
                                    continue
                                dist = math.sqrt(
                                    ((section.lengths[cur_lane]) ** 2) + ((abs(cur_lane - action[2]) * (track_definition.width) / (tls-1)) ** 2)) if tls > 1 else section.lengths[cur_lane]
                                min_time = calc_min_time(avg_init_v, v, dist,
                                                         car_definition.max_braking, car_definition.max_acceleration, car_definition.max_v)
                                if min_time is None:
                                    always_allowed=False
                                    continue
                                else: dt = math.ceil(min_time*time_precision.value)
                                max_dt = max(max_dt, dt)
                                t_update_str += f"+({dt}*{active_section_strings[section_idx]})"
                                lane_change_count_update += f"+ ({1 if type(section)==TrackStraight else f'-lane_changes{idx}'} * {active_section_strings[section_idx]})"
                                pos_guard_str += f" | track_pos={section_idx}"
                            if t_update_str == f"t{idx}": continue
                            update_str = f"(velocity{idx}'={v})&(track_lane{idx}'={action[2]})&(t{idx}'={t_update_str})&({lane_change_count_update})" if cur_lane != action[2] else f"(velocity{idx}'={v})&(track_lane{idx}'={action[2]})&(t{idx}'={t_update_str})"
                            # print(update_str)
                            updates.append(update_str)
                            t_updates.append(f"({t_update_str})")
                        lane_change_guard = "true" if cur_lane == action[2] else f"lane_changes_allowed{idx}"
                        if not len(updates): continue
                        prob_str = f"1/{len(updates)}"
                        crash_guards = []
                        for t_update in t_updates:
                            for opp in range(len(car_definitions)):
                                if opp ==idx: continue
                                crash_guards.append(
                                    f"((turn{opp}=1) & ({action[2]} = track_lane{opp}) & ({t_update}-t{opp}<{max(1, crash_tolerance * time_precision.value)}) & ({t_update}-t{opp} >-{max(1, crash_tolerance * time_precision.value)}))")
                        crash_guard = f"!({' | '.join(crash_guards)})"
                        for i in range(len(updates)):
                            updates[i] = f"{prob_str}:{updates[i]}"
                        if pos_guard_str == "false":continue
                        guard_str = f"(p{idx}_go) & ({action_allowed_strings[action]}) & (track_lane{idx}={cur_lane}) & (t{idx}<max_time-{max_dt}) " \
                                    f"& (velocity{idx}>={cur_v}) & (velocity{idx} < {cur_v + car_definition.ivs}) & ({pos_guard_str if not always_allowed else 'true'}) " \
                                    f"& ({lane_change_guard}) & ({crash_guard})"
                        _write_with_newline_and_sc(f"{action_str} {guard_str} -> {' + '.join(updates)}", output)
                        final_player_action_str[idx].add(action_str)

            action = f"[pit_{idx}]"
            guard = f"(p{idx}_go) & (track_pos={tps-1}) & (t{idx}<max_time-{pit_time})"
            _write_with_newline_and_sc(
                f"{action} {guard} -> 1:(velocity{idx}'={min(max(car_definition.min_v, pit_out_v), max_v)}) & (track_lane{idx}'={pit_out_l}) & (t{idx}'=t{idx}+{pit_time})", output)

            _write_with_newline_and_sc(f"[lap_update] true -> (t{idx}'=t{idx}-min({','.join(map(lambda i: f't{i}', range(len(car_definitions))))}))", output)

            action = f"[worn_{idx}]"
            guard = f"p{idx}_go"
            _write_with_newline_and_sc(
                f"{action} {guard} -> 1:(worn{idx}'=true)",
                output)

            action = f"[goal_{idx}]"
            guard = f"p{idx}_go & lap=num_laps & !reached{idx}"
            _write_with_newline_and_sc(
                f"{action} {guard} -> 1:(reached{idx}'=true)",
                output)
            _write_with_newline_and_sc("endmodule", output, False)
            _write_with_newline_and_sc("", output, False)

            # Tire Age Module
            _write_with_newline_and_sc(f'module tire_wear{idx}\n', output, False)
            _write_with_newline_and_sc(f'tire_age{idx} : [0..{MAX_TIRE_AGE + 00}] init p{idx}_init_ta', output)
            for i, section in enumerate(track_definition.landmarks):
                for action, action_string in action_set.items():
                    if action_string not in final_player_action_str[idx]: continue
                    action_str = f"{action_string}"
                    max_dta = 0
                    updates = []
                    for v in range(action[0], action[1]):
                        if v > section.max_v or v < section.min_v: continue
                        dta = section.tire_wear(v, action[2], car_definition.tire_wear_factor)
                        max_dta = max(max_dta, dta)
                        updates.append(f"(tire_age{idx}'=tire_age{idx}+{dta})")
                    prob_str = f"1/{len(updates)}"
                    for index in range(len(updates)):
                        updates[index] = f"{prob_str}:{updates[index]}"
                    guard_str = f"track_pos={i} & tire_age{idx} < {MAX_TIRE_AGE + 00 - max_dta}"
                    if len(updates):
                        _write_with_newline_and_sc(f"{action_str} {guard_str} -> {' + '.join(updates)}", output)

            _write_with_newline_and_sc(f"[pit_{idx}] track_pos={tps - 1} -> 1: (tire_age{idx}'=0)", output)
            _write_with_newline_and_sc(f"[goal_{idx}] true -> 1: (tire_age{idx}'=tire_age{idx})", output)
            _write_with_newline_and_sc("endmodule", output, False)

            _write_with_newline_and_sc(f'label \"goal{idx}\" = reached{idx}', output)

            if is_game:
                # Player definitions
                _write_with_newline_and_sc(f"player p{idx}", output, False)
                worn_actions = f"[worn_{idx}]"
                if allow_worn_progress: worn_actions = f"{', '.join(map(lambda l: f'[worn_{idx}_l{l}]', range(tls)))}"
                _write_with_newline_and_sc(f"racecar{idx}, {', '.join(final_player_action_str[idx])}, [pit_{idx}], [goal_{idx}], [worn_{idx}]", output, False)
                _write_with_newline_and_sc("endplayer", output, False)

            # Lane changing rules
            if len(car_definitions) > 1:
                near_strings = []
                for opp in range(len(car_definitions)):
                    if opp == idx: continue
                    near_strings.append(
                        f"(t{idx}-t{opp}<=0 & t{idx}-t{opp} >=-{2 * time_precision.value}) & turn{opp}=0")
                car_near_and_ahead_condition = f"{' | '.join(near_strings)}"
                track_pos_condition = f"{' | '.join(map(lambda pos: f'track_pos={track_def.landmarks.index(pos)}', filter(lambda landmark: type(landmark)==TrackStraight, track_def.landmarks)))}"
                num_change_condition = f"lane_changes{idx} < 1"
                _write_with_newline_and_sc(f"formula lane_changes_allowed{idx} = (({car_near_and_ahead_condition}) & ({track_pos_condition}))? {num_change_condition} : true", output)
            else:
                _write_with_newline_and_sc(f"formula lane_changes_allowed{idx} = true", output)

        if len(car_definitions) > 1:
            crash_strings = []
            for pair in itertools.combinations(list(range(len(car_definitions))), 2):
                crash_strings.append(f"((turn{pair[0]} = turn{pair[1]}) & (track_lane{pair[0]} = track_lane{pair[1]}) & (t{pair[0]}-t{pair[1]}<{max(1, crash_tolerance*time_precision.value)} & t{pair[0]}-t{pair[1]} >-{max(1, crash_tolerance*time_precision.value)}))")
            _write_with_newline_and_sc(f"label \"crash\" = {' | '.join(crash_strings)}", output)
            _write_with_newline_and_sc(f"formula is_crash = {' | '.join(crash_strings)}", output)
        _write_with_newline_and_sc(f"formula worn_game = {' | '.join(map(lambda i: f'worn{i}', range(len(car_definitions))))}", output)
        if game_type == 'smg':
            _write_with_newline_and_sc(f"label \"end\" = end_state", output)
            # Turn Module
            _write_with_newline_and_sc("module turns", output, False)
            for i in range(len(car_definitions)):
                _write_with_newline_and_sc(f"turn{i}: [0..1] init 0", output)
            _write_with_newline_and_sc(f"track_pos: [0..{max(1, tps-1)}] init 0", output)
            _write_with_newline_and_sc(f"lap: [0..num_laps] init 0", output)
            _write_with_newline_and_sc(f"end_state: bool init false", output)
            if len(car_definitions) > 1:
                for i in range(len(car_definitions)):
                    for action_str in filter(lambda action_str: action_str in final_player_action_str[i], player_action_str[i]):
                        _write_with_newline_and_sc(f"{action_str} !end_state -> (turn{i}'=1)", output)

                    _write_with_newline_and_sc(
                        f"[pit_{i}] !end_state -> (turn{i}'=1)", output)
                    _write_with_newline_and_sc(
                        f"[goal_{i}] !end_state -> (turn{i}'=1)", output)
                    _write_with_newline_and_sc(
                        f"[worn_{i}] !end_state -> (turn{i}'=1)", output)
                _write_with_newline_and_sc(f"[end_update] !end_state & (({' & '.join(map(lambda i: f'reached{i}', range(len(car_definitions))))}) | (worn_game & {' & '.join(map(lambda i: f'turn{i}=1', range(len(car_definitions))))})) -> (end_state'=true)", output)
                _write_with_newline_and_sc(
                    f"""[pos_update] !worn_game & !end_state & {' & '.join(map(lambda i: f'turn{i}=1', range(len(car_definitions))))} & track_pos < {tps - 1} -> (track_pos'=track_pos+1)& {' & '.join(map(lambda i: f"(turn{i}'=0)", range(len(car_definitions))))}""",
                    output)
                _write_with_newline_and_sc(
                    f"""[lap_update] !worn_game & !end_state & {' & '.join(map(lambda i: f'turn{i}=1', range(len(car_definitions))))} & track_pos = {tps - 1} -> (track_pos'=0) & (lap'=lap+1) & {' & '.join(map(lambda i: f"(turn{i}'=0)", range(len(car_definitions))))}""",
                    output)
            else:
                for action_str in filter(lambda action_str: action_str in final_player_action_str[i],
                                         player_action_str[i]):
                    _write_with_newline_and_sc(f"{action_str} !end_state -> (turn{i}'=1)", output)
                # if not allow_worn_progress:
                #     _write_with_newline_and_sc(f"[worn_{i}] true -> (turn{i}'=1)", output)
                # else:
                #     for l in range(tls):
                #         _write_with_newline_and_sc(f"[worn_{i}_l{l}] true -> (turn{i}'=1)", output)
                _write_with_newline_and_sc(f"[pit_{i}] !end_state -> (turn{i}'=1)", output)
                _write_with_newline_and_sc(f"[goal_{i}] !end_state -> (turn{i}'=1)", output)
                _write_with_newline_and_sc(f"[worn_{i}] !end_state -> (turn{i}'=1)", output)
                _write_with_newline_and_sc(
                    f"[pos_update] !worn_game & !end_state & turn{i}=1 & track_pos < {tps - 1} -> (track_pos'=track_pos+1) & (turn{i}'=0)",
                    output)
                _write_with_newline_and_sc(
                    f"[lap_update] !worn_game & !end_state & turn{i}=1 & track_pos = {tps - 1} & lap < num_laps -> (track_pos'=0) & (turn{i}'=0) & (lap'=lap+1)",
                    output)

            _write_with_newline_and_sc("endmodule\n", output, False)

            _write_with_newline_and_sc("player scheduler", output, False)
            _write_with_newline_and_sc("turns, [lap_update], [pos_update], [end_update]", output, False)
            _write_with_newline_and_sc("endplayer", output, False)

            # Define Formulas for who gets to choose an action first
            if len(car_definitions) > 1:
                _write_with_newline_and_sc(f"const int M=1000", output)
                for idx in range(len(car_definitions)):
                    time_comp_str = f"t{idx}=min({','.join(map(lambda i: f't{i} + turn{i}*M', range(len(car_definitions))))})"
                    turn_gone_str = f"turn{idx}=0"
                    tie_breaker_str = ' & '.join(map(lambda i: f'!p{i}_go', range(idx)))
                    _write_with_newline_and_sc(
                        f"""formula p{idx}_go = {time_comp_str} & {turn_gone_str} {'& ' + tie_breaker_str if len(tie_breaker_str) else ''} & !reached{idx}""", output)
            else:
                _write_with_newline_and_sc("formula p0_go = !reached0", output)


        if is_game and len(car_definitions) > 1:
            for idx in range(len(car_definitions)):
                _write_with_newline_and_sc(f"rewards \"time_diff{idx}\"", output, False)
                guard = f"lap=num_laps & {' & '.join(map(lambda i: f'!reached{i}', range(len(car_definitions))))} & !worn{idx}"
                if len(car_definitions) == 2:
                    min_t_str = f"t{1 if idx==0 else 0}"
                else:
                    min_t_str = f"min({','.join(map(str, filter(lambda i: i != idx, range(len(car_definitions)))))})"
                _write_with_newline_and_sc(f"{guard}: max_time + {min_t_str}-t{idx}", output)

                guard = f"!end_state & !worn{idx} & ({' & '.join(map(lambda i: f'turn{i}=1', range(len(car_definitions))))}) & ({' | '.join(map(lambda i: f'worn{i}', filter(lambda i: i != idx, range(len(car_definitions)))))})"
                _write_with_newline_and_sc(f"{guard}: 1000 - track_pos", output)
                _write_with_newline_and_sc("endrewards", output, False)


if __name__ == "__main__":
    LANES = 3
    WIDTH = 10
    LENGTH = 2
    pit_exit_p = 1
    pit_exit_v = 3
    pit_exit_line = 0
    pit_time = 5
    MAX_TIRE_AGE = 100

    ## SCENARIO 1
    # components = [TrackStraight(3, LANES), TrackStraight(3, LANES), TrackStraight(3, LANES), TrackStraight(3, LANES),TrackStraight(3, LANES),]

    ## SCENARIO 2
    components = [TrackStraight(15, LANES, 10, 20), TrackStraight(15, LANES, 10, 20), TrackCorner(num_lines=LANES, width=WIDTH, inside_turn_radius=10, degrees=180, exit_length=15, left_turn=False, min_v=[5, 5, 8], max_v=[15, 15, 20]),]
    # components = [TrackStraight(30, LANES, 30, 50), TrackStraight(30, LANES, 30, 60),]


    ## SCENARIO 3
    # components = [TrackStraight(3, LANES), TrackStraight(3, LANES), TrackChicane(num_lines=LANES, width=WIDTH, inside_turn_radius=2.5, degrees=80, exit_length=3, left_turn_first=False),]

    track_def = TrackDef(components, width=WIDTH, num_lanes=LANES, pit_exit_position=pit_exit_p, pit_exit_velocity=pit_exit_v, pit_exit_line=pit_exit_line, pit_time=pit_time)
    # car_def1 = CarDef(min_velocity=10, max_velocity=70, main_velocity_step=10, init_velocity_step=5, min_gs=2*9.8, max_gs=3*9.8, max_braking=20, max_acceleration=9,
    #                  tire_wear_factor=.08,
    #                  init_time=0, init_tire=0, init_line=0, init_velocity=1, init_position=0)
    # car_def2 = CarDef(min_velocity=10, max_velocity=65, main_velocity_step=10, init_velocity_step=5, min_gs=2*9.8, max_gs=3.5*9.8, max_braking=20, max_acceleration=12,
    #                  tire_wear_factor=.1,
    #                  init_time=0, init_tire=0, init_line=0, init_velocity=3, init_position=0)
    # print(datetime.now())
    car_def1 = CarDef(min_velocity=5, max_velocity=20, main_velocity_step=2, init_velocity_step=1, min_gs=.7*9.8, max_gs=2*9.8, max_braking=9.8, max_acceleration=7,
                     tire_wear_factor=.4,
                     init_time=0, init_tire=0, init_line=0, init_velocity=1, init_position=0)
    car_def2 = CarDef(min_velocity=3, max_velocity=17, main_velocity_step=2, init_velocity_step=1, min_gs=.7*9.8, max_gs=2.5*9.8, max_braking=9.8, max_acceleration=9,
                     tire_wear_factor=.5,
                     init_time=0, init_tire=0, init_line=0, init_velocity=3, init_position=0)
    print(datetime.now())
    print("Generating Prism Program...")
    generate_modules('two_player_smg.prism', total_seconds=500, laps=10, track_definition=track_def, car_definitions=[car_def1, car_def2], time_precision=TimePrecision.Tenths, is_game=True, allow_worn_progress=False)
    print("Generated Prism Program")
    print(datetime.now())