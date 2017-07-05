from .state_base import State

__author__ = 'xhou'


class Memory:
    valid_transition = {(State.acc, State.acc),
                        (State.acc, State.l_pre_turn),
                        (State.acc, State.r_pre_turn),
                        (State.acc, State.defense),

                        (State.l_pre_turn, State.l_pre_turn),
                        (State.l_pre_turn, State.l_turn),
                        (State.l_pre_turn, State.defense),

                        (State.l_turn, State.l_turn),
                        (State.l_turn, State.acc),
                        (State.l_turn, State.defense),

                        (State.r_pre_turn, State.r_pre_turn),
                        (State.r_pre_turn, State.r_turn),
                        (State.r_pre_turn, State.defense),

                        (State.r_turn, State.r_turn),
                        (State.r_turn, State.acc),
                        (State.r_turn, State.defense),

                        (State.defense, State.acc),
                        (State.defense, State.defense)}

    def __init__(self):
        self.timestamp = -1
        self.prev_state = None
        self.next_state = None
        self.debug_msg = ''
        self.scores = dict()
        self.traj = []
        self.turn_info = None
        self.defense_info = None

    def _turn_start(self, v_info, init_lane, target_lane):
        turn_info = dict()
        turn_info['start_time'] = self.timestamp
        turn_info['init_lane'] = init_lane
        turn_info['target_lane'] = target_lane
        turn_info['init_loc'] = v_info['abs_loc']
        turn_info['direction'] = 'l' if self.next_state == State.l_pre_turn else 'r'
        self.turn_info = turn_info

    def _defense_start(self, v_info, perc_info):
        defense_info = dict()
        defense_info['start_time'] = self.timestamp
        defense_info['v_info'] = v_info
        defense_info['perc_info'] = perc_info

    def update_memory(self, prev_state, next_state, debug_msg, timestamp, scores=dict(), traj=list(),
                      v_info=None, perc_info=None, init_lane=None, target_lane=None):
        self.debug_msg = debug_msg
        if next_state in State.valid_states:
            assert len(scores)
            assert len(traj)
        self.scores = scores
        assert self.timestamp == timestamp - 1, 'time_stamp not synchronized!'
        self.timestamp = timestamp
        assert (prev_state, next_state) in Memory.valid_transition, 'Prev-->Next transition not supported!'
        self.prev_state = prev_state
        self.next_state = next_state

        assert isinstance(traj, list), 'traj is not list()'
        self.traj = traj

        if prev_state in State.turn_states and next_state not in State.turn_states:
            self.turn_info = None
        if prev_state == State.defense and next_state != State.defense:
            self.defense_info = None

        if prev_state == State.acc and next_state in {State.l_pre_turn, State.r_pre_turn}:
            assert (v_info, init_lane, target_lane), 'Not enough input for _turn_start(): ACC-->Turn'
            self._turn_start(v_info, init_lane, target_lane)

        if next_state == State.defense:
            assert (v_info, perc_info), 'Not enough input for _defense_start(): -->Defense'
            self._defense_start(v_info, perc_info)

    def get_turn_info(self):
        assert self.next_state in State.turn_states, 'Turn info not supported for the current state'
        return self.turn_info

    def get_defense_info(self):
        return self.defense_info

    def get_traj(self):
        return self.traj
