from sklearn.linear_model import Ridge
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
from collections import defaultdict
import numpy as np

__author__ = ['xhou', 'zpyan']

NANOSECONDS_IN_SECOND = 1e9


# TODO: better logging
class Predictor:
    normal = 'normal'
    overspeed = 'overspeed'
    underspeed = 'underspeed'
    l_turn = 'l_turn'
    r_turn = 'r_turn'
    unknown = 'unknown'
    reckless = 'reckless'

    '''
    For each car:
    rel_l (opt)
    rel_d (opt)
    rel_lv (opt)
    rel_dv (opt)
    rel_x
    rel_y
    rel_xv
    rel_yv
    state
    state_conf
    '''

    def __init__(self, reg_win=0, fps=20., reckless_th=(1e6, 1e6), unknown_th=10,
                 pos_reg_cfg={}, vel_reg_cfg={}, frame_to_pred=5, lr_th=1.5, ou_th=5.):
        if unknown_th < frame_to_pred:
            print "[Warning] Unknown_th smaller than frame_to_pred, state confidence will be low"
        self._car_perc = defaultdict(lambda: defaultdict(list))
        self.reg_win = reg_win
        self.fps = fps
        self.reckless_th = reckless_th
        self.unknown_th = unknown_th
        self.pos_reg_cfg = pos_reg_cfg
        self.vel_reg_cfg = vel_reg_cfg
        self.frame_to_pred = frame_to_pred
        self.lr_th = lr_th
        self.ou_th = ou_th

    def _update(self, new_perc, ts):
        for traj_id, value in new_perc.items():
            car_perc = self._car_perc[traj_id]
            car_perc['ts'].append(ts / NANOSECONDS_IN_SECOND)
            car_perc['pos'].append(value[:3])
            car_perc['vel'].append(value[3:6])
            car_perc['len'].append(value[6:9])
            car_perc['conf'].append(value[9])
            car_perc['dist'].append(value[10])
            car_perc['move'].append(value[11])

    # TODO: get a proper reckless threshold
    def _is_reckless(self, traj_id):
        car_perc = self._car_perc[traj_id]
        xs = np.array(car_perc['pos'])[:, 0]
        ys = np.array(car_perc['pos'])[:, 1]
        xvs = np.array(car_perc['vel'])[:, 0]
        yvs = np.array(car_perc['vel'])[:, 1]

        pos_std = max(np.std(xs), np.std(ys))
        vel_std = max(np.std(yvs), np.std(xvs))
        reck_state = pos_std > self.reckless_th[0] or vel_std > self.reckless_th[1]
        reck_state_conf = 1. if reck_state else 1 - max(pos_std/self.reckless_th[0], vel_std/self.reckless_th[1])
        return reck_state, reck_state_conf

    def _is_unknown(self, traj_id):
        return len(self._car_perc[traj_id]['ts']) <= 10

    def _regress(self, x, y, ts_to_pred, reg_cfg={}, feature='linear'):
        if feature == 'linear':
            model = Ridge(**reg_cfg)
        elif feature == 'quadratic':
            model = make_pipeline(PolynomialFeatures(2), Ridge(**reg_cfg))
        else:
            raise ValueError('Invalid feature:'.format(feature))
        model.fit(x, y)
        return model.predict(ts_to_pred)

    def _predict(self, traj_id):
        car_perc = self._car_perc[traj_id]
        ts_train = np.array(car_perc['ts'])[-self.reg_win:, None]
        ts_to_pred = np.array([ts_train[-1] + (1 + i) / self.fps for i in xrange(self.frame_to_pred)])

        # perc regression
        pos_train = np.array(car_perc['pos'])[-self.reg_win:]
        xpos_pred = self._regress(ts_train, pos_train[:, 0], ts_to_pred, reg_cfg=self.pos_reg_cfg, feature='quadratic')
        ypos_pred = self._regress(ts_train, pos_train[:, 1], ts_to_pred, reg_cfg=self.pos_reg_cfg, feature='quadratic')
        vel_train = np.array(car_perc['vel'])[-self.reg_win:]
        xvel_pred = self._regress(ts_train, vel_train[:, 0], ts_to_pred, reg_cfg=self.vel_reg_cfg, feature='linear')
        yvel_pred = self._regress(ts_train, vel_train[:, 1], ts_to_pred, reg_cfg=self.vel_reg_cfg, feature='linear')
        ret_perc = {'pos': zip(list(xpos_pred), list(ypos_pred)),
                    'vel': zip(list(xvel_pred), list(yvel_pred))}

        # state classification
        # n, l, r, over, under, unk, reck
        state = [0] * 7
        if np.mean(ypos_pred) > self.lr_th:
            state[1] = 1
        if np.mean(ypos_pred) < -self.lr_th:
            state[2] = 1
        if np.mean(xvel_pred) > self.ou_th and np.any(0 > xpos_pred):
            state[3] = 1
        if np.mean(xvel_pred) < self.ou_th and np.any(0 < xpos_pred):
            state[4] = 1
        if not any(state[1:6]):
            state[0] = 1

        # state confidence
        test_frame_for_conf = min(ts_train.size - self.frame_to_pred, self.frame_to_pred)
        state_conf = np.ones(7) * float(test_frame_for_conf) / self.frame_to_pred
        ts_train_conf = ts_train[:-test_frame_for_conf]
        ts_test_conf = ts_train[-test_frame_for_conf:]

        pos_train_conf = pos_train[:-test_frame_for_conf]
        ypos_pred_conf = self._regress(ts_train_conf, pos_train_conf[:, 1], ts_test_conf,
                                       reg_cfg=self.pos_reg_cfg, feature='quadratic')
        ypos_test_conf = pos_train[-test_frame_for_conf:, 1]
        pos_state_err = np.mean(np.abs(np.divide(ypos_test_conf - ypos_pred_conf, ypos_test_conf + 1e-6)))

        vel_train_conf = vel_train[:-test_frame_for_conf]
        xvel_pred_conf = self._regress(ts_train_conf, vel_train_conf[:, 0], ts_test_conf,
                                       reg_cfg=self.vel_reg_cfg, feature='quadratic')
        xvel_test_conf = vel_train[-test_frame_for_conf:, 0]
        vel_state_err = np.mean(np.abs(np.divide(xvel_test_conf - xvel_pred_conf, xvel_test_conf + 1e-6)))

        state_conf[1:3] *= 1 - pos_state_err
        state_conf[3:5] *= 1 - vel_state_err
        return ret_perc, state, state_conf

    def _wrap_ret_perc(self, data, state, state_conf):
        """

        :param data: {'pos': (x, y), 'vel': (xv, yv)...}
        :param state:
        :param state_conf:
        :return: {'rel_x': x, 'rel_y': y, 'rel_xv': xv, 'rel_yv': yv, 'state': state, 'state_conf': state_conf}
        """
        ret = {'rel_x': [pos[0] for pos in data['pos']],
               'rel_y': [pos[1] for pos in data['pos']],
               'rel_xv': [pos[0] for pos in data['vel']],
               'rel_yv': [pos[1] for pos in data['vel']],
               'state': state,
               'state_conf': state_conf}
        return ret

    def _wrap_mock(self, data):
        data['state'] = Predictor.unknown
        return data
    
    def update(self, raw_perc, ts):
        """
        :param ts: timestamp of raw_perc
        :param raw_perc: {traj_id: [x, y, z, xv, yv, zv, xl, yl, zl, confidence, dist_class, move_class]}
        :return: {traj_id: {rel_x:, rel_y:, rel_xv:, rel_yv:, state:}}
        """
        self._update(raw_perc, ts)
        processed_perc = defaultdict(dict)
        for traj_id in raw_perc:
            state = [0] * 7
            perc = {key: value[-self.frame_to_pred:] for key, value in self._car_perc[traj_id].items()}
            state_conf = np.ones(7)
            if self._is_unknown(traj_id):
                state[-2] = 1
            else:
                perc, state, state_conf = self._predict(traj_id)
                is_reckless, reckless_conf = self._is_reckless(traj_id)
                state[-1] = is_reckless
                state_conf[-1] *= reckless_conf
            # processed_perc[traj_id] = self._wrap_ret_perc(perc, state, state_conf)
            processed_perc[traj_id] = self._wrap_mock(perc)
        return processed_perc

