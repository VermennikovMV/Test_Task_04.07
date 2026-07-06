import contextlib
import io
import math
import threading
import time

import numpy as np
from pymavlink import mavutil


def lat_lon_to_ne(lat, lon, origin_lat, origin_lon):
    north = (lat - origin_lat) * 111320.0
    east = (lon - origin_lon) * 111320.0 * math.cos(math.radians(origin_lat))
    return north, east


class Bridge:
    def __init__(self, conn, rate_hz, origin_alt, model, correction):
        self.conn = conn
        self.dt = 1.0 / rate_hz
        self.origin_alt = origin_alt
        self.model = model
        self.correction = correction
        self.lock = threading.Lock()              # model+correction трогают два потока
        self.last_truth = None                    # последняя известная истина из SIMSTATE
        self.last_truth_time = time.time()        # когда она приходила в последний раз
        self.pause_until = 0.0                    # имитация потери трекинга (loss)
        self.mav = None
        self.origin_lat = None
        self.origin_lon = None

    # ---------- подключение и одноразовая настройка ----------

    def connect(self):
        print(f'Подключение к {self.conn} ...')
        t0 = time.time()
        last_note = 0.0
        while True:
            try:
                self.mav = mavutil.mavlink_connection(
                    self.conn, retries=0, source_system=1, source_component=197)
                with contextlib.redirect_stdout(io.StringIO()):  # без спама pymavlink
                    hb = self.mav.wait_heartbeat(timeout=5)
                if hb is not None:
                    break
                self.mav.close()
            except OSError:
                pass
            if time.time() - last_note > 15:
                last_note = time.time()
                print(f'  ... жду SITL ({int(time.time() - t0)} с) - '
                      f'запущен ли sitl_launch.sh в окне 1?')
            time.sleep(2)
        print(f'Heartbeat: sysid={self.mav.target_system}')

        # без запроса телеметрии SITL на этом порту молчит (и SIMSTATE не придёт)
        self.mav.mav.request_data_stream_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 30, 1)

    def setup_origin(self):
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 0, 0, 0, 0, 0, 0)
        msg = self.mav.recv_match(type='GPS_GLOBAL_ORIGIN', blocking=True, timeout=2.0)
        if msg is not None and (msg.latitude != 0 or msg.longitude != 0):
            self.origin_lat, self.origin_lon = msg.latitude * 1e-7, msg.longitude * 1e-7
            print(f'Origin уже установлен EKF: {self.origin_lat:.7f}, {self.origin_lon:.7f}')
        else:

            # EKF принимает origin только после инициализации фильтра (первые
            # секунды - откажет), поэтому шлём с повторами до эха-подтверждения.
            msg = self.mav.recv_match(type='SIMSTATE', blocking=True, timeout=5.0)
            if msg is None:
                raise RuntimeError('SITL не шлёт SIMSTATE')
            self.origin_lat, self.origin_lon = msg.lat * 1e-7, msg.lng * 1e-7
            print(f'Origin: {self.origin_lat:.7f}, {self.origin_lon:.7f}')
            for _ in range(30):
                self.mav.mav.set_gps_global_origin_send(
                    self.mav.target_system,
                    int(self.origin_lat * 1e7), int(self.origin_lon * 1e7),
                    int(self.origin_alt * 1000))
                if self.mav.recv_match(type='GPS_GLOBAL_ORIGIN',
                                       blocking=True, timeout=2.0) is not None:
                    print('Origin принят EKF')
                    break
            else:
                raise RuntimeError('EKF не принял origin за 60 с')

        for _ in range(15):
            self.mav.mav.command_int_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0, 0, 0, 0, 0, 0,
                int(self.origin_lat * 1e7), int(self.origin_lon * 1e7), self.origin_alt)
            t0 = time.time()
            while time.time() - t0 < 2.0:
                msg = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=1.0)
                if msg is not None and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_HOME:
                    if msg.result == 0:
                        print('HOME установлен в origin')
                        return
                    break
            time.sleep(1)
        raise RuntimeError('автопилот не принял HOME')

    # ---------- цикл отправки ----------

    def get_truth(self):
        msg = self.mav.recv_match(type='SIMSTATE', blocking=True, timeout=self.dt)
        if msg is None:
            return                                  # нового нет - живём на старом
        n, e = lat_lon_to_ne(msg.lat * 1e-7, msg.lng * 1e-7,
                             self.origin_lat, self.origin_lon)
        self.last_truth = (np.array([n, e, 0.0]), msg.roll, msg.pitch, msg.yaw)
        self.last_truth_time = time.time()

    def make_covariance(self):
        """Упаковка 6x6 в 21 число: диагональ на позициях 0,6,11,15,18,20."""
        cov = [0.0] * 21
        cov[0] = cov[6] = cov[11] = self.model.sigma_n ** 2   # дисперсии позиции, м2
        cov[15] = cov[18] = 0.01 ** 2       # крен/тангаж не дрейфуют - малый шум
        cov[20] = math.radians(3.0) ** 2    # заявленная неуверенность курса
        return cov

    def run(self):
        cov = self.make_covariance()
        next_send = time.time()
        last_vpe = 0.0
        while True:
            self.get_truth()                     # пришёл новый SIMSTATE - запомнит

            now = time.time()
            if now - self.last_truth_time > 5.0:   
                raise SystemExit('связь с SITL потеряна - выходим')
            if self.last_truth is None or now < next_send:
                continue
            next_send = now + self.dt
            if now < self.pause_until:           # имитация потери трекинга: молчим
                continue

            p_true, roll, pitch, yaw_true = self.last_truth
            with self.lock:
                p_vio, yaw_vio = self.model.tick(p_true, yaw_true)
                p_out = self.correction.apply(p_vio)
                rc = self.correction.reset_counter

            # крен/тангаж шлём истинные: у VIO они не дрейфуют (гравитация - якорь)
            self.mav.mav.vision_position_estimate_send(
                int(now * 1e6),                    # usec - время по часам "камеры"
                p_out[0], p_out[1], p_out[2],      # метры NED от origin
                roll, pitch, yaw_vio,              # радианы
                cov, rc)

    # ---------- команды оператора (зовутся из консоли) ----------

    def snap(self, name):
        with self.lock:
            delta, err = self.correction.snap(name)
        if err:
            return f'Отбивка отклонена: {err}'
        return (f'ОТБИВКА {name}: убран дрейф N={delta[0]:+.2f} м, E={delta[1]:+.2f} м '
                f'(reset_counter={self.correction.reset_counter})')

    def loss(self, seconds):
        self.pause_until = time.time() + seconds
        with self.lock:
            self.correction.bump()   # за паузу дрон улетит - скачок потока легален
        return f'Потеря трекинга на {seconds} с'

    def status(self):
        m, c = self.model, self.correction
        true_str = np.round(self.last_truth[0][:2], 1) if self.last_truth is not None else '---'
        sent_str = np.round(c.last_output[:2], 1) if c.last_output is not None else '---'
        return (f'ИСТИНА(NE)={true_str} м  отправлено={sent_str} м\n'
                f'v_err={m.v_err[:2].round(3)} м/с  p_err={m.p_err[:2].round(2)} м  '
                f'psi_err={math.degrees(m.psi_err):.2f} град  '
                f'offset={c.offset[:2].round(2)} м')
