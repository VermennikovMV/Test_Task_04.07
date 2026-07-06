import os
os.environ['MAVLINK20'] = '1'   # covariance и reset_counter - расширенные поля MAVLink2

import threading

from vio_bridge import Bridge
from vio_correction import SnapCorrection
from vio_data import VioModel

# ------------------------------ Конфиг ------------------------------
CONFIG = {
    'conn': 'tcp:127.0.0.1:5760',    
                                     
    'rate_hz': 30.0,                 

    # модель дрейфа (vio_data.py)
    'sigma_a': 0.25,   # м/с2   остаток ошибки акселерометра -> дрейф позиции
    'sigma_w': 0.25,   # град/с остаток ошибки гироскопа -> дрейф курса
    'sigma_n': 0.03,   # м      дрожь измерений камеры (не копится)
    'seed': 42,        

    'markers': {'A': (150.0, 200.0)},

    'origin_alt': 150.0,             
}


def main():
    cfg = CONFIG
    model = VioModel(dt=1.0 / cfg['rate_hz'], sigma_a=cfg['sigma_a'],
                     sigma_w=cfg['sigma_w'], sigma_n=cfg['sigma_n'], seed=cfg['seed'])
    correction = SnapCorrection(markers=cfg['markers'])
    bridge = Bridge(conn=cfg['conn'], rate_hz=cfg['rate_hz'], origin_alt=cfg['origin_alt'],
                    model=model, correction=correction)

    bridge.connect()
    bridge.setup_origin()

    threading.Thread(target=bridge.run, daemon=True).start()
    print('\nПоток VIO запущен. Mission Planner: TCP 127.0.0.1:5762')
    print('Команды: snap A | loss <сек> | status\n')

    while True:
        try:
            cmd = input().strip().split()
        except (EOFError, KeyboardInterrupt):
            print('Выход')
            return
        if not cmd:
            continue
        if cmd[0] == 'snap' and len(cmd) > 1:
            print(bridge.snap(cmd[1]))
        elif cmd[0] == 'loss' and len(cmd) > 1:
            print(bridge.loss(float(cmd[1])))
        elif cmd[0] == 'status':
            print(bridge.status())
        else:
            print('Команды: snap A | loss <сек> | status')


main()
