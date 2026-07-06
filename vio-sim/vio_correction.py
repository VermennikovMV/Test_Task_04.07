import numpy as np


class SnapCorrection:
    def __init__(self, markers):
        self.markers = markers        
        self.offset = np.zeros(3)     
        self.reset_counter = 0        
        self.last_output = None       

    def apply(self, p_vio):
        p = p_vio - self.offset
        self.last_output = p.copy()
        return p

    def bump(self):
        self.reset_counter = (self.reset_counter + 1) % 256

    def snap(self, name):
        if name not in self.markers:
            return None, f'нет маркера {name}'
        if self.last_output is None:
            return None, 'поток VIO ещё не начался'
        delta = self.last_output[:2] - np.array(self.markers[name])
        self.offset[0] += delta[0]
        self.offset[1] += delta[1]
        self.bump()
        return delta, None
