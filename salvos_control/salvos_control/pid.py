import numpy as np
 
class PID():

    def __init__(self, kp, ki, kd, tau, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.tau = tau
        self.out_min = out_min
        self.out_max = out_max

        self.target = None
        self.current = None
        self.debug = False
        self.temp = self.ki

        self.error = 0
        self.prev_error = 0
        self.int_error = 0
        self.der_error = 0
        self.saturation = False
        self.clamp = False
        self.out = 0

        self.lpf = FirstOrderLPF(self.tau)


    def compute(self, target, current, dt):

        self.target = target
        self.current = current

        self.error = self.target - self.current


        if dt > 0.0 and np.isfinite(self.error):
            if self.clamp:
                pass
                #self.ki = 0
            else:
                self.ki = self.temp
                self.int_error += self.error*dt


            self.der_error = (self.error - self.prev_error) / dt

        self.der_error = self.lpf.update(self.der_error, dt)

        self.prev_error = self.error

        raw_output = self.kp*self.error + self.ki*self.int_error + self.kd*self.der_error

        self.output = np.clip(raw_output, self.out_min, self.out_max)

        if self.output != raw_output:
            self.saturation = True

        if np.sign(self.error) == np.sign(self.output) and self.saturation:
            self.clamp = True
        else:
            self.clamp = False


        if self.debug:
            print(
                f'Error: {self.error}, Output: {self.output}, Setpoint: {self.target}, '
                f'Current: {self.current}, kp: {self.kp}, kd: {self.kd}, ki: {self.ki}'
            )

        return self.output
	


class FirstOrderLPF:
    def __init__(self, tau):
        self.tau = tau
        self.dt = 0
        self.y = 0.0
        self.alpha = 0.0

    def reset(self, x0=0.0):
        self.y = x0

    def update(self, u, dt):
        self.dt = dt
        self.alpha = self.dt / (self.tau + self.dt)

        if not (self.alpha == 0.0 or self.dt == 0.0):
            self.y += self.alpha * (u - self.y)

        #print('LPF log: ', self.alpha, self.y, self.dt, u)

        return self.y