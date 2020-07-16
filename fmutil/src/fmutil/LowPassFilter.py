class LowPassFilter:
    '''A class that implements a low pass filter. Jeong Hwan's implementation,
    with improvements.
    '''

    def __init__(self, tau):
        '''Constructor. Takes the filter's time constant as argument.'''
        if tau<=0:
            raise ValueError("Filter's time constant (tau) must be > 0")
        self.tau = tau
        self.reset()

    def reset(self):
        '''Reset the filter.'''
        self.y = None
        self.t = None

    def filter(self, t, x):
        '''Update the filter with a new value (and a timestamp).
        Arguments:
        t -- the timestamp of this sample
        x -- the sample value
        '''
        if self.y is None:
            self.y = x
            self.t = t
            return x

        dt = t - self.t
        self.t = t
        if dt > self.tau:
            self.y = x
            return x

        self.y += (x - self.y) * dt / self.tau
        return self.y

    def filter_dt(self, dt, x):
        '''Update the filter with a new value (and a time difference).
        Arguments:
        dt -- the time difference since last update
        x  -- the sample value
        '''
        if self.y is None:
            self.t = 0
            self.y = x
            return x

        return self.filter(self.t+dt, x)
