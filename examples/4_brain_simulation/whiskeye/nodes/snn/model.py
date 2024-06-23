# Copyright (c) 2023 Lee Perry

import nest
import numpy as np
from scipy.stats import circmean

def timestamp_to_milliseconds(ts):
    return (ts.sec * 1_000) + (ts.nanosec / 1_000_000)

class Model:

    def __init__(self):
        cell_params = {
                'C_m': 250.0,
                'E_L': -70.0,
                'I_e': 0.0,
                't_ref': 2.0,
                'tau_Ca': 10000.0,
                'tau_m': 10.0,
                'tau_minus': 20.0,
                'tau_syn_ex': 2.0,
                'tau_syn_in': 2.0,
                'V_m': -70.0,
                'V_reset': -70.0,
                'V_th': -55.0,
                'V_min': -1e9}

        N = 180 # number of cells in each layer
        # layers = excitory, inhibitory, anti-clockwise, clockwise

        sigma = 0.12
        mu = 0.5
        delay = 0.1
        base_ex = 4000 #
        base_in = 450 #
        base_cj = 169 #
        w_ex_cj = 660 #

        I_init = 300.0 # pA
        I_init_dur = 100.0 # ms
        I_init_pos = N//2

        #----------------------------------
        # Create populations of cells
        #----------------------------------
        
        nest.CopyModel("iaf_psc_alpha", "hd_cell", cell_params)
        population = nest.Create('hd_cell', N*4)
        nest.SetStatus(population[:N], {"I_e": 450.0})

        #----------------------------------
        # Define connection weights
        #----------------------------------

        w_ex = np.empty((N,N))
        w_in = np.empty((N,N))
        for e in range(N):
            for i in range(N):
                d1 = abs(e/N - i/N)
                d2 = abs(e/N - i/N -1)
                d3 = abs(e/N - i/N +1)
                d = min(abs(d1),abs(d2),abs(d3))
                w_gauss = np.exp(-(d)**2/2/sigma**2)
                w_ring = np.exp(-(d - mu)**2/2/sigma**2)

                w_ex[i,e] = base_ex * w_gauss
                w_in[e,i] = base_in * w_ring

        w_ex[w_ex<10]=0
        w_in[w_in<10]=0

        w_l = np.empty((N,N))
        w_r = np.empty((N,N))
        for c in range(N):
            for e in range(N):
                d1 = abs((e-1)/N - c/N)
                d2 = abs((e-1)/N - c/N -1)
                d3 = abs((e-1)/N - c/N +1)
                d = min(abs(d1),abs(d2),abs(d3))
                w_l[e,c] = base_cj * (np.exp(-(d)**2/2/sigma**2))

                d1 = abs((e+1)/N - c/N)
                d2 = abs((e+1)/N - c/N -1)
                d23 = abs((e+1)/N - c/N +1)
                d = min(abs(d1),abs(d2),abs(d3))
                w_r[e,c] = base_cj * (np.exp(-(d)**2/2/sigma**2))

        m = np.amax(w_l)
        w_l[w_l<m] = 0
        m = np.amax(w_r)
        w_r[w_r<m] = 0

        #----------------------------------
        # Connect the network
        #----------------------------------

        CIRCUIT = population

        self._correction_layer = nest.Create('dc_generator', 180)
        nest.Connect(self._correction_layer, CIRCUIT[0:N], 'one_to_one')

        bump_init = nest.Create('step_current_generator', 1, params = {'amplitude_times':[0.1,0.1+I_init_dur],'amplitude_values':[I_init,0.0]})
        nest.Connect(bump_init,[CIRCUIT[I_init_pos]])

        SYN = {'weight': w_ex, 'delay': delay}
        nest.Connect(CIRCUIT[0:N],CIRCUIT[N:N*2], 'all_to_all', SYN)

        SYN = {'weight': -w_in, 'delay': delay}
        nest.Connect(CIRCUIT[N:N*2],CIRCUIT[0:N], 'all_to_all', SYN)

        SYN = {'weight': w_ex_cj, 'delay': delay}
        nest.Connect(CIRCUIT[0:N],CIRCUIT[N*2:N*3], 'one_to_one', SYN)
        nest.Connect(CIRCUIT[0:N],CIRCUIT[N*3:N*4], 'one_to_one', SYN)

        SYN = {'weight': w_l, 'delay': delay}
        nest.Connect(CIRCUIT[N*2:N*3],CIRCUIT[0:N], 'all_to_all', SYN)

        SYN = {'weight': w_r, 'delay': delay}
        nest.Connect(CIRCUIT[N*3:N*4],CIRCUIT[0:N], 'all_to_all', SYN)

        self._brain = CIRCUIT

        self._detector = nest.Create("spike_detector", N, params={
            "withgid"  : True,
            "withtime" : True })
        nest.Connect(CIRCUIT[:N], self._detector)

        self._left_input = nest.Create('step_current_generator', N)
        nest.Connect(self._left_input, CIRCUIT[2*N:3*N], 'one_to_one')

        self._right_input = nest.Create('step_current_generator', N)
        nest.Connect(self._right_input, CIRCUIT[3*N:4*N], 'one_to_one')

        #self._input = nest.Create('step_current_generator', N)
        #nest.Connect(self._input, CIRCUIT[:N], 'one_to_one')

        self._time = 0

    def estimate(self, imu):

        t = timestamp_to_milliseconds(imu.header.stamp)
        td = 20
        time_range = [t, t + td]

        yaw = imu.angular_velocity.z
        #vel = yaw * 0.35 * 10000 # TODO may need tuning
        vel = yaw * 1_000_000_000_000
        vel_range = [vel, 0.0]

        zeros = [0.0, 0.0]

        # check angular velocity
        anti_clockwise = vel < 0
        if anti_clockwise:
            nest.SetStatus(self._left_input,  { "amplitude_times"  : time_range,
                                                "amplitude_values" : vel_range })
            nest.SetStatus(self._right_input, { "amplitude_times"  : time_range,
                                                "amplitude_values" : zeros })
        else:
            nest.SetStatus(self._left_input,  { "amplitude_times"  : time_range,
                                                "amplitude_values" : zeros })
            nest.SetStatus(self._right_input, { "amplitude_times"  : time_range,
                                                "amplitude_values" : vel_range })

        nest.Prepare()
        nest.Run(td)
        data = nest.GetStatus(self._detector)[0]["events"]
        av = circmean(data["senders"], low=1, high=180)
        nest.Cleanup()
        # TODO check that correction layer retains data
        nest.SetStatus(self._detector, {"n_events": 0})
        
        if not np.isnan(av):
            return int(av)

    def apply_correction(self, odom, stamp):
        
        odom = odom * (1 / odom.max())
        expected = len(self._correction_layer)
        actual = len(odom)
        if expected != actual:
            raise Exception("Correction is unexpected length. " + 
                f"Expected: {expected}. Actual: {actual}.")

        #t = timestamp_to_milliseconds(stamp)
        for i in range(expected):
            nest.SetStatus([self._correction_layer[i]], { "amplitude" : odom[i] })
            #nest.SetStatus([self._correction_layer[i]], { "amplitude_times" : [t, t + 200],
            #                                              "amplitude_values": [odom[i], 0.0]})
