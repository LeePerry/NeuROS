# Copyright (c) 2024 Lee Perry

import nest
import numpy as np

nest.set_verbosity("M_ERROR")
nest.local_num_threads = 8

def ring_mean_activity(data, centre = True):
    # Data is expected to be a single ring's activity history of shape (timesteps, ring_size)
    # Centre == False: rays are from 0 -> 2pi, half-open interval. Centre == True: rays are adjusted to project from halfway along their arc
    data = np.array(data)
    if data.ndim < 2:
        data = data.reshape(1, -1)

    ring_size = data.shape[1]
    arc_per_ring_segment = (2 * np.pi) / ring_size
    rays = np.repeat(np.arange(0, 2 * np.pi, arc_per_ring_segment).reshape(1, -1), data.shape[0], axis = 0)
    if centre:
        rays = rays + arc_per_ring_segment / 2

    rays_for_each_spike = np.empty(shape = (data.shape[0]), dtype = 'object')
    mean_activity = np.empty(shape = (data.shape[0]), dtype = 'float')
    for i in range(rays_for_each_spike.shape[0]):
        if len(np.nonzero(data[i,:])) > 0:
            rays_for_each_spike[i] = np.repeat(rays[i,:][data[i,:] > 0], data[i, :][data[i,:] > 0].astype('int'))
            mean_activity[i] = np.arctan2(np.mean(np.sin(rays_for_each_spike[i])), np.mean(np.cos(rays_for_each_spike[i]))) % (2 * np.pi)
        else:
            mean_activity[i] = 0
        mean_activity[np.isnan(mean_activity)] = 0
    mean_activity_ring_index = mean_activity * (ring_size / (2*np.pi))
    return mean_activity_ring_index

class Model:

    def __init__(self, ring_model = 'HD'):

        if ring_model == 'HD':

            self._ring_model = 'HD'
        
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
            base_in = 450 * 1.8 #
            base_cj = 169 * 1 #
            w_ex_cj = 660 * 1 #
    
            I_init = 300.0 # pA
            I_init_dur = 100.0 # ms
            I_init_pos = N//2
    
            #----------------------------------
            # Create populations of cells
            #----------------------------------
            
            nest.CopyModel("iaf_psc_alpha", "hd_cell", cell_params)
            self._brain = nest.Create('hd_cell', N*4)
            nest.SetStatus(self._brain[:N], {"I_e": 385.0})
    
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
    
            self._correction_layer = nest.Create('step_current_generator', N)
            nest.Connect(self._correction_layer, self._brain[0:N], 'one_to_one')
    
            bump_init = nest.Create('step_current_generator', 1, params = {
                'amplitude_times'  : [0.1, 0.1+I_init_dur],
                'amplitude_values' : [I_init, 0.0]})
            nest.Connect(bump_init,[self._brain[I_init_pos]])
    
            SYN = {'weight': w_ex, 'delay': delay}
            nest.Connect(self._brain[0:N],self._brain[N:N*2], 'all_to_all', SYN)
    
            SYN = {'weight': -w_in, 'delay': delay}
            nest.Connect(self._brain[N:N*2],self._brain[0:N], 'all_to_all', SYN)
    
            SYN = {'weight': w_ex_cj, 'delay': delay}
            nest.Connect(self._brain[0:N],self._brain[N*2:N*3], 'one_to_one', SYN)
            nest.Connect(self._brain[0:N],self._brain[N*3:N*4], 'one_to_one', SYN)
    
            SYN = {'weight': w_l, 'delay': delay}
            nest.Connect(self._brain[N*2:N*3],self._brain[0:N], 'all_to_all', SYN)
    
            SYN = {'weight': w_r, 'delay': delay}
            nest.Connect(self._brain[N*3:N*4],self._brain[0:N], 'all_to_all', SYN)
    
            self._detector = nest.Create("spike_detector", 1, params={
                "withgid"  : True,
                "withtime" : True })
            nest.Connect(self._brain[:N], self._detector)
    
            self._left_input = nest.Create('step_current_generator', 1)
            nest.Connect(self._left_input, self._brain[2*N:3*N], 'all_to_all')
    
            self._right_input = nest.Create('step_current_generator', 1)
            nest.Connect(self._right_input, self._brain[3*N:4*N], 'all_to_all')

        elif ring_model == 'Grid':

            self._ring_model = 'Grid'

            N = 120
            
            N_ex = N
            N_in = N_ex
            N_cj = N_ex

            tension = True

            sigma = 0.1
            in_sigma = 0.1
            in_cj_sigma = 0.09375
            mu = 0.5
            prune_smaller_than = 10
        
            smooth_sigma = 10

            base_ex = 1750
            base_in = 0
            base_cj = 500
            w_ex_cj = 0
            w_in_cj = 3000
            w_ex_pa = 100
            w_in_pa = 1000

            cj_in_offset = 0 # Is the inhibitory 'bowl' biased towards the direction of input? If so, by how many cells?

            conjunctive_mode = 'negative'
            
            # Synaptic transmission delay (I believe this includes the synapse proper and action potential travel time)

            delay = 0.1
        
            # Velocity scaling parameters
        
            # I_vel: multiply incoming velocity by this amount to get the input current representing the vestibular signal
            # velocity_threshold: Are very small values for velocity set to zero?
            # miniumum_velocity: What is the minimum non-zero velocity? (only works if velocity_threshold is True)
        
            I_vel = 2000000#800 # Seems to work best if you can get the velocities in a 0-4000 range
        
            # Are conjuctive cells synapsed onto by excitatory layer or inhibitory layer.
        
            # 'positive': scalar excitatory weight, conjunctive weights must be tuned to act as a coincidence detector for input velocity and bump activity
            # 'negative': Gaussian inhibitory weight, suppresses incoming velocity input too far from the attractor bump
            # 'both': inhibitory 'bowl' as per 'negative' and self-reinforcing excitatory connections

            # Intrinsic excitation of the excitatory ring, constant current in picoamps

            intrinsic_excitation = 0.#225.
        
            # Initial (bump-forming) current injection parameters. This is a short spike of input to form the initial attractor state, to be adjusted by conjunctive input
        
            # I_init: strength of input current in picoamps
            # I_init_dur: how long this is applied for, in milliseconds
            # I_init_pos: where is this applied, in ring index (1-120). NEST, for better or worse, has neuron IDs starting at 1
        
            I_init = 450.0#300.0
            I_init_dur = 100.0
            I_init_pos = 60 - 1#(N_ex - 1)
    
            ## Create neuron populations from the above parameters

            self._brain = nest.Create('iaf_psc_alpha', N*4)
            
            # Lists to store each ring's population
        
            exc = self._brain[:N]
            inh = self._brain[N:2*N]
            l = self._brain[2*N:3*N]
            r = self._brain[3*N:4*N]

            ## Define connection weight matrices

            # Empty matrices
        
            w_ex = np.empty((N_in,N_ex))
            w_in = np.empty((N_ex,N_in))
        
            for e in range(N_ex):
                for i in range(N_in):
                    # Find minimum (true) distance between adjacent cells
                    d1 = abs(e/N_ex - i/N_in)
                    d2 = abs(e/N_ex - i/N_in -1)
                    d3 = abs(e/N_ex - i/N_in +1)
                    d = min(abs(d1),abs(d2),abs(d3))
                    # Create gaussian value based on parameters above to define connection strengths
                    w_gauss = np.exp(-(d - mu)**2/2/sigma**2) # Exitatory -> inhibitory
                    w_ring = np.exp(-(d)**2/2/in_sigma**2) # Inhibitory -> excitatory
                    # Assign appropriate weight values to matrices
                    w_ex[i,e] = base_ex * w_gauss
                    w_in[e,i] = base_in * w_ring 
        
            # Very small weights become 0
        
            w_ex[w_ex<prune_smaller_than] = 0
            w_in[w_in<prune_smaller_than] = 0
        
            # Plot weight matrix interactions as a sanity check. Should be an 'arch' of inhibition, leaving the suppressing areas far from the injection site
        
            intrinsic_input = np.tile(450., N_ex)
        
            injection_site = I_init_pos
        
            # As before, connection weight matrices, this time between conjunctive layers and the excitatory layer
        
            w_l = np.empty((N_ex,N_cj))
            w_r = np.empty((N_ex,N_cj))
        
            for c in range(N_cj):  
                for e in range(N_ex):
                    # Minimum distance, this time between each conjunctive cell and the excitatory cell displaced 1 away (e +/- 1)
                    # Left is anticlockwise, therefore drives the cell immediately to the left
                    # Right is clockwise, therefore drives the cell immediately to the right
                    d1 = abs((e-1)/N_cj - c/N_ex)
                    d2 = abs((e-1)/N_cj - c/N_ex -1)
                    d3 = abs((e-1)/N_cj - c/N_ex +1)
                    d = min(abs(d1),abs(d2),abs(d3))
                    w_l[e,c] = base_cj * (np.exp(-(d)**2/2/sigma**2))
        
                    d1 = abs((e+1)/N_cj - c/N_ex)
                    d2 = abs((e+1)/N_cj - c/N_ex -1)
                    d3 = abs((e+1)/N_cj - c/N_ex +1)
                    d = min(abs(d1),abs(d2),abs(d3))
                    w_r[e,c] = base_cj * (np.exp(-(d)**2/2/sigma**2))
        
            # Set all not the max to zero; makes sure the conjunctive cells only drive the immediate neighbour
            # Still uses the Gaussian connection weight, just doesn't use the whole Gaussian (for now)
        
            m = np.amax(w_l)
            w_l[w_l<m] = 0
            m = np.amax(w_r)
            w_r[w_r<m] = 0
        
            # Gaussian weight matrix for inhibitory->left conjunctive cells (if conjuctive_mode == 'negative')
        
            w_in_l_cj_gauss = np.empty((N_cj,N_in))
        
            for i in range(N_in):
                for c in range(N_cj):  
                    # Minimum distance, this time between each conjunctive cell and the excitatory cell displaced 1 away (e +/- 1)
                    # Left is anticlockwise, therefore drives the cell immediately to the left
                    # Right is clockwise, therefore drives the cell immediately to the right
                    d1 = abs((c-cj_in_offset)/N_cj - i/N_in)
                    d2 = abs((c-cj_in_offset)/N_cj - i/N_in -1)
                    d3 = abs((c-cj_in_offset)/N_cj - i/N_in +1)
                    d = min(abs(d1),abs(d2),abs(d3))
                    w_in_l_cj_gauss[c,i] = w_in_cj * (np.exp(-(d)**2/2/in_cj_sigma**2))
        
            w_in_l_cj_gauss = w_in_l_cj_gauss# - np.max(w_in_cj_gauss)
        
            # Very small weights become 0
        
            w_in_l_cj_gauss[w_in_l_cj_gauss<prune_smaller_than] = 0
            w_in_l_cj_gauss[w_in_l_cj_gauss<prune_smaller_than] = 0
        
            # Gaussian weight matrix for inhibitory->right conjunctive cells (if conjuctive_mode == 'negative')
        
            w_in_r_cj_gauss = np.empty((N_cj,N_in))
        
            for i in range(N_in):
                for c in range(N_cj):  
                    # Minimum distance, this time between each conjunctive cell and the excitatory cell displaced 1 away (e +/- 1)
                    # Left is anticlockwise, therefore drives the cell immediately to the left
                    # Right is clockwise, therefore drives the cell immediately to the right
                    d1 = abs((c+cj_in_offset)/N_cj - i/N_in)
                    d2 = abs((c+cj_in_offset)/N_cj - i/N_in -1)
                    d3 = abs((c+cj_in_offset)/N_cj - i/N_in +1)
                    d = min(abs(d1),abs(d2),abs(d3))
                    w_in_r_cj_gauss[c,i] = w_in_cj * (np.exp(-(d)**2/2/in_cj_sigma**2))
        
            w_in_r_cj_gauss = w_in_r_cj_gauss# - np.max(w_in_cj_gauss)
        
            # Very small weights become 0
        
            w_in_r_cj_gauss[w_in_r_cj_gauss<prune_smaller_than] = 0
            w_in_r_cj_gauss[w_in_r_cj_gauss<prune_smaller_than] = 0
        
            ## Wire everything up
    
            # Excitatory and inhibitory set to connect all to all, using the prior calculated weight matrix
    
            exc_2_inh = nest.Connect(exc,inh,'all_to_all',syn_spec={'weight': w_ex, 'delay': delay})
            inh_2_exc = nest.Connect(inh,exc,'all_to_all',syn_spec={'weight': -w_in, 'delay': delay})
    
            # Conjunctive layers connecting to the excitatory layer, with weights
    
            l_2_exc = nest.Connect(l,exc,'all_to_all',syn_spec={'weight': w_l, 'delay': delay})
            r_2_exc = nest.Connect(r,exc,'all_to_all',syn_spec={'weight': w_r, 'delay': delay})
    
            if conjunctive_mode == 'positive':
    
                # Excitatory connecting one-to-one to both conjunctive layers, with fixed weight.  A 'coincidence detector'.
    
                exc_2_l = nest.Connect(exc,l,'one_to_one',syn_spec={'weight': w_ex_cj, 'delay': delay})
                exc_2_r = nest.Connect(exc,r,'one_to_one',syn_spec={'weight': w_ex_cj, 'delay': delay})
    
            elif conjunctive_mode == 'negative':
    
                # Inhibitory connecting one-all_to_all-one to both conjunctive layers, with inverse Gaussian weights
    
                inh_2_l = nest.Connect(inh,l,'all_to_all',syn_spec={'weight': -w_in_l_cj_gauss, 'delay': delay})
                inh_2_r = nest.Connect(inh,r,'all_to_all',syn_spec={'weight': -w_in_r_cj_gauss, 'delay': delay})
    
            elif conjunctive_mode == 'both':
    
                exc_2_l = nest.Connect(exc,l,'one_to_one',syn_spec={'weight': w_ex_cj, 'delay': delay})
                exc_2_r = nest.Connect(exc,r,'one_to_one',syn_spec={'weight': w_ex_cj, 'delay': delay})
    
                inh_2_l = nest.Connect(inh,l,'all_to_all',syn_spec={'weight': -w_in_l_cj_gauss, 'delay': delay})
                inh_2_r = nest.Connect(inh,r,'all_to_all',syn_spec={'weight': -w_in_r_cj_gauss, 'delay': delay})

            self._detector = nest.Create("spike_detector", 1, params={
                "withgid"  : True,
                "withtime" : True })
            nest.Connect(exc, self._detector)
            
            self._left_input = nest.Create('step_current_generator', 1)
            nest.Connect(self._left_input, l, 'all_to_all')
    
            self._right_input = nest.Create('step_current_generator', 1)
            nest.Connect(self._right_input, r, 'all_to_all')

            bump_init = nest.Create('step_current_generator', 1, params = {'amplitude_times':[0.1,0.1+I_init_dur],'amplitude_values':[I_init,0.0]})
            nest.Connect(bump_init,[exc[I_init_pos]])
        
        self.resolution = N
        self.current_exc_state = np.zeros(shape = (N))
        self._time = 20.0

    def estimate(self, imu):

        td = 20.0
        time_range = [self._time, self._time + td]
        self._time += td

        yaw = imu.angular_velocity.z
        vel = yaw * 23.5 # tuned

        vel_range = [vel, 0.0]
        zeros = [0.0, 0.0]

        if self._ring_model == 'HD':
        
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

        elif self._ring_model == 'Grid':

            minimum_input = 2700 * 1.1

            positive_component = vel
            negative_component = vel
        
            positive_component = 0 if vel < 0 else vel
            negative_component = 0 if vel > 0 else vel
        
            Y_input_l = vel * positive_component
            Y_input_r = vel * negative_component
        
            Y_input_r = -Y_input_r
        
            Y_input_l_compliment = vel - Y_input_l
            Y_input_r_compliment = vel - Y_input_r
        
            Y_input_l = vel + Y_input_l + Y_input_r_compliment + minimum_input
            Y_input_r = vel + Y_input_r + Y_input_l_compliment + minimum_input

            nest.SetStatus(self._left_input,  { "amplitude_times"  : time_range,
                                                    "amplitude_values" : [Y_input_l, 0.0]})
            nest.SetStatus(self._right_input, { "amplitude_times"  : time_range,
                                                    "amplitude_values" : [Y_input_r, 0.0]})

        # run simulation
        nest.Prepare()
        nest.Run(td)

        ring1_exc, ring1_spikes_exc = np.unique(nest.GetStatus(self._detector)[0]['events']['senders'], return_counts = True)
        
        if ring1_spikes_exc.size > 0:
            self.current_exc_state = np.zeros(shape = (self.resolution))
            self.current_exc_state[ring1_exc - min(self._brain[0:self.resolution])] = ring1_spikes_exc
        
        #ring1_most_active_exc_index = np.argmax(self.current_exc_state) if np.argmax(self.current_exc_state) is not None else None
        ring1_most_active_exc_index = ring_mean_activity(self.current_exc_state)
        
        nest.Cleanup()
        nest.SetStatus(self._detector, {"n_events": 0})

        # New way to calculate maximum ring activity AKA the neuronal network's guess of what angle we are at
        return ring1_most_active_exc_index
        

    def apply_correction(self, odom, stamp):
        """
        odom = odom * (1 / odom.max())
        odom *= 8
        expected = len(self._correction_layer)
        actual = len(odom)
        if expected != actual:
            raise Exception("Correction is unexpected length. " + 
                f"Expected: {expected}. Actual: {actual}.")

        for i in range(expected):
            #nest.SetStatus([self._correction_layer], { "amplitude" : odom[i] })
            nest.SetStatus([self._correction_layer], { "amplitude_times"  : [self._time, self._time + 200],
                                                       "amplitude_values" : [odom[i], 0.0]})
        """ 
        pass