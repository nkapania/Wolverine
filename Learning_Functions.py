#Nathan Spielberg
#7.2.18

import scipy.io as sio
import numpy as np
from tqdm import tqdm
import tensorflow as tf
from scipy.signal import butter, lfilter, freqz

def butter_lowpass(cutoff, fs, order=2):
    nyq = 0.5 * fs
    normal_cutoff = cutoff/nyq
    b, a = butter(order, normal_cutoff)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=2):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b,a,data)
    return y

def euler_int(x, xdot, dt):
    return x + dt*xdot

# tire model
def fiala(alpha, Ca, mu, fz):
    alpha_slide = np.abs( np.arctan(3*mu*fz/Ca) )
    if np.abs(alpha) < alpha_slide:
        fy = ( -Ca*np.arctan(alpha) + ((Ca**2)/(3*mu*fz))*(np.abs(np.arctan(alpha)))*np.arctan(alpha) -
            ((Ca**3)/(9*(mu**2)*(fz**2)))*(np.arctan(alpha)**3)*(1-2*mu/(3*mu)) )
    else:
        fy = -mu*fz*np.sign(alpha)
    return fy

# tire model for tensorflow model

def fiala_tf(alpha, Ca, mu, fz):
    alpha_slide = tf.abs( tf.atan(3*mu*fz/Ca) )
    fy = tf.where( tf.abs(alpha) < alpha_slide,
            ( -Ca*tf.tan(alpha) + ((Ca**2)/(3*mu*fz))*(tf.abs(tf.tan(alpha)))*tf.tan(alpha) -
                ((Ca**3)/(9*(mu**2)*(fz**2)))*(tf.tan(alpha)**3)*(1-2*mu/(3*mu)) ),
            -mu*fz*tf.sign(alpha) )
    return fy


def sample(Param, Veh, mu):
    Ux_lim  = Param["Ux_lim"] 
    a       = Veh["a"]
    b       = Veh["b"]
    m       = Veh["m"]
    Cr      = Veh["Cr"]
    del_lim = Veh["del_lim"] 

    #First sample Ux to ensure dynamically feasible uy and r! This was a mistake before.
    Ux      = np.random.uniform(low = 1.0     , high = Ux_lim)

    r_lim   = (mu*9.81)/Ux
    Uy_lim  = (3*Ux*m*9.81*mu)/Cr * (a/(a+b)) + b*r_lim

    r       = np.random.uniform(low = -r_lim  , high = r_lim  )
    Uy      = np.random.uniform(low = -Uy_lim , high = Uy_lim )
    delta   = np.random.uniform(low = -del_lim, high = del_lim)

    return r, Uy, delta, Ux

def sample_Fx(Veh, ux):
    mu      = Veh["mu"]
    m       = Veh["m"]
    p_lim   = Veh["p_lim"]
    b_bias  = Veh["b_bias"]

    #This seems to give way!!! Too high values!
    #print(p_lim/ux)
    #print(-(mu*m*9.81))

    #Fx = np.random.uniform(low = -(mu*m*9.81) , high = p_lim/ux)
    Fx = np.random.uniform(low = -(19000) , high = 8640)
    #print(Fx)

    #Could try modifying above

    if Fx > 0:
        Fxf = Fx
        Fxr = 0
    else:
        Fxf = b_bias*Fx
        Fxr = (1-b_bias)*Fx
    return Fxf, Fxr

def sample_delta_traj(DT, sim_time, del_lim):
    N = int(sim_time/DT)
    del_sim = np.random.uniform(low = -del_lim, high = -del_lim, size = N)

    return del_sim

def sample_Ux_traj(DT, sim_time, Ux_lim):
    N = int(sim_time/DT)
    Ux_sim = np.random.uniform(low = 1.0 , high = Ux_lim,  size = N)

    return Ux_sim

def simulate_nn(DT, sim_time, x_0, e_0, psi_0, prediction_nn, Ux, k_lk, x_la, T, del_sim, Ux_sim):
    N = int(sim_time/DT)

    t_sim      = np.linspace(0, sim_time, N)
    e_sim      = np.zeros(shape= (N,1) )
    psi_sim    = np.zeros(shape= (N,1) )
    Uy_sim     = np.zeros(shape= (N,1) )
    r_sim      = np.zeros(shape= (N,1) )
    #del_sim   = np.zeros(shape= (N,1) )

    e_sim[0]   = e_0
    psi_sim[0] = psi_0

    #Sill maybe some bug or artifact here bc this shouldnt matter
    #Shouldnt see a kink in yaw rate
    Uy_sim[0]  = x_0[0, -2]
    r_sim[0]   = x_0[0, -3]

    dummy = np.zeros( shape=(1,2) )

    #REMEMBER have to run stuff in a session here!
    for i in range(N-1):

        #Calculate the control with LK controller
        #del_sim[i] = -k_lk * ( e_sim[i] + x_la*np.sin( psi_sim[i] ) )

        input_sim  = np.zeros( shape=(1, (3*T) ) ) 
        input_sim[0, (3*T - 3)] = r_sim[i]
        input_sim[0, (3*T - 2)] = Uy_sim[i]
        input_sim[0, (3*T - 1)] = del_sim[i]

        if i==0:
            input_sim[i,:] = x_0


        #STILL NEEED TO FEED IN full thing! need 12 inputs etc. just fill others with crap.

        #init the data
        sess.run(iter.initializer, feed_dict={ x: input_sim, y: dummy ,batch_size: 1})

        #forward prop
        pred         = sess.run(prediction_nn)

        #pull out and update states with network model
        r_sim[i+1]   = pred[0][0] #Note no integration need
        Uy_sim[i+1]  = pred[0][1]

        #Update position states
        e_sim[i+1]   = e_sim[i]   + DT*(Uy_sim[i]*np.cos(psi_sim[i]) + Ux*np.sin(psi_sim[i]))
        psi_sim[i+1] = psi_sim[i] + DT*(r_sim[i])

    #return sim results
    return t_sim, r_sim, Uy_sim, e_sim, psi_sim

def simulate_rnn(DT, sim_time, x_0, e_0, psi_0, prediction_rnn, Ux, k_lk, x_la, T, del_sim, Ux_sim):
    N = int(sim_time/DT)

    t_sim      = np.linspace(0, sim_time, N)
    e_sim      = np.zeros(shape= (N,1) )
    psi_sim    = np.zeros(shape= (N,1) )
    Uy_sim     = np.zeros(shape= (N,1) )
    r_sim      = np.zeros(shape= (N,1) )
    #del_sim   = np.zeros(shape= (N,1) )

    e_sim[0]   = e_0
    psi_sim[0] = psi_0

    #Sill maybe some bug or artifact here bc this shouldnt matter
    #Shouldnt see a kink in yaw rate
    Uy_sim[0]  = x_0[0, -2]
    r_sim[0]   = x_0[0, -3]

    dummy = np.zeros( shape=(1,2) )
    input_sim  = np.zeros( shape=(1, (3*T) ) ) 

    #REMEMBER have to run stuff in a session here!
    for i in range(N-1):

        #Calculate the control with LK controller
        #del_sim[i] = -k_lk * ( e_sim[i] + x_la*np.sin( psi_sim[i] ) )
        input_sim[0, 0:(3*T - 3)] = input_sim[0, 3:] 
        input_sim[0, (3*T - 3)]   = r_sim[i]
        input_sim[0, (3*T - 2)]   = Uy_sim[i]
        input_sim[0, (3*T - 1)]   = del_sim[i]

        if i==0:
            input_sim[i,:] = x_0


        #STILL NEEED TO FEED IN full thing! need 12 inputs etc. just fill others with crap.

        #init the data
        sess.run(iter.initializer, feed_dict={ x: input_sim, y: dummy ,batch_size: 1})

        #forward prop
        pred         = sess.run(prediction_rnn)

        #pull out and update states with network model
        r_sim[i+1]   = pred[0][0] #Note no integration need
        Uy_sim[i+1]  = pred[0][1]

        #Update position states
        e_sim[i+1]   = e_sim[i]   + DT*(Uy_sim[i]*np.cos(psi_sim[i]) + Ux*np.sin(psi_sim[i]))
        psi_sim[i+1] = psi_sim[i] + DT*(r_sim[i])

    #return sim results
    return t_sim, r_sim, Uy_sim, e_sim, psi_sim

def simulate_bike(DT, sim_time, x_0, e_0, psi_0, prediction_bike, Ux, k_lk, x_la, T, del_sim, Ux_sim):
    N = int(sim_time/DT)

    t_sim      = np.linspace(0, sim_time, N)
    e_sim      = np.zeros(shape= (N,1) )
    psi_sim    = np.zeros(shape= (N,1) )
    Uy_sim     = np.zeros(shape= (N,1) )
    r_sim      = np.zeros(shape= (N,1) )
    #del_sim      = np.zeros(shape= (N,1) )

    e_sim[0]   = e_0
    psi_sim[0] = psi_0
    Uy_sim[0]  = x_0[0, -2]
    r_sim[0]   = x_0[0, -3]

    dummy = np.zeros( shape=(1,2) )

    #REMEMBER have to run stuff in a session here!
    for i in range(N-1):

        #Calculate the control with LK controller
        #del_sim[i] = -k_lk * ( e_sim[i] + x_la*np.sin( psi_sim[i] ) )

        input_sim  = np.zeros( shape=(1, (3*T) ) ) 
        input_sim[0, (3*T - 3)] = r_sim[i]
        input_sim[0, (3*T - 2)] = Uy_sim[i]
        input_sim[0, (3*T - 1)] = del_sim[i]

        if i==0:
            input_sim[0,:]= x_0


        #STILL NEEED TO FEED IN full thing! need 12 inputs etc. just fill others with crap.

        #init the data
        sess.run(iter.initializer, feed_dict={ x: input_sim, y: dummy ,batch_size: 1})

        #forward prop
        pred         = sess.run(prediction_bike)

        #pull out and update states with network model
        r_sim[i+1]   = pred[0][0] #Note no integration need
        Uy_sim[i+1]  = pred[0][1]

        #Update position states
        e_sim[i+1]   = e_sim[i]   + DT*(Uy_sim[i]*np.cos(psi_sim[i]) + Ux*np.sin(psi_sim[i]))
        psi_sim[i+1] = psi_sim[i] + DT*(r_sim[i])

    #return sim results
    return t_sim, r_sim, Uy_sim, e_sim, psi_sim

def simulate_true(sim_time, x_0, e_0, psi_0, k_lk, x_la, T, mu, Ux, a, b, Cr, Cf, DT, m, Izz, del_sim, Ux_sim):
    N = int(sim_time/DT)

    t_sim      = np.linspace(0, sim_time, N)
    e_sim      = np.zeros(shape= (N,1) )
    psi_sim    = np.zeros(shape= (N,1) )
    Uy_sim     = np.zeros(shape= (N,1) )
    r_sim      = np.zeros(shape= (N,1) )
    #del_sim    = np.zeros(shape= (N,1) )

    e_sim[0]   = e_0
    psi_sim[0] = psi_0
    Uy_sim[0]  = x_0[0, -2]
    r_sim[0]   = x_0[0, -3]

    dummy = np.zeros( shape=(1,2) )

    #REMEMBER have to run stuff in a session here!
    for i in range(N-1):

        #Calculate the control with LK controller
        #del_sim[i] = -k_lk * ( e_sim[i] + x_la*np.sin( psi_sim[i] ) )

        #pull out and update states with network model

        r_sim[i+1], Uy_sim[i+1] =  step_dynamics(r_sim[i], Uy_sim[i], del_sim[i], mu, Ux, a, b, Cr, Cf, DT, m, Izz)

        #Update position states
        e_sim[i+1]   = e_sim[i]   + DT*(Uy_sim[i]*np.cos(psi_sim[i]) + Ux*np.sin(psi_sim[i]))
        psi_sim[i+1] = psi_sim[i] + DT*(r_sim[i])

    #return sim results
    return t_sim, r_sim, Uy_sim, e_sim, psi_sim


def step_dynamics(r_t, Uy_t, del_t, Ux_t, Fxf, Fxr, Param, Veh, mu):

    DT     = Param["DT"]

    a      = Veh["a"]
    b      = Veh["b"]  
    l      = Veh["l"]
    h      = Veh["h"]
    Cr     = Veh["Cr"]
    Cf     = Veh["Cf"]
    m      = Veh["m"]
    Izz    = Veh["Izz"]
    

    #Calculate slip angles:
    a_f    = np.arctan( (Uy_t + a*r_t) / Ux_t) - del_t
    a_r    = np.arctan( (Uy_t - b*r_t) / Ux_t)


    if Param["Weight_Transfer"]:
        #First with WT Calc ax:
        ax = (Fxf * np.cos(del_t) + Fxr)/m

        #Now Calc Normal loads which are now dep on ax
        Fzf   = (b/l)*m*9.81 - (h/l)*m*ax
        Fzr   = (a/l)*m*9.81 + (h/l)*m*ax
    else:
        Fzf   = (m*9.81*b)/(l)
        Fzr   = (m*9.81*a)/(l)

    #Calculate Forces
    Fyf    = fiala(a_f, Cf, mu, Fzf)
    Fyr    = fiala(a_r, Cr, mu, Fzr)

    #Calculate Derivatives
    Uy_dot = (Fyf * np.cos(del_t) + Fxf*np.sin(del_t) + Fyr)/m  - r_t*Ux_t
    r_dot  = (a*Fyf*np.cos(del_t) + a*Fxf*np.sin(del_t)  - b*Fyr) / Izz
    Ux_dot = (Fxf * np.cos(del_t) + Fxr)/m  + r_t*Uy_t

    Uy_t_1 = Uy_t + DT * Uy_dot
    r_t_1  = r_t  + DT * r_dot
    Ux_t_1 = Ux_t + DT * Ux_dot

    return r_t_1, Uy_t_1, Ux_t_1


def step_deriv_dynamics(r_t, Uy_t, del_t, mu, Ux_t, a, b, Cr, Cf, DT, m, Izz):
    #Calculate slip angles:
    a_f    = np.arctan( (Uy_t + a*r_t) / Ux_t) - del_t
    a_r    = np.arctan( (Uy_t - b*r_t) / Ux_t)

    #Calculate Forces
    Fyf    = fiala(a_f, Cf, mu, (m*9.81*b)/(a+b))
    Fyr    = fiala(a_r, Cr, mu, (m*9.81*a)/(a+b))

    #Calculate Derivatives
    Uy_dot = (Fyf * np.cos(del_t) + Fyr)/m  - r_t*Ux_t
    r_dot  = (a*Fyf*np.cos(del_t)  - b*Fyr) / Izz

    return r_dot, Uy_dot

def load_data(Param, Veh, filename):
    raw_data       = sio.loadmat(filename, squeeze_me=True)

    N_SAMPLES      = Param["N_SAMPLES"]
    N_STATE_INPUT  = Param["N_STATE_INPUT"]
    T              = Param["T"]
    DT             = Param["DT"]
    VEHICLE_NAME   = Param["VEHICLE_NAME"]
    CUTOFF         = Param["CUTOFF"]
    FS             = Param["FS"]
    m              = Veh["m"]


    if VEHICLE_NAME == "Niki": 
        #All bicycle model velocity states
        ux     = raw_data['OxTSData']['vxCG_mps'].sum()
        uy     = raw_data['OxTSData']['vyCG_mps'].sum()
        r      = raw_data['OxTSData']['yawRate_radps'].sum()
        exp_t  = raw_data['t']

        #now for other vehicle inputs
        fxf    = m*raw_data['OxTSData']['axCG_mps2'].sum()
        print(fxf.shape)
        ay     = raw_data['OxTSData']['ayCG_mps2'].sum()

        #Stupid naming conventions
        try:
            delta = raw_data['TTSdata']['roadWheelAngle_rad'].sum()
        except:
            delta = raw_data['vehicleData']['roadWheelAngle_rad'].sum()


        #Downsample for Niki (Oxts factor 10 slower than data recording)
        ux     = ux[0::10]
        uy     = uy[0::10]
        r      = r[0::10]
        delta  = delta[0::10]
        fxf    = fxf[0::10]
        print(fxf.shape)


    else:
        #All bicycle model velocity states
        ux     = raw_data['ux_mps']
        uy     = raw_data['uy_mps']
        r      = raw_data['r_radps']
        delta  = raw_data['delta_rad']
        fxf    = raw_data['fxf_N']
        exp_t  = raw_data['t']
        print("The experimental DT is: " + str(exp_t[1] - exp_t[0]) )

    #Get speed mask
    #for now assume no stopping during test, plot to be sure of continuity...
    #there is probably a smarter way to do this/ check for this.
    speed_mask = ux > 1.0
    ux     = ux[speed_mask]
    uy     = uy[speed_mask]
    r      = r[speed_mask]
    delta  = delta[speed_mask]
    fxf    = fxf[speed_mask]

    ux    = butter_lowpass_filter(ux, CUTOFF, FS)
    uy    = butter_lowpass_filter(uy, CUTOFF, FS)
    r     = butter_lowpass_filter(r, CUTOFF, FS)
    delta = butter_lowpass_filter(delta, CUTOFF, FS)
    fxf   = butter_lowpass_filter(fxf, CUTOFF, FS) #to have same phase lag.

    N_DATA = len(ux)
    print("Number of Data points in the recorded data is: " + str(N_DATA) )

    if N_SAMPLES < (N_DATA - T):

        gen_data = np.zeros( (N_SAMPLES, (N_STATE_INPUT*T+3) ) )
        N_DATA   = N_SAMPLES

        for i in range(N_DATA):
            #Order of state etc
            #r uy delta ux ... T ... T+1
            for j in range(T):
                gen_data[i, N_STATE_INPUT*j + 0] = r[i+j]
                gen_data[i, N_STATE_INPUT*j + 1] = uy[i+j]
                gen_data[i, N_STATE_INPUT*j + 2] = delta[i+j]
                gen_data[i, N_STATE_INPUT*j + 3] = ux[i+j]

            #Then do the last two for the targets.
            gen_data[i, N_STATE_INPUT*T]         = fxf[i+T-1] / m 
            gen_data[i, N_STATE_INPUT*T+1]       = r[i+T] 
            gen_data[i, N_STATE_INPUT*T+2]       = uy[i+T]

    else:

        #Now ax is -3 in input array
        gen_data = np.zeros( ( (N_DATA - T), (N_STATE_INPUT*T+3) ) ) 
        N_DATA   = (N_DATA - T)

        for i in range(N_DATA):
            #Order of state etc
            #r uy delta ux ... T ... T+1
            for j in range(T):
                gen_data[i, N_STATE_INPUT*j + 0] = r[i+j]
                gen_data[i, N_STATE_INPUT*j + 1] = uy[i+j]
                gen_data[i, N_STATE_INPUT*j + 2] = delta[i+j]
                gen_data[i, N_STATE_INPUT*j + 3] = ux[i+j]

            #Then do the last two for the targets.
            gen_data[i, N_STATE_INPUT*T]         = fxf[i+T-1] / m
            gen_data[i, N_STATE_INPUT*T+1]       = r[i+T] 
            gen_data[i, N_STATE_INPUT*T+2]       = uy[i+T]

        print("NUMBER OF SAMPLES LARGER THAN DATA")  

    return gen_data, N_DATA, exp_t

def generate_data(Param, Veh):

    #Unpack dicts
    N_SAMPLES     = Param["N_SAMPLES"]
    Ux_lim        = Param["Ux_lim"]
    T             = Param["T"]
    DT            = Param["DT"]
    N_SAMPLES     = Param["N_SAMPLES"]
    N_STATE_INPUT = Param["N_STATE_INPUT"]
    TWO_FRIC      = Param["TWO_FRIC"]

    m             = Veh["m"]
    mu            = Veh["mu"] 
    mu_2          = Veh["mu_2"]
    a             = Veh["a"]
    b             = Veh["b"]
    Cr            = Veh["Cr"]
    del_lim       = Veh["del_lim"]
    Izz           = Veh["Izz"]
    p_lim         = Veh["p_lim"]
    b_bias        = Veh["b_bias"]


    gen_data = np.zeros( (N_SAMPLES, (N_STATE_INPUT*T+3) ) )

    print("Starting Data Generation! ")
    if TWO_FRIC:
        for i in tqdm(range(N_SAMPLES)):
            if i <  N_SAMPLES/2.0:
                for j in range(T): 
                #FIll out the delay states!
                    if j==0:
                        gen_data[i, 0], gen_data[i, 1], gen_data[i, 2], gen_data[i, 3]                                       = sample(Param, Veh, mu)
                        Fxf, Fxr                                                                                             = sample_Fx(Veh, gen_data[i, 3])
                    else: 
                        gen_data[i, (N_STATE_INPUT*j)], gen_data[i, (N_STATE_INPUT*j)+1], gen_data[i, (N_STATE_INPUT*j) + 3] = step_dynamics(gen_data[i, N_STATE_INPUT*(j-1)], gen_data[i, N_STATE_INPUT*(j-1) + 1], gen_data[i, N_STATE_INPUT*(j-1)+2], gen_data[i, N_STATE_INPUT*(j-1)+3], Fxf, Fxr, Param, Veh, mu)
                        _,_, gen_data[i, (N_STATE_INPUT*j) + 2], _                                                           = sample(Param, Veh, mu)      
                        Fxf, Fxr                                                                                             = sample_Fx(Veh, gen_data[i, (N_STATE_INPUT*j) + 3])

                #gen_data[i, (N_STATE_INPUT*T)], gen_data[i, (N_STATE_INPUT*T)+1]                     = step_dynamics(gen_data[i, N_STATE_INPUT*(T-1)], gen_data[i, N_STATE_INPUT*(T-1) + 1], gen_data[i, N_STATE_INPUT*(T-1)+2], mu, gen_data[i, N_STATE_INPUT*(T-1)+3], a, b, Cr, Cf, DT, m, Izz)
                #gen_data[i, (N_STATE_INPUT*T)], gen_data[i, (N_STATE_INPUT*T)+1]                     = step_deriv_dynamics(gen_data[i, N_STATE_INPUT*(T-1)], gen_data[i, N_STATE_INPUT*(T-1) + 1], gen_data[i, N_STATE_INPUT*(T-1)+2], mu, gen_data[i, N_STATE_INPUT*(T-1)+3], a, b, Cr, Cf, DT, m, Izz)
            
                gen_data[i, (N_STATE_INPUT*T+1)], gen_data[i, (N_STATE_INPUT*T)+2], _                 = step_dynamics(gen_data[i, N_STATE_INPUT*(T-1)], gen_data[i, N_STATE_INPUT*(T-1) + 1], gen_data[i, N_STATE_INPUT*(T-1)+2], gen_data[i, N_STATE_INPUT*(T-1)+3], Fxf, Fxr, Param, Veh, mu)
                gen_data[i, (N_STATE_INPUT*T)]                                                        = Fxf/ (m*100.0)
            
            else:
                for j in range(T): 
                #FIll out the delay states!
                    if j==0:
                        gen_data[i, 0], gen_data[i, 1], gen_data[i, 2], gen_data[i, 3]                                       = sample(Param, Veh, mu_2)
                        Fxf, Fxr                                                                                             = sample_Fx(Veh, gen_data[i, 3])
                    else: 
                        gen_data[i, (N_STATE_INPUT*j)], gen_data[i, (N_STATE_INPUT*j)+1], gen_data[i, (N_STATE_INPUT*j) + 3] = step_dynamics(gen_data[i, N_STATE_INPUT*(j-1)], gen_data[i, N_STATE_INPUT*(j-1) + 1], gen_data[i, N_STATE_INPUT*(j-1)+2], gen_data[i, N_STATE_INPUT*(j-1)+3], Fxf, Fxr, Param, Veh, mu_2)
                        _,_, gen_data[i, (N_STATE_INPUT*j) + 2], _                                                           = sample(Param, Veh, mu_2)      
                        Fxf, Fxr                                                                                             = sample_Fx(Veh, gen_data[i, (N_STATE_INPUT*j) + 3])

                #gen_data[i, (N_STATE_INPUT*T)], gen_data[i, (N_STATE_INPUT*T)+1]                     = step_dynamics(gen_data[i, N_STATE_INPUT*(T-1)], gen_data[i, N_STATE_INPUT*(T-1) + 1], gen_data[i, N_STATE_INPUT*(T-1)+2], mu, gen_data[i, N_STATE_INPUT*(T-1)+3], a, b, Cr, Cf, DT, m, Izz)
                #gen_data[i, (N_STATE_INPUT*T)], gen_data[i, (N_STATE_INPUT*T)+1]                     = step_deriv_dynamics(gen_data[i, N_STATE_INPUT*(T-1)], gen_data[i, N_STATE_INPUT*(T-1) + 1], gen_data[i, N_STATE_INPUT*(T-1)+2], mu, gen_data[i, N_STATE_INPUT*(T-1)+3], a, b, Cr, Cf, DT, m, Izz)
            
                gen_data[i, (N_STATE_INPUT*T+1)], gen_data[i, (N_STATE_INPUT*T)+2], _                 = step_dynamics(gen_data[i, N_STATE_INPUT*(T-1)], gen_data[i, N_STATE_INPUT*(T-1) + 1], gen_data[i, N_STATE_INPUT*(T-1)+2], gen_data[i, N_STATE_INPUT*(T-1)+3], Fxf, Fxr, Param, Veh, mu_2)
                gen_data[i, (N_STATE_INPUT*T)]                                                        = Fxf/(m*100.0)                                               
    else:
        for i in tqdm(range(N_SAMPLES)):

            #This should be a function! ie generate one history!
            #run a single simulation for one trajectory of length T
            for j in range(T): 
                #FIll out the delay states!
                if j==0:
                    gen_data[i, 0], gen_data[i, 1], gen_data[i, 2], gen_data[i, 3]                                       = sample(Param, Veh, mu)
                    Fxf, Fxr                                                                                             = sample_Fx(Veh, gen_data[i, 3])
                else: 
                    gen_data[i, (N_STATE_INPUT*j)], gen_data[i, (N_STATE_INPUT*j)+1], gen_data[i, (N_STATE_INPUT*j) + 3] = step_dynamics(gen_data[i, N_STATE_INPUT*(j-1)], gen_data[i, N_STATE_INPUT*(j-1) + 1], gen_data[i, N_STATE_INPUT*(j-1)+2], gen_data[i, N_STATE_INPUT*(j-1)+3], Fxf, Fxr, Param, Veh, mu)
                    _,_, gen_data[i, (N_STATE_INPUT*j) + 2], _                                                           = sample(Param, Veh, mu)      
                    Fxf, Fxr                                                                                             = sample_Fx(Veh, gen_data[i, (N_STATE_INPUT*j) + 3])

            #gen_data[i, (N_STATE_INPUT*T)], gen_data[i, (N_STATE_INPUT*T)+1]                     = step_dynamics(gen_data[i, N_STATE_INPUT*(T-1)], gen_data[i, N_STATE_INPUT*(T-1) + 1], gen_data[i, N_STATE_INPUT*(T-1)+2], mu, gen_data[i, N_STATE_INPUT*(T-1)+3], a, b, Cr, Cf, DT, m, Izz)
            #gen_data[i, (N_STATE_INPUT*T)], gen_data[i, (N_STATE_INPUT*T)+1]                     = step_deriv_dynamics(gen_data[i, N_STATE_INPUT*(T-1)], gen_data[i, N_STATE_INPUT*(T-1) + 1], gen_data[i, N_STATE_INPUT*(T-1)+2], mu, gen_data[i, N_STATE_INPUT*(T-1)+3], a, b, Cr, Cf, DT, m, Izz)
            
            gen_data[i, (N_STATE_INPUT*T+1)], gen_data[i, (N_STATE_INPUT*T)+2], _                 = step_dynamics(gen_data[i, N_STATE_INPUT*(T-1)], gen_data[i, N_STATE_INPUT*(T-1) + 1], gen_data[i, N_STATE_INPUT*(T-1)+2], gen_data[i, N_STATE_INPUT*(T-1)+3], Fxf, Fxr, Param, Veh, mu)
            gen_data[i, (N_STATE_INPUT*T)]                                                        = Fxf/(m*100.0)
    
    return gen_data

def get_gen_tires(r, uy, delta, ux, Veh):

    Cf = Veh["Cf"]
    Cr = Veh["Cr"]
    mu = Veh["mu"]
    m  = Veh["m"]
    a  = Veh["a"]
    b  = Veh["b"]

    a_f    = np.arctan( (uy + a*r) / ux) - delta
    a_r    = np.arctan( (uy - b*r) / ux)

    #Calculate Forces
    Fyf    = fiala(a_f, Cf, mu, (m*9.81*b)/(a+b))
    Fyr    = fiala(a_r, Cr, mu, (m*9.81*a)/(a+b))

    return a_f, Fyf, a_r, Fyr

def plot_final_tires(Cf, Cr, mu):

    pass

def add_noise(gen_data, Param):

    N_STATE_INPUT   = Param["N_STATE_INPUT"]
    T               = Param["T"]

    data_noise       = np.zeros(shape = (gen_data.shape))
    N_Samples        = gen_data.shape[0]

    for i in range(T):
        data_noise[:, N_STATE_INPUT*i     ] = np.random.normal(0.0, .01, size = (N_Samples))
        data_noise[:, N_STATE_INPUT*i + 1 ] = np.random.normal(0.0, .01, size = (N_Samples))
        data_noise[:, N_STATE_INPUT*i + 2 ] = np.random.normal(0.0, .001, size = (N_Samples))
        data_noise[:, N_STATE_INPUT*i + 3 ] = np.random.normal(0.0, .001, size = (N_Samples))

    data_noise[:, N_STATE_INPUT*T     ] = np.random.normal(0.0, .001, size = (N_Samples))
    data_noise[:, N_STATE_INPUT*T + 1 ] = np.random.normal(0.0, .01,  size = (N_Samples))
    data_noise[:, N_STATE_INPUT*T + 2 ] = np.random.normal(0.0, .01,  size = (N_Samples))

    return (data_noise + gen_data)
