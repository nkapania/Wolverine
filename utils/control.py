import numpy as np
import tiremodels as tm
import vehicles
import paths
import math
import tensorflow as tf


#Controller from Nitin Kapania's PhD thesis - lookahead with augmented sideslip for
#steering feedback, longitudinal is simple PID feedback control
class LaneKeepingController():
    def __init__(self, path, vehicle, profile):
        self.path = path
        self.vehicle = vehicle
        self.profile = profile
        self.xLA = 14.2    #lookahead distance, meters
        self.kLK = 0.0538  #proportional gain , rad / meter
        self.kSpeed = 3000.0 #Speed proportional gain - N / (m/s)
        self.alphaFlim = 7.0 * np.pi / 180 #steering limits for feedforward controller
        self.alphaRlim = 5.0 * np.pi / 180 #steering limits for feedforward controller
        
        #Initialize force lookup tables for feedforward
        numTableValues = 250

        #values where car is sliding
        alphaFslide = np.abs(np.arctan(3*vehicle.muF*vehicle.m*vehicle.b/vehicle.L*vehicle.g/vehicle.Cf)) 
        alphaRslide = np.abs(np.arctan(3*vehicle.muR*vehicle.m*vehicle.a/vehicle.L*vehicle.g/vehicle.Cr))

        alphaFtable = np.linspace(-alphaFslide, alphaFslide, numTableValues)
        alphaRtable = np.linspace(-alphaRslide, alphaRslide, numTableValues) # vector of rear alpha (rad)
        
        FyFtable = tm.fiala(vehicle.Cf, vehicle.muF, vehicle.muF, alphaFtable, vehicle.FzF)
        FyRtable = tm.fiala(vehicle.Cr, vehicle.muR, vehicle.muR, alphaRtable, vehicle.FzR)

        #flip arrays so Fy is increasing - important for numpy interp!!
        self.alphaFtable = np.flip(alphaFtable, 0)
        self.alphaRtable = np.flip(alphaRtable, 0)
        self.FyFtable = np.flip(FyFtable, 0) 
        self.FyRtable = np.flip(FyRtable, 0)



    def getDeltaFB(self, localState, betaFFW):
        kLK = self.kLK
        xLA = self.xLA
        e = localState.e
        deltaPsi = localState.deltaPsi

        deltaFB = -kLK * (e + xLA * np.sin(deltaPsi + betaFFW))
        return deltaFB


    def speedTracking(self, localState):

        #note - interp requires rank 0 arrays
        AxTable = self.profile.Ax
        UxTable = self.profile.Ux
        sTable = self.profile.s
        m = self.vehicle.m
        fdrag = self.vehicle.dragCoeff
        frr = self.vehicle.rollResistance

        s = localState.s
        Ux = localState.Ux

        AxDes = np.interp(s, sTable, AxTable) #run interp every time - this is slow, but we may be able to get away with
        UxDes = np.interp(s, sTable, UxTable) #run interp every time - this is slow, but we may be able to get away with


        FxFFW = m*AxDes + np.sign(Ux)*fdrag*Ux ** 2 + frr*np.sign(Ux) # Feedforward
        FxFB = -self.kSpeed*(Ux - UxDes) # Feedback
        FxCommand = FxFFW + FxFB
        return FxCommand, UxDes, AxDes, FxFFW, FxFB


    def getDeltaFFW(self, localState, K):
        a = self.vehicle.a
        b = self.vehicle.b
        L = self.vehicle.L
        m = self.vehicle.m
        Ux = localState.Ux


        FyFdes = b / L * m * Ux**2 * K
        FyRdes = a / b * FyFdes

        alphaFdes = _force2alpha(self.FyFtable, self.alphaFtable, FyFdes)
        alphaRdes = _force2alpha(self.FyRtable, self.alphaRtable, FyRdes)

        betaFFW = alphaRdes + b * K 
        deltaFFW = K * L + alphaRdes - alphaFdes

        return deltaFFW, betaFFW, FyFdes, FyRdes, alphaFdes, alphaRdes  
        
    def lanekeeping(self, localState):
        #note - interp requires rank 0 arrays
        sTable = self.path.s
        kTable = self.path.curvature

        K = np.interp(localState.s, sTable, kTable) #run interp every time - this is slow, but we may be able to get away with    
        deltaFFW, betaFFW, FyFdes, FyRdes, alphaFdes, alphaRdes = self.getDeltaFFW(localState, K)
        deltaFB = self.getDeltaFB(localState, betaFFW)
        delta = deltaFFW + deltaFB
        return delta, deltaFFW, deltaFB, K, alphaFdes, alphaRdes, betaFFW

    def updateInput(self, localState, controlInput):
        delta, deltaFFW, deltaFB, K, alphaFdes, alphaRdes, betaFFW = self.lanekeeping(localState)
        Fx, UxDes, AxDes, FxFFW, FxFB = self.speedTracking(localState)
        controlInput.update(delta, Fx)
        auxVars = {'K': K , 'UxDes': UxDes, 'AxDes': AxDes, 'alphaFdes': alphaFdes,
        'alphaRdes': alphaRdes, 'deltaFFW': deltaFFW, 'deltaFB': deltaFB, 'betaFFW': betaFFW}

        return auxVars


class OpenLoopControl():
    def __init__(self, vehicle, delta = 2 * np.pi / 180, Fx = 100.):
        self.delta = delta
        self.Fx = Fx
            

    #Note, Local state not needed for open loop control, no feedback!    
    def updateInput(self, localState, controlInput):
        
        delta = self.delta
        Fx = self.Fx
            
        #Curvature is 0 for open loop control - no path to track 
        auxVars = {'K': 0., 'UxDes': 0.}
        controlInput.update(delta, Fx)

        return auxVars


class ControlInput:
    def __init__(self):
        self.delta = 0.0
        self.Fx = 0.0

    def update(self, delta, Fx):
        self.delta = delta
        self.Fx = Fx


#Uses FB / FFW infrastructure from Nitin Kapania's thesis, but adds in Nathan Spielberg's
#RNN to learn feedforward sideslip and steer angle. Requires TensorFlow.

class NeuralNetFeedforward():
    def __init__(self, path, vehicle, profile):
        self.path = path
        self.vehicle = vehicle
        self.profile = profile
        self.xLA = 14.2    #lookahead distance, meters
        self.kLK = 0.0538  #proportional gain , rad / meter
        self.kSpeed = 3000.0 #Speed proportional gain - N / (m/s)


        self.NUM_TARGETS   = 2
        self.N_STATE_INPUT = 6
        self.T             = 4
        NUM_FEATURES  = self.N_STATE_INPUT*self.T #6 n state and control 4 is delay states in rnn
        N1            = 256
        N2            = 256
        self.DATA_FULL     = False        
        self.iterCount     = 0

        self.input_buff  = np.zeros(shape = (self.T , self.N_STATE_INPUT))
        self.input_cur   = np.zeros(shape = (1 , NUM_FEATURES))

        #This may be the problem! try float 64 here.
        self.x, self.y = tf.placeholder(tf.float64, shape=[None, NUM_FEATURES]), tf.placeholder(tf.float64, shape=[None,self.NUM_TARGETS])

        #Create a placeholder to dyn switch between batch sizes for test and train...
        self.batch_size = tf.placeholder(tf.int64) #here just 1

        dataset = tf.data.Dataset.from_tensor_slices((self.x, self.y)).batch(self.batch_size).repeat()

        self.iter = dataset.make_initializable_iterator()
        inputs, labels = self.iter.get_next()

        with tf.variable_scope("nn"):
            nn_inputs = tf.slice(inputs, (0, (self.N_STATE_INPUT*self.T - self.N_STATE_INPUT) ), (-1, self.N_STATE_INPUT) )
            net_nn = tf.layers.dense(nn_inputs, N1, activation=tf.nn.relu ) # pass the first value from iter.get_next() as input
            net_nn = tf.layers.dense(net_nn, N2, activation=tf.nn.relu )

            #prediction_nn = tf.layers.dense(net_nn, NUM_TARGETS)
            prediction_nn = tf.layers.dense(net_nn, self.NUM_TARGETS)

            loss_nn = tf.losses.mean_squared_error(prediction_nn, labels) 

        with tf.variable_scope("rnn"):
            net_rnn = tf.layers.dense(inputs, N1, activation=tf.nn.relu ) # pass the first value from iter.get_next() as input
            net_rnn = tf.layers.dense(net_rnn, N2, activation=tf.nn.relu )

            self.prediction_rnn = tf.layers.dense(net_rnn, self.NUM_TARGETS)
            loss_rnn = tf.losses.mean_squared_error(self.prediction_rnn, labels) 

        #Add ops to save and restore all the variables
        saver = tf.train.Saver() 

        self.sess = tf.Session()
        self.sess.run(tf.global_variables_initializer())
        self.save_path = saver.restore(self.sess, "saved_models/model.ckpt")       
        print("Checkpoint restored!")




    def getDeltaFB(self, localState, betaFFW):
        kLK = self.kLK
        xLA = self.xLA
        e = localState.e
        deltaPsi = localState.deltaPsi

        deltaFB = -kLK * (e + xLA * np.sin(deltaPsi + betaFFW))
        return deltaFB


    def speedTracking(self, localState):

        #note - interp requires rank 0 arrays
        AxTable = self.profile.Ax
        UxTable = self.profile.Ux
        sTable = self.profile.s
        m = self.vehicle.m
        fdrag = self.vehicle.dragCoeff
        frr = self.vehicle.rollResistance

        s = localState.s
        Ux = localState.Ux

        AxDes = np.interp(s, sTable, AxTable) #run interp every time - this is slow, but we may be able to get away with
        UxDes = np.interp(s, sTable, UxTable) #run interp every time - this is slow, but we may be able to get away with


        FxFFW = m*AxDes + np.sign(Ux)*fdrag*Ux ** 2 + frr*np.sign(Ux) # Feedforward
        FxFB = -self.kSpeed*(Ux - UxDes) # Feedback
        FxCommand = FxFFW + FxFB
        return FxCommand, UxDes, AxDes, FxFFW, FxFB


    def getDeltaFFW(self, localState, K, delta, ax):
        if localState.Ux > 1:

            self.input_buff       =  np.roll(self.input_buff, 1, axis=0)
            
            #fill in first row
            self.input_buff[0,0]  = localState.Ux
            self.input_buff[0,1]  = localState.Uy
            self.input_buff[0,2]  = localState.r
            self.input_buff[0,3]  = delta
            self.input_buff[0,4]  = ax / 100
            self.input_buff[0,5]  = K

            self.iterCount += 1
            if self.iterCount > self.T:
                self.DATA_FULL = True
            

        else:
            iter_count = 0         #reset the data buffer if we are stopped
            print("Not Running, Vehicle is not in motion")

        if  self.DATA_FULL:

            for j in range(self.T):
                K     = self.input_buff[self.T-j-1, 5]
                ax    = self.input_buff[self.T-j-1, 4]
                delta = self.input_buff[self.T-j-1, 3]
                uy    = self.input_buff[self.T-j-1, 1]
                ux    = self.input_buff[self.T-j-1, 0]

                self.input_cur[0, self.N_STATE_INPUT*j + 0] = K*ux
                self.input_cur[0, self.N_STATE_INPUT*j + 1] = K
                self.input_cur[0, self.N_STATE_INPUT*j + 2] = ux
                self.input_cur[0, self.N_STATE_INPUT*j + 3] = ax #this is desired
                self.input_cur[0, self.N_STATE_INPUT*j + 4] = delta #used delayed curavture as an input?
                self.input_cur[0, self.N_STATE_INPUT*j + 5] = uy #used delayed curavture as an input?

            #Below lines might be redundant since start with zeros array
            self.input_cur[0, self.N_STATE_INPUT*self.T-2]       = 0  #approx rdot
            self.input_cur[0, self.N_STATE_INPUT*self.T-1]       = 0 #approx uydot

            self.sess.run(self.iter.initializer, feed_dict={ self.x: self.input_cur, self.y: np.zeros(shape = (1,2)), self.batch_size: 1})

            pred    = self.sess.run(self.prediction_rnn)
            pred[0][1] = np.arctan(pred[0][1]/ux)
            self.iterCount += 1

            deltaFFW = pred[0][0]
            betaFFW  = pred[0][1]
            FyFdes = 0
            FyRdes = 0
            alphaFdes = 0
            alphaRdes = 0

        else: 
            deltaFFW = 0
            betaFFW = 0
            FyFdes = 0
            FyRdes = 0
            alphaFdes = 0
            alphaRdes = 0

        return deltaFFW, betaFFW, FyFdes, FyRdes, alphaFdes, alphaRdes  
        
    def lanekeeping(self, localState, controlInput):
        #note - interp requires rank 0 arrays
        sTable = self.path.s
        kTable = self.path.curvature

        K = np.interp(localState.s, sTable, kTable) #run interp every time - this is slow, but we may be able to get away with    
        deltaFFW, betaFFW, FyFdes, FyRdes, alphaFdes, alphaRdes = self.getDeltaFFW(localState, K, controlInput.delta, controlInput.Fx / self.vehicle.m)
        deltaFB = self.getDeltaFB(localState, betaFFW)
        delta = deltaFFW + deltaFB
        return delta, deltaFFW, deltaFB, K, alphaFdes, alphaRdes, betaFFW

    def updateInput(self, localState, controlInput):
        delta, deltaFFW, deltaFB, K, alphaFdes, alphaRdes, betaFFW = self.lanekeeping(localState, controlInput)
        Fx, UxDes, AxDes, FxFFW, FxFB = self.speedTracking(localState)
        controlInput.update(delta, Fx)
        auxVars = {'K': K , 'UxDes': UxDes, 'AxDes': AxDes, 'alphaFdes': alphaFdes,
        'alphaRdes': alphaRdes, 'deltaFFW': deltaFFW, 'deltaFB': deltaFB, 'betaFFW': betaFFW}

        return auxVars






##################################HELPER FUNCTIONS ##############################################

def _force2alpha(forceTable, alphaTable, Fdes):
        if Fdes > max(forceTable):
             Fdes = max(forceTable) - 1

        elif Fdes < min(forceTable):
             Fdes = min(forceTable) + 1

        #note - need to slice to rank 0 for np to work
        #note - x values must be increasing in numpy interp!!!
        alpha = np.interp(Fdes, forceTable ,alphaTable)
        

        return alpha







