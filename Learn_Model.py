#Nathan Spielberg
#7.2.18

import tensorflow as tf
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import time
from scipy.signal import butter, lfilter, freqz
from tqdm import tqdm
from Learning_Params import *
from Learning_Functions import *


# Uses Nathan's code to run RNN given train data and test data

def LearnModel(genDataNitin, method = "nathan"):

    #Unpack some useful things
    TRAIN_PERCENT = Param["TRAIN_PERCENT"]
    T             = Param["T"]
    N_STATE_INPUT = Param["N_STATE_INPUT"]
    N1            = Param["N1"]
    N2            = Param["N2"]
    BATCH_SIZE    = Param["BATCH_SIZE"]
    DT            = Param["DT"]
    LEARNING_RATE = Param["LEARNING_RATE"]

    if Param["GEN_DATA"]:
        gen_data     =  generate_data(Param, Veh)
        if Param["ADD_NOISE"]:
            gen_data =  add_noise(gen_data, Param)

        N_SAMPLES    = Param["N_SAMPLES"] #Use the originally generated sample num
    else:
        print("Using Recorded Vehicle Data! ")
        gen_data, N_SAMPLES, exp_t  =  load_data(Param, Veh, TRAIN_FILE)


    if method is "nitin":
        gen_data = genDataNitin


    if Param["INSPECT_TIRES"]:
        a_f = np.zeros((N_SAMPLES, 1))
        a_r = np.zeros((N_SAMPLES, 1))
        Fyf = np.zeros((N_SAMPLES, 1))
        Fyr = np.zeros((N_SAMPLES, 1))

        for i in range(N_SAMPLES):
            a_f[i], Fyf[i], a_r[i], Fyr[i] = get_gen_tires(gen_data[i, N_STATE_INPUT*(T-1)], gen_data[i, N_STATE_INPUT*(T-1)+1], gen_data[i, N_STATE_INPUT*(T-1)+2], 
                gen_data[i, N_STATE_INPUT*(T-1)+3], Cf, Cr, mu, m, a , b)
        plt.figure()
        plt.title("Generated Tire Curve Data")
        plt.scatter(a_f, Fyf, c = 'b', label = "Front Tire")
        plt.scatter(a_r, Fyr, c = 'r', label = "Rear Tire")
        plt.xlabel("Slip Angle (rad)")
        plt.ylabel("Lateral Force (N)")

        plt.figure()
        plt.title("Generated Ux")
        Ux_gen = gen_data[:, N_STATE_INPUT*(T-1)+3]
        plt.plot(Ux_gen)

        
    #Pull out the features and targets from the generated data
    
    
    features = gen_data[:,0:-2]
    targets  = gen_data[:,-2:]

    NUM_FEATURES  = features.shape[1]
    NUM_TARGETS   = targets.shape[1]


    print("The Number of Samples is: " + str(N_SAMPLES))
    print("The Number of Training Samples is "+ str(int(TRAIN_PERCENT*N_SAMPLES)))

    #randomly divide into train and test based on des train test dist.
    indices = np.arange(N_SAMPLES)
    #Shuffle

    #apparently I thought this was a bug before- see what happens without it?
    #np.random.shuffle(indices)

    #This was really bad!
    #np.random.shuffle(indices)

    train_data = (features[ indices[ : int(TRAIN_PERCENT*N_SAMPLES) ] ],
    	targets[ indices[ : int(TRAIN_PERCENT*N_SAMPLES) ] ])

    NUM_TRAIN = train_data[0].shape[0]

    test_data  = (features[ indices[ int(TRAIN_PERCENT*N_SAMPLES): ] ],
    	targets[ indices[ int(TRAIN_PERCENT*N_SAMPLES): ] ])

    #none sizing is for variable batch sizing.

    #This may be the problem! try float 64 here.
    x, y = tf.placeholder(tf.float64, shape=[None,NUM_FEATURES]), tf.placeholder(tf.float64, shape=[None,NUM_TARGETS])

    #Create a placeholder to dyn switch between batch sizes for test and train...
    batch_size = tf.placeholder(tf.int64)

    #Shuffling is integrated now so we can shuffle during training-- maybe switch for testing
    #so no shuffling for testing ie to test in order since time series
    #Check buffer size on shuffle later.
    dataset = tf.data.Dataset.from_tensor_slices((x, y)).batch(batch_size).repeat()
    #shuffle(buffer_size=1000).
    #// denotes floor division
    n_batches = NUM_TRAIN // BATCH_SIZE

    iter = dataset.make_initializable_iterator()
    inputs, labels = iter.get_next()


    #Compare these three models for Thursday!!!!!

    #Without Regularization
    #For Regular FFW Network
    #These were brackets before which seems wrong to me

    ####and lastly for the bike model. tf.squeeze( 
    r     = tf.slice(inputs,[0, (N_STATE_INPUT*T-4)],[-1,1])
    uy    = tf.slice(inputs,[0, (N_STATE_INPUT*T-3)],[-1,1])
    delta = tf.slice(inputs,[0, (N_STATE_INPUT*T-2)],[-1,1])
    ux    = tf.slice(inputs,[0, (N_STATE_INPUT*T-1)],[-1,1])
    fxf   = tf.slice(inputs,[0, (N_STATE_INPUT*T  )],[-1,1]) * Veh["m"] * 100.0

    nn_inputs = tf.slice(inputs, (0, (N_STATE_INPUT*T - N_STATE_INPUT) ), (-1, N_STATE_INPUT+1) )
    net_nn = tf.layers.dense(nn_inputs, N1, activation=tf.nn.relu ) # pass the first value from iter.get_next() as input
    net_nn = tf.layers.dense(net_nn, N2, activation=tf.nn.relu )
    net_nn = tf.layers.dense(net_nn, N2, activation=tf.nn.relu )

    #prediction_nn = tf.layers.dense(net_nn, NUM_TARGETS)
    prediction_nn = tf.concat([r, uy], 1) + DT * tf.layers.dense(net_nn, NUM_TARGETS)


    loss_nn = tf.losses.mean_squared_error(prediction_nn, labels) 

    net_rnn = tf.layers.dense(inputs, N1, activation=tf.nn.relu ) # pass the first value from iter.get_next() as input
    net_rnn = tf.layers.dense(net_rnn, N2, activation=tf.nn.relu )
    net_rnn = tf.layers.dense(net_rnn, N2, activation=tf.nn.relu )

    #prediction_rnn = tf.layers.dense(net_rnn, NUM_TARGETS)
    prediction_rnn = tf.concat([r, uy], 1) + DT * tf.layers.dense(net_rnn, NUM_TARGETS)

    loss_rnn = tf.losses.mean_squared_error(prediction_rnn, labels) 


    # model params (normalize variables)
    a_tf = tf.constant(Veh["a"], dtype=tf.float64)
    b_tf = tf.constant(Veh["b"], dtype=tf.float64)
    L = tf.constant(Veh["a"] + Veh["b"], dtype=tf.float64)
    g = tf.constant(9.81, dtype=tf.float64)

    m_const     = tf.constant(Veh["m"], dtype=tf.float64)
    I_const     = tf.constant(Veh["Izz"], dtype=tf.float64)
    Ca_f_const  = tf.constant(Veh["Cf"], dtype=tf.float64)
    Ca_r_const  = tf.constant(Veh["Cr"], dtype=tf.float64)
    mu_const    = tf.constant(Veh["mu"], dtype=tf.float64)
    #Cd0_const   = tf.constant(veh['Cd0'], dtype=tf.float32)
    #Cd1_const   = tf.constant(veh['Cd1'], dtype=tf.float32)

    loc         = Param["loc"] #This defines best first guess mean of parameter multipliers
    #IE 1.1 is 10% more on average on parameters or .9 10% less
    scale       = Param["scale"] #This is the variance on our initial param percentage estimate

    m_norm      = tf.Variable(np.random.normal(loc, scale), name='m'   , dtype=tf.float64)
    I_norm      = tf.Variable(np.random.normal(loc, scale), name='I'   , dtype=tf.float64)
    Ca_f_norm   = tf.Variable(np.random.normal(loc, scale), name='Ca_f', dtype=tf.float64)
    Ca_r_norm   = tf.Variable(np.random.normal(loc, scale), name='Ca_r', dtype=tf.float64)
    mu_norm     = tf.Variable(np.random.normal(loc, scale), name='mu'  , dtype=tf.float64)

    m_tf = m_const;
    #I_tf = I_norm*I_const;
    I_tf = I_const;
    #Ca_f = Ca_f_const;
    Ca_f = Ca_f_norm*Ca_f_const;
    Ca_r = Ca_r_norm*Ca_r_const;
    #Ca_r = Ca_r_const;
    mu_tf = mu_norm*mu_const;
    #mu_tf = mu_const;
    #Cd0 = Cd0_const;
    #Cd1 = Cd1_const;

    # model
    fz_f = (1.0/L)*(m_tf*b_tf*g)
    fz_r = (1.0/L)*(m_tf*a_tf*g)
    alpha_f = tf.atan( tf.div( (uy + a_tf*r), ux) ) - delta
    alpha_r = tf.atan( tf.div( (uy - b_tf*r), ux) )
    fy_f = fiala_tf(alpha_f, Ca_f, mu_tf, fz_f)
    fy_r = fiala_tf(alpha_r, Ca_r, mu_tf, fz_r)
    #fy_f = -Ca_f*alpha_f
    #fy_r = -Ca_r*alpha_r
    #fx_drag = Cd0 + Cd1*ux

    #Calc Derivs
    r_dot   = (1/I_tf)*(a_tf*fy_f*tf.cos(delta) + a_tf*fxf*tf.sin(delta)  - b_tf*fy_r)
    uy_dot  = (1/m_tf)*(  fy_f*tf.cos(delta) +  fxf*tf.sin(delta) + fy_r)           - r*ux
    ay      = (1/m_tf)*(  fy_f*tf.cos(delta) +  fxf*tf.sin(delta) + fy_r) 

    #Predict Next State
    model_next_r = euler_int(r, r_dot, DT)
    model_next_uy = euler_int(uy, uy_dot, DT)

    prediction_bike = tf.concat([model_next_r, model_next_uy], 1)

    loss_bike = tf.losses.mean_squared_error(prediction_bike, labels)

    train_nn = tf.train.AdamOptimizer(learning_rate   = Param["LEARNING_RATE"]).minimize(loss_nn)
    train_rnn = tf.train.AdamOptimizer(learning_rate  = Param["LEARNING_RATE"]).minimize(loss_rnn)
    train_bike = tf.train.AdamOptimizer(learning_rate = Param["LEARNING_RATE"]).minimize(loss_bike)

    #Add ops to save and restore all the variables
    saver = tf.train.Saver()

    start = time.time()

    train_cost_nn = []
    train_cost_rnn = []
    train_cost_bike = []

    test_cost  = []

    #########################################################################
    #LEARNING
    #########################################################################

    #Now time to do some learning
    with tf.Session() as sess:
        sess.run(tf.global_variables_initializer())

        sess.run(iter.initializer, feed_dict={ x: features, y: targets, batch_size: N_SAMPLES})
        uy_tf_init      = sess.run(model_next_uy)
        r_tf_init    = sess.run(model_next_r)


        if Param["RESTORE"]:
            save_path = saver.restore(sess, "saved_models/model.ckpt")
            print("Model restored from saved model! ")

        if Param["TRAIN"]:

            print("The Initial Cf Values is: " + str(sess.run(Ca_f))  ) 
            print("The Initial Cr Values is: " + str(sess.run(Ca_r))  )
            print("The Initial mu Values is: " + str(sess.run(mu_tf)) )
            print("The Initial Izz Values is: " + str(sess.run(I_tf)) )

            
            print('Training...')

            for i in tqdm( range( Param["EPOCHS"]) ):
            	#Need to make big change here for cost calculation.
            	#sess.run(iter.initializer, feed_dict={ x: test_data[0], y: test_data[1], batch_size: test_data[0].shape[0]})
            	#test_cost.append(sess.run(loss))
                
                # initialise iterator with train data
                sess.run(iter.initializer, feed_dict={ x: train_data[0], y: train_data[1], batch_size: train_data[0].shape[0]})


                #Start with debugging stuff here:
                #print(sess.run(inputs))
                #print(sess.run(nn_inputs))
                #print(sess.run(labels))


                nnc, rnnc, bikec = sess.run([loss_nn, loss_rnn, loss_bike])
                #nnc = sess.run(loss_nn)
                #print(nnc)
                #bikec = sess.run([loss_bike])
                train_cost_nn.append(nnc)
                train_cost_rnn.append(rnnc)
                train_cost_bike.append(bikec)

                sess.run(iter.initializer, feed_dict={ x: train_data[0], y: train_data[1], batch_size: BATCH_SIZE})
            	#tot_loss = 0
                for _ in range(n_batches):
            		#start = time.time()
                    _, _, _ = sess.run([train_nn, train_rnn, train_bike])
                    #_ = sess.run([train_nn])

            print("The Oracle has spoken!")
            print("The Final Cf Values is: " + str(sess.run(Ca_f)) + " And the Actual is: " + str( Veh["Cf"])  )
            print("The Final Cr Values is: " + str(sess.run(Ca_r)) + " And the Actual is: " + str( Veh["Cr"])  )
            print("The Final mu Values is: " + str(sess.run(mu_tf))+ " And the Actual is: " + str( Veh["mu"])  )
            print("The Final Izz Values is: " + str(sess.run(I_tf))+ " And the Actual is: " + str( Veh["Izz"]) )

            print("The Final Training Cost for NN is: " + str(train_cost_nn[Param["EPOCHS"]-1]))
            print("The Final Training Cost for RNN is: " + str(train_cost_rnn[Param["EPOCHS"]-1]))
            print("The Final Training Cost for Bike is: " + str(train_cost_bike[Param["EPOCHS"]-1]))

        
        
        if Param["SAVE"]:
            save_path = saver.save(sess, "saved_models/model.ckpt")
            print("Model saved in path: %s" % save_path)


        #Check to make sure everything is being shufled as expected...
        #sess.run(iter.initializer, feed_dict={ x: train_data[0], y: train_data[1], batch_size: train_data[0].shape[0]})
        #uy_tf      = sess.run(uy)

        #first see if the slip angles are matching!
        #load in data since not training
        sess.run(iter.initializer, feed_dict={ x: features, y: targets, batch_size: N_SAMPLES})

        pred_nn      = sess.run(prediction_nn)
        pred_rnn     = sess.run(prediction_rnn)

        uy_next_tf   = sess.run(model_next_uy)
        uy_nn        = pred_nn[:,1]
        uy_rnn       = pred_rnn[:,1]

        r_next_tf    = sess.run(model_next_r)
        r_nn         = pred_nn[:,0]
        r_rnn        = pred_rnn[:,0]

        lbls         = sess.run(labels)
        uy_next_dat  = lbls[:,1]
        r_next_dat   = lbls[:,0]
        pred_test    = sess.run(prediction_bike)
        uy_next_pred = pred_test[:,1]
        r_next_pred  = pred_test[:,0]

        ind_test = np.arange(len(uy_next_dat))      

        return train_cost_bike, train_cost_nn, train_cost_rnn
    

        
        
        

