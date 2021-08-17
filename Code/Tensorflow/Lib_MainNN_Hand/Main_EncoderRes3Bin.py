# CONTACT MLP
import numpy as np
import tensorflow as tf
import sys

sys.path.append('../Lib_Opt')
from MLP_new import MLP
from NeuralNetwork import NeuralNetwork

sys.path.append('../Lib_Utils')
import Utils as utils


class MainNN(NeuralNetwork):
    def __init__(self):
        NeuralNetwork.__init__(self)

    def BuildModel(self,
                   # For data process
                   load_path, save_path, type_normalize,
                   # For main network
                   hiddenDim=[2048, 2048, 1024],
                   activations=[tf.nn.elu, tf.nn.elu, tf.nn.elu, 0],
                   # For encoder
                   index_encoders=[],
                   dim_encoders=[],
                   activation_encoders=[]
                   ):

        # Process Data, why process here because need the Dims to build Tensors
        self.ProcessData(load_path, save_path, type_normalize)
        # Initialize Placeholders
        self.nn_X = tf.placeholder(tf.float32, [None, self.input_dim], name=self.name_tensor_input)
        self.nn_Y = tf.placeholder(tf.float32, [None, self.output_dim], name=self.name_tensor_output)
        self.nn_keep_prob_main = tf.placeholder(tf.float32, name=self.name_tensor_keep_prob_main)
        self.nn_keep_prob_encoders = tf.placeholder(tf.float32, [len(dim_encoders)],
                                                    name=self.name_tensor_keep_prob_encoders)
        self.nn_lr_c = tf.placeholder(tf.float32, name='nn_lr_c')

        # Encoding
        self.num_encoders = len(index_encoders)
        if self.num_encoders > 0:
            assert len(dim_encoders) == len(activation_encoders)
            self.encoders = []
            output_encoders = []
            for i in range(self.num_encoders):
                # ===============print==============
                print('--------------------')
                print('Encoder ', i)
                print('Input dim ', len(index_encoders[i]))
                print('Hidden dim ', dim_encoders[i])
                print('Activations ', str(activation_encoders[i]))
                print('--------------------')
                # ===============print==============
                input_encoder = tf.gather(self.nn_X, index_encoders[i], axis=-1)
                if len(dim_encoders[i]) > 0:  # if need to encode
                    encoder_dims = [len(index_encoders[i])] + dim_encoders[i]
                    encoder_activations = activation_encoders[i]
                    encoder = MLP(self.rng,
                                  encoder_dims,
                                  encoder_activations,
                                  'encoder%0i' % i)
                    self.encoders.append(encoder)
                    # SAVER FOR ENCODERS
                    output_encoder = encoder.forward(input_encoder, self.nn_keep_prob_encoders[i])
                else:  # else just copy the input
                    output_encoder = input_encoder
                tf.add_to_collection("nn_latent_encoder%0i" % i, output_encoder)
                output_encoders.append(output_encoder)
            input_finalComp = tf.concat(output_encoders, axis=-1)
        else:
            input_finalComp = self.nn_X

        hiddenDim = input_finalComp.shape[-1].value
        print("activation", activations)

        # ============== res0 ==============
        # elu + fc
        out0_before = activations(input_finalComp)
        mlp0 = MLP(self.rng,
                  [hiddenDim] + [hiddenDim],
                  [0],
                  'MLP0')
        out0 = mlp0.forward(out0_before, self.nn_keep_prob_main)
        out0 = out0 + out0_before

        # ==========res1==========
        out1_before = activations(out0)
        mlp1 = MLP(self.rng,
                  [hiddenDim] + [hiddenDim],
                  [0],
                  'MLP1')
        out1 = mlp1.forward(out1_before, self.nn_keep_prob_main)
        out1 = out1 + out1_before + out0_before

        # ==========res2==========
        out2_before = activations(out1)
        mlp2 = MLP(self.rng,
                  [hiddenDim] + [hiddenDim],
                  [0],
                  'MLP2')
        out2 = mlp2.forward(out2_before, self.nn_keep_prob_main)
        out2 = out2 + out2_before

        # ==========res3==========
        out3_before = activations(out2)
        mlp3 = MLP(self.rng,
                  [hiddenDim] + [hiddenDim],
                  [0],
                  'MLP3')
        out3 = mlp3.forward(out3_before, self.nn_keep_prob_main)
        out3 = out3 + out3_before + out2_before

        self.DensResNet = [mlp0, mlp1, mlp2, mlp3]

        # ============== final output ================
        # here could choose which "add on" we want
        nn_addOnFinal = out3
        # here add one more fc layer without activation
        endMLP = MLP(self.rng,
                     [hiddenDim] + [self.output_dim],
                     [0],
                     'endMLP')
        self.decoder = endMLP

        # final output
        self.nn_prediction = endMLP.forward(nn_addOnFinal, self.nn_keep_prob_main)
        tf.add_to_collection("nn_prediction", self.nn_prediction)

        # L2 loss
        self.mse_loss = tf.reduce_mean(tf.square(self.nn_Y - self.nn_prediction))
        # final loss
        self.loss = self.mse_loss
        self.optimizer = tf.train.AdamOptimizer(learning_rate=self.nn_lr_c).minimize(self.loss)

    def Train(self,
              # Tensorflow Session
              sess,
              # Model name for Saving
              name_model,
              # Flag/Path of test data
              path_test='',
              # Flag for saving model and Flag for saving bin
              flag_save_tf=False, flag_save_bin=False,
              # Saving settings
              step_save=1, max_save=1,
              # HyperParameters
              keep_prob_main=0.7, batch_size=32, epoch=40, learning_rate_ini=0.0001,
              keep_prob_encoders=[]
              ):

        # Print Training Information
        total_batch = int(self.data_size / batch_size)
        print('Training information')
        print('Total Batch', '->', total_batch)
        print('--------------------')
        print('Input', 'X', '->', self.input_dim)
        print('Output', 'Y', '->', self.output_dim)
        print('--------------------')
        print('Main Keep Prob', '->', keep_prob_main)
        print('Encoders Keep Prob', '->', keep_prob_encoders)

        # Print Testing Information if test
        if (len(path_test) > 0):
            self.LoadTestData(path_test)
            if self.data_size_test > batch_size:
                batch_size_test = batch_size
            else:
                batch_size_test = self.data_size_test
            total_batch_test = int(self.data_size_test / batch_size_test)

        # Initialize Graph
        sess.run(tf.global_variables_initializer())
        # Initialize Saver
        saver = tf.train.Saver(max_to_keep=max_save)
        # Start training
        I = np.arange(self.data_size)
        train_loss = []
        test_loss = []
        for e in range(epoch):
            # Randomly select training set
            self.rng.shuffle(I)
            avg_cost_train = 0
            for i in range(total_batch):
                print('Progress', round(i / total_batch, 2), "%", end="\r")
                index_train = I[i * batch_size:(i + 1) * batch_size]
                batch_xs = self.input_data[index_train]
                batch_ys = self.output_data[index_train]
                feed_dict = {self.nn_X: batch_xs,
                             self.nn_Y: batch_ys,
                             self.nn_keep_prob_main: keep_prob_main,
                             self.nn_keep_prob_encoders: keep_prob_encoders,
                             self.nn_lr_c: learning_rate_ini}
                l, _, = sess.run([self.mse_loss, self.optimizer],
                                 feed_dict=feed_dict)
                avg_cost_train += l / total_batch
            print('Epoch:', '%04d' % (e + 1), 'Training Loss =', '{:.9f}'.format(avg_cost_train))
            train_loss.append(avg_cost_train)
            # Test
            if (len(path_test) > 0):
                avg_cost_test = 0
                for i in range(total_batch_test):
                    batch_xs = self.input_test[i * batch_size_test:(i + 1) * batch_size_test]
                    batch_ys = self.output_test[i * batch_size_test:(i + 1) * batch_size_test]
                    feed_dict = {self.nn_X: batch_xs,
                                 self.nn_Y: batch_ys,
                                 self.nn_keep_prob_main: 1.0,
                                 self.nn_keep_prob_encoders: [1.0] * len(keep_prob_encoders)}
                    l, = sess.run([self.mse_loss],
                                  feed_dict=feed_dict)
                    avg_cost_test += l / total_batch_test
                print('Epoch:', '%04d' % (e + 1), 'Testing Loss =', '{:.9f}'.format(avg_cost_test))
                test_loss.append(avg_cost_test)

            # SAVER
            if (flag_save_bin and e % step_save == 0) or e + 1 == epoch:
                for i in range(len(self.encoders)):
                    self.encoders[i].saveNNTranspose(sess, self.save_path + '/' + 'Encoder%0i' % i)
                for i in range(len(self.DensResNet)):
                    self.DensResNet[i].saveNNTranspose(sess, self.save_path + '/' + 'DenseRes%0i' % i)
                self.decoder.saveNNTranspose(sess, self.save_path + '/' + 'Decoder')
            
            if flag_save_tf and (e % step_save == 0 or e + 1 == epoch):
                saver.save(sess, self.save_path + '/' + name_model, global_step=e)

        print('Learning Finished')
