# Vanilla MLP
import tensorflow as tf
import numpy as np


class MLP(object):
    def __init__(self, rng, dim_layers, activation, name):
        """
        feed forward network that is usually used as encoder/decoder
        :param rng: random seed for initialization
        :param dim_layers: list of int values for dim
        :param activation: list of activation functions
        :param keep_prob: keep prob
        :param name: name
        """
        self.name = name

        """rng"""
        self.initialRNG = rng

        """"NN structure"""
        self.dim_layers = dim_layers
        self.activation = activation
        self.num_layer = len(activation)
        """Build NN"""
        self.buildNN()

    def buildNN(self, initialzer=tf.contrib.layers.xavier_initializer()):
        assert self.num_layer + 1 == len(self.dim_layers)
        self.W = []
        self.b = []
        for i in range(self.num_layer):
            weights = tf.get_variable(self.name + "W%0i" % i, shape=[self.dim_layers[i], self.dim_layers[i + 1]],
                                      initializer=initialzer)
            bias = tf.Variable(tf.zeros(self.dim_layers[i + 1], tf.float32), name=self.name + 'b%0i' % i)
            self.W.append(weights)
            self.b.append(bias)
        print(self.name + " MLP is built", self.dim_layers, self.activation)

    def forward(self, input, keep_prob):
        H = input
        for i in range(self.num_layer):
            H = tf.nn.dropout(H, keep_prob=keep_prob)
            H = tf.matmul(H, self.W[i]) + self.b[i]
            if self.activation[i] != 0:
                if self.activation[i] == tf.nn.softmax:
                    H = self.activation[i](H, axis=0)
                else:
                    H = self.activation[i](H)
        return H

    def saveNN(self, sess, path):
        for i in range(len(self.W)):
            sess.run(self.W[i]).tofile(path + '_w%0i.bin' % i)
            sess.run(self.b[i]).tofile(path + '_b%0i.bin' % i)

    def saveNNTranspose(self, sess, path):
        for i in range(len(self.W)):
            np.transpose(sess.run(self.W[i])).tofile(path + '_w%0i.bin' % i)
            sess.run(self.b[i]).tofile(path + '_b%0i.bin' % i)
