#!/usr/bin/env python
import numpy as np
import pickle
import math


class backprop:

    def __init__(self, n, m, h):
        self.wIH = (2.0 * np.random.random((n+1, h)) - 1.0)  # random values between -0.05 to 0.05 for weights
        self.wHO = (2.0 * np.random.random((h+1, m)) - 1.0)
        #print(self.w)  # print to check weights
        self.n = n
        self.m = m
        self.h = h
        self.rms = 0

    def __str__(self):
        return 'A perceptron with ' + str(self.n)+' inputs and ' + str(self.m) + ' outputs.'

    def sigmoid(self, x):
        return 1.0/(1.0 + np.exp(-x))

    def sigmoidPrime(self, x):
        return np.multiply(x, (1 - x))

    def test(self, inputs):
        H = np.dot(np.hstack((inputs, np.ones([len(inputs), 1]))), self.wIH)
        H = self.sigmoid(H)
        O = np.dot(np.hstack((H, np.ones([len(H), 1]))), self.wHO)
        O = self.sigmoid(O)
        return O

    def train(self, I, T, iterations, eta=0.5, mu=0, rms_flag=0):

        if rms_flag:
            rms_array = np.zeros((iterations,1))
        for i in range(iterations):  # loop for iterations
            delta_wIH = np.zeros((self.n + 1, self.h))  # weights changes
            delta_wHO = np.zeros((self.h + 1, self.m))
            H_net = np.dot(np.hstack((I, np.ones([len(I), 1]))), self.wIH)
            H = self.sigmoid(H_net)
            O_net = np.dot(np.hstack((H, np.ones([len(H), 1]))), self.wHO)
            O = self.sigmoid(O_net)
            if rms_flag:
                rms_array[i] = self.RMS(T, O)
            delta_O = np.multiply((T - O), self.sigmoidPrime(O))
            delta_H = np.multiply(np.dot(delta_O, np.transpose(self.wHO))[:, :-1], self.sigmoidPrime(H))
            delta_wIH += np.dot(np.transpose(np.hstack((I, np.ones([len(I), 1])))), delta_H)
            delta_wHO += np.dot(np.transpose(np.hstack((H, np.ones((len(H), 1))))), delta_O)
            self.wIH += np.multiply(eta, delta_wIH) / len(I) + np.multiply(mu, delta_wIH)
            self.wHO += np.multiply(eta, delta_wHO) / len(I)
        print("Completed", i + 1, "/", iterations, "iterations")
        if rms_flag:
            self.rms = rms_array
        return self.wIH, self.wHO

    def save(self, p1, file_name):
        pickle_out = open(file_name, "wb")
        pickle.dump(p1, pickle_out)
        pickle_out.close()
        return

    def load(self, file_name):
        return pickle.load(open(file_name, "rb"))

    def rms_return(self):
        return self.rms

    def RMS(self, T, O):
        return np.sqrt(np.mean(np.multiply((T - O), (T - O))))
