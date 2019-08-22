#!/usr/bin/env python
import numpy as np
from backprop import backprop
import csv


def read_file(file_name):

    inputs = []
    targets = []

    result = np.array(list(csv.reader(open(file_name, "r"), delimiter=","))).astype("float")
    inputs = np.array([result[:, 1], result[:, 2], result[:, 3]], dtype='float')
    inputs = np.transpose(inputs)
    targets = np.array([result[:, 0]], dtype='int')
    targets = np.transpose(targets)

    targets = one_hot(targets, 2)

    # print(targets)
    # print(targets)
    return inputs, targets


def check_soil(inputs, targets):
    n = 3
    m = 2
    h = 4
    iterations = 10000  # num of iterations
    threshold = 0.3
    eta = 0.6
    print("Soil type check")
    print("Total: ", iterations, "iterations")

    # inputs, targets = read_file('digits_train.txt')
    p1 = backprop(n, m, h)  # build new backprop
    p1.train(inputs, targets, iterations, eta)  # train the backprop
    p1.save(p1, "soil.wgt")

    p1 = p1.load("soil.wgt")


    inputs_test, targets_test = read_file('scores_test.csv')
    result = p1.test(inputs_test)
    results = result > threshold
    print(result)
    # print(p1.test(np.transpose(np.array([[7199.9998390675], [0.3725629354], [0.5786004936]], dtype='float'))))

def one_hot(a, num_classes):
  return np.squeeze(np.eye(num_classes)[a.reshape(-1)])

def int_to_onehot(n, n_classes):
    v = [0] * n_classes
    v[n] = 1
    return v

if __name__ == '__main__':
    inputs, targets = read_file("scores.csv")
    check_soil(inputs, targets)

    # v = int_to_onehot(1, 5)
    # print(v)
