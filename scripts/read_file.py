#!/usr/bin/env python
import numpy as np
from backprop import backprop
import csv


def read_file(file_name):

    inputs = []
    targets = []
    # X = []
    # Y = []
    # Z = []
    #
    # f = open(file_name)
    # f.readline()
    # for line in f:
    #     if line != '':
    #         line = line.strip()
    #         columns = line.split()
    #         x = columns[1]
    #         y = columns[2]
    #         z = columns[3]
    #         X.append(x)
    #         Y.append(y)                #appends data in list
    #         Z.append(z)
    #
    # A = np.array([X,Y,Z], dtype=float)
    # print(A)
    # return inputs, targets
    #     # for i in range(len(z)):
    #     #     print (w[i])

    result = np.array(list(csv.reader(open(file_name, "r"), delimiter=","))).astype("float")
    inputs = np.array([result[:, 1], result[:, 2],result[:, 3]], dtype='float')
    inputs = np.transpose(inputs)
    targets = result[:, 0]
    return inputs, targets


def check_soil(inputs, targets):
    n = 3
    m = 1
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

    # p1 = p1.load("part3.wgt")

    # inputs_test, targets_test, labels_test = read_file('digits_test.txt')
    # result = p1.test(inputs_test)
    # results = result > threshold


if __name__ == '__main__':
    inputs, targets = read_file("scores.csv")
    check_soil(inputs, targets)

