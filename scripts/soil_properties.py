import numpy as np
from backprop import backprop
from read_file import read_file


def main():
    n = 3  # int(input("Please enter num of inputs: "))
    m = 1  # int(input("Please enter num of outputs: "))
    h = 4
    iterations = 10000  # num of iterations
    threshold = 0.3
    eta = 0.6
    print("Part 3")
    print("Total: ", iterations, "iterations")

    inputs, targets, labels = read_file('digits_train.txt')
    p1 = backprop(n, m, h)  # build new backprop
    p1.train(inputs, targets, iterations, eta)  # train the backprop
    p1.save(p1, "part3.wgt")

    # p1 = p1.load("part3.wgt")

    inputs_test, targets_test, labels_test = read_file('digits_test.txt')
    result = p1.test(inputs_test)
    results = result > threshold

    print("\t" + "\t".join([str(col) for col in np.arange(10)]))  # print first line for the matrix

    confusion = np.add.reduceat(results, np.arange(0, len(results), 250))  # sum each 250 cells
    confusion = np.hstack(([[0], [1], [2], [3], [4], [5], [6], [7], [8], [9]], confusion))  # add vector for first col
    for row in confusion:
        print("\t".join([str(x) for x in row]))


if __name__ == '__main__':
    main()
