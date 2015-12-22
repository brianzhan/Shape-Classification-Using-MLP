import random
import string
import numpy as np
import math
import scipy as sp
from scipy import interpolate
from scipy import signal
import os, os.path
import matplotlib.pyplot as plt

# I think tanh works faster over less hidden layers or something like that
def sigmoid(x, Derivative = False):
    if Derivative is True:
        return 1-np.power(x, 2)
    else:
        return np.tanh(x)


# initializing all relevant instances
class NeuralNetwork:
    def __init__(self, nInputs, nHiddenLayers, nOutputs):
        # declaring these as global throughout the class
        self.nInputs = nInputs # +1 for bias node
        self.nHiddenLayers = nHiddenLayers
        self.nOutputs = nOutputs
        # initialize arrays
        self.arrayInputs = np.ones(nInputs)
        self.arrayHiddens = np.ones(nHiddenLayers) # by this I mean array hidden nodes
        self.arrayOutputs = np.ones(nOutputs)
        self.weightedInputs = np.zeros((self.nInputs, self.nHiddenLayers))
        self.weightOutputs = np.zeros((self.nHiddenLayers, self.nOutputs))
        self.prevInputWeightDelta = np.zeros((self.nInputs, self.nHiddenLayers))
        self.prevOutputWeightDelta = np.zeros((self.nHiddenLayers,self.nOutputs))
        for i in range(self.nInputs):
            for j in range(self.nHiddenLayers):
                self.weightedInputs[i][j] = random.uniform(-0.1,0.1) # unfiorm distribution
        for i in range(self.nHiddenLayers):
            for j in range(self.nOutputs):
                self.weightOutputs[i][j] = random.uniform(-0.1,0.1)



    def InputActivations(self, inputArray):
        for i in range(self.nInputs): # keep track of all inputs
            self.arrayInputs[i] = inputArray[i]
        for layerColumn in range(self.nHiddenLayers): # activate inputs to hidden layer
            sigmoidSum = 0.0
            for inputRow in range(self.nInputs):
                sigmoidSum += np.multiply(self.arrayInputs[inputRow],self.weightedInputs[inputRow][layerColumn])
            self.arrayHiddens[layerColumn] = sigmoid(sigmoidSum) # store weights
        # output activations. Usually there'd be a for loop over nOutputs but in this case that's not really necessary bc one output column
        for layerColumn in range(self.nOutputs):
            sum = 0.0
            for hiddenRow in range(self.nHiddenLayers):
                sum += np.multiply(self.arrayHiddens[hiddenRow],self.weightOutputs[hiddenRow][layerColumn])
            self.arrayOutputs[layerColumn] = sigmoid(sum)
        return self.arrayOutputs[:]

    def backPropagate(self, targets, LearningRate, Momentum):
        # calculate error terms for output
        outputDelta = np.zeros(self.nOutputs)
        for outputRow in range(self.nOutputs):
            outputDelta[outputRow] = np.multiply(sigmoid(self.arrayOutputs[outputRow], Derivative=True),np.subtract(targets[outputRow],self.arrayOutputs[outputRow]))
        # calculate error terms for hidden
        hiddenDelta = np.zeros(self.nHiddenLayers)
        for hiddenColumn in range(self.nHiddenLayers):
            error = 0.0
            for outputRow in range(self.nOutputs):
                error+= np.multiply(outputDelta[outputRow],self.weightOutputs[hiddenColumn][outputRow])
            hiddenDelta[hiddenColumn] = sigmoid(self.arrayHiddens[hiddenColumn], Derivative=True) * error
        # output weights
        for hiddenColumn in range(self.nHiddenLayers):
            for outputRow in range(self.nOutputs):
                outputChange = np.multiply(outputDelta[outputRow],self.arrayHiddens[hiddenColumn])
                self.weightOutputs[hiddenColumn][outputRow] = self.weightOutputs[hiddenColumn][outputRow] + LearningRate*outputChange + Momentum*self.prevOutputWeightDelta[hiddenColumn][outputRow]
                self.prevOutputWeightDelta[hiddenColumn][outputRow] = outputChange
        # input weights
        for i in range(self.nInputs):
            for j in range(self.nHiddenLayers):
                inputDelta = np.multiply(hiddenDelta[j],self.arrayInputs[i])
                self.weightedInputs[i][j] = self.weightedInputs[i][j] + LearningRate*inputDelta + Momentum*self.prevInputWeightDelta[i][j]
                self.prevInputWeightDelta[i][j] = inputDelta

        # calculate error
        error = 0.0
        for k in range(len(targets)):
            error = error + 0.5*(targets[k]-self.arrayOutputs[k])**2
        return error


    def test(self, input, testset):
        for inputNodes in input:
            out = self.InputActivations(inputNodes[0])
            if testset is 1:
                print "Not Learned Result"
            if testset is 2:
                print "Quarter Learned Result"
            if testset is 3:
                print "Half Learned Result"
            if testset is 4:
                print "Three-Quarter Learned Result"
            if testset is 5:
                print "Fully Learned Result"
            print out
            #commented to make viewing easier
            #print(p[0], '->', self.InputActivations(p[0]))
        return out

    def weights(self):
        print('Input weights:')
        for i in range(self.nInputs):
            print(self.weightedInputs[i])
        print('Output weights:')
        for j in range(self.nHiddenLayers):
            print(self.weightOutputs[j])

    def train(self, inputArray, numIterate=10000, LearningRate=0.5, Momentum=0.1):
        for i in range(numIterate):
            error = 0.0
            for inp in inputArray:
                input = inp[0]
                desiredOutput = inp[1]
                self.InputActivations(input)
                error += self.backPropagate(desiredOutput, LearningRate, Momentum)
            j = int(i)         
            if i % 100 is 0:
                print error
                #print i
                self.saveWeights(2)
            if i is 0:
                self.saveWeights(1)
            if j is 3000:
                self.saveWeights(2)
        self.saveWeights(3)

    def saveWeights(self, testset):
        if testset is 1:
            folder = "/Not"
        if testset is 2:
            folder = "/Quarter"
        if testset is 3:
            folder = "/Half"
        if testset is 4:
            folder = "/ThreeQuarter"
        if testset is 5:
            folder = "/Full"
        sInputs = os.getcwd() + "/saved" + folder + "/savedInputs.txt"
        sOutputs = os.getcwd() + "/saved" + folder + "/savedOutputs.txt"
        np.savetxt(sInputs, self.weightedInputs)
        np.savetxt(sOutputs, self.weightOutputs)

    def loadWeights(self, testset):
        if testset is 1:
            folder = "/Not"
        if testset is 2:
            folder = "/Quarter"
        if testset is 3:
            folder = "/Half"
        if testset is 4:
            folder = "/ThreeQuarter"
        if testset is 5:
            folder = "/Full"
        sInputs = os.getcwd() + "/saved" + folder + "/savedInputs.txt"
        sOutputs = os.getcwd() + "/saved" + folder + "/savedOutputs.txt"
        self.weightedInputs = np.loadtxt(sInputs)
        self.weightOutputs = np.loadtxt(sOutputs)

#---------------------------------------------Start data manipulation code --------------------------------------#
def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

#removes zeros from sensor readings
def remove_zeros(arr):
    number = (arr.size/2)
    build = [[0,0]];
    for x in range (0,number):
        if arr[x,0] != 0:
            build = np.vstack([build , arr[x]])
    return np.delete(build, (0), axis = 0)

#inputs - finput: filehandler of the text file that is saved
#         nodes: number of nodes in the network
#         normmax and normmin: what to normalize dataset by, currently 8.5 based on max squares and 0.8 based on min circles
#output - rn (radius_new) an array of distances that are equally 
#spaced based on the data and the number of nodes
def data_to_array(finput,nodes):
    global minimumdist, maximumdist
    known = 13
    T = np.loadtxt(finput)
    T = remove_zeros(T)
    row = (T.size/2) - 1
    vel = (2*math.pi)/T[row,1]
    dist = np.exp((T[:,0] - 4491.68)/(-1223.778))
    r = known - dist
    # r = dist
    rs = moving_average(r)
    rs = np.append(r[0],rs)
    rs = np.append(rs, r[-1])
    rs = sp.signal.medfilt(rs,3)

    mmax = np.amax(rs)
    mmin = np.amin(rs)

    #if (mmax > maximumdist):
    #    maximumdist = mmax
    #    print('maximum is', mmax)
    #if (mmin < minimumdist):
    #    minimumdist = mmin
    #    print('minimum is',mmin)

    normmin = -20.8
    normmax = 8.51


    omega = T[:,1]*vel
    omegan = np.linspace(0.04,2*math.pi,num = nodes)

    f = interpolate.interp1d(omega,rs)

    rn = f(omegan)
    rn = (rn-normmin)/(normmax-normmin)
    

    xp = rs*np.sin(omega);
    yp = rs*np.cos(omega);

    xpn = rn*np.sin(omegan);
    ypn = rn*np.cos(omegan);

    #plt.plot(omega,r)
    #plt.plot(omegan,rn,'yo-')
    #plt.show()
    # plt.plot(xp,yp)
    #plt.plot(xpn,ypn)
    #plt.show()

    #look at data by uncommenting below, omega by distance graph
    # save = np.column_stack((omegan,rn))
    # np.savetxt("save.txt",save) 
    return rn

#this function creates the list that is inputted into the neural net
#the directories for the data files must be in the correct folder and also named accordingly (0-n)
def pattern_creator():
    sqdir = os.getcwd() + "/squares"
    squares = os.listdir(sqdir)
    cdir = os.getcwd() + "/circles"
    circles = os.listdir(cdir)
    tdir = os.getcwd() + "/triangles"
    triangles = os.listdir(tdir)

    pat = []
    x = []
    for i in range(len(squares)):
       line = one_run([1,0,0],sqdir+"/"+str(squares[i]))
       print line
       pat.append(line)

    for j in range(len(circles)):
        line = one_run([0,1,0],cdir+"/"+str(circles[j]))
        print line
        pat.append(line)

    for k in range(len(triangles)):
        line = one_run([0,0,1],tdir+"/"+str(triangles[k]))
        pat.append(line)
    return pat
#this function takes in the output for the neural net and the input name of an individual test and returns
#one formatted line
def one_run(output,name):
    #y = data_to_array(name,20,8.5,0.8)
    y = data_to_array(name,20)
    y = y.tolist()
    line = [y,output]
    return line


def train_net():
    pat = pattern_creator()
    network = NeuralNetwork(20,7,3) # nInputs, n
    network.train(pat)

def demo(wpath,testset):
    test = [one_run([0,1],wpath)]
    network = NeuralNetwork(20,7,1)
    #network.train(pat)
    network.loadWeights(testset)
    out = network.test(test, testset)
    if out[0] > out[1] and out[0] > out[2]:
        print "Identified as Square\n"
    if out[1] > out[0] and out[1] > out[2]:
        print "Identified as Circle\n"
    if out[2] > out[1] and out[2] > out[0]:
        print "Identified as Triangle\n"

global minimumdist
global maximumdist

if __name__ == "__main__":
    minimumdist = 0.8;
    maximumdist = 8.5;
    # train_net()
    # print(minimumdist)
    # print('\n')
    # print(maximumdist)



