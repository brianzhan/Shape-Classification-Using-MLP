# Shape-Classification-Using-MLP
Robust shape classification using multilayer neural network. Allows robot to scan a shape on a rotating panel, and classify the shape type

A picture of a sample of shapes used to train the neural network can be found here: https://www.dropbox.com/s/00ytrj5pzodflij/2015-12-01%2017.40.36.jpg?dl=0

A picture of the robot design can be found here:
https://www.dropbox.com/s/q8c21u78slj45i4/2015-12-03%2014.06.25.jpg?dl=0

The src directory contains main.py, neuralnet.py, the subdirectory data, and the subdirectory saved:

main.py:  Used to run the robot.  The user is required to choose between the two modes (1 is for training the robot on a new shape, 2 is for detecting the shape type). 

neuralnet.py: The MLP Artificial Neural Network. Prior to passing in analog signals obtained by scanning the shape with a DMS sensor, signal smoothing is used to reduce noise and reduce number of inputs in each input neuron. Network configurations: Tanh activation function; learning rate of 0.5; momentum of 0.1; 1 hidden layer; 7 hidden neurons in the hidden layer 

Data subredirectory: A sample of distance analog values obtained by scanning shapes. The data in this subdirectory is used to test the ability of the code to accurately detect shapes.

saved subdirectory: Contains the weights of the robot obtained through training it. Because of experimentation on the effect of the size of the training set, the "saved" folder contains weights obtained by training the robot across different numbers of training sets. For accurate results, weights from the "full" subdirectory should be used.


