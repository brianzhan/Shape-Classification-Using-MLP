import math
import numpy as np
from scipy import interpolate

#filter the data by using a moving average
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
#		  nodes: number of nodes in the network
#		  normmax and normmin: what to normalize dataset by, currently 8.5 based on max squares and 0.8 based on min circles
#output - rn (radius_new) an array of distances that are equally 
#spaced based on the data and the number of nodes
def data_to_array(finput,nodes,normmax, normmin):
	known = 13 #distance from sensor to center of platform (cm)
	T = np.loadtxt(finput)
	T = remove_zeros(T)
	row = (T.size/2) - 1
	vel = (2*math.pi)/T[row,1]
	dist = np.exp((T[:,0] - 4491.68)/(-1223.778))
	r = known - dist
	rs = moving_average(r)
	rs = np.append(r[0],rs)
	rs = np.append(rs, r[-1])

	omega = T[:,1]*vel
	omegan = np.linspace(0.03,2*math.pi,num = nodes)

	f = interpolate.interp1d(omega,rs)
	rn = f(omegan)
	rn = (rn-normmin)/normmax

	#look at data by uncommenting below, omega by distance graph
	save = np.column_stack((omegan,rn))
	np.savetxt("save.txt",save) 
	return rn

neural_input = data_to_array("20",20,8.5,0.8)
print neural_input
