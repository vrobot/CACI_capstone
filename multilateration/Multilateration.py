import numpy as np
import sympy as sp
import scipy
import matplotlib.pyplot as plt
import random
import math

from itertools import combinations

from sympy import Eq, solve_linear_system, Matrix
from sympy.interactive import printing

from scipy.optimize import least_squares

printing.init_printing(use_unicode=True)

# Random Seed
random.seed(69)

#nodes is formatted as follows: [[x1, y1], [x2, y2], [x3, y3] ... [xn, yn]]
def test_MLZ(iterations, nodes, display):
	
	# speed of sound
	s_o_s = 1

	#calculate the middle of the triangle
	mean_x = np.mean([row[0] for row in nodes])
	mean_y = np.mean([row[1] for row in nodes])

	# calculate the standard deviation from the center of the triangle
	std_x = np.std([row[0] for row in nodes])
	std_y = np.std([row[1] for row in nodes])

	# generate a guassian distro of guesses
	s_x = np.random.normal(mean_x, std_x, iterations)
	s_y = np.random.normal(mean_y, std_y, iterations)

	output = []
	incorrect_guesses = 0

	# loop through each sonic source
	for i in range(iterations):

		times = calcTimes(s_x[i], s_y[i], nodes, s_o_s) 
		
		# see if an error get generated
		try:
			#output.append(MLZ(nodes, times, s_o_s))
			output.append(noisy_MLZ(nodes, times, s_o_s, 0.0000000001))
			#output.append(noisy_TRI(nodes, times, s_o_s, 0.0001))
			print("completed equation ", (i+1), " of ", iterations)

		# increment wrong guesses if an error occurs
		except:
			output.append([float("NaN"), float("NaN")])
			print("could not solve equation ", (i+1), " of ", iterations)

	# for each prediction, check if the output is correct
	for i, check in enumerate(output):
		
		correct_show = True

		#check for wrong guesses
		if not (math.isclose(s_x[i], check[0], rel_tol=0.0, abs_tol= 0.001) & math.isclose(s_y[i], check[1], rel_tol=0.0, abs_tol= 0.001)):
			
			print("incorrect guess:")
			print("real : ", s_x[i], ", ", s_y[i])
			print("guess: ", check[0], ", ", check[1])

			correct_show = False

			incorrect_guesses += 1

		# display the guesses and sound sources
		if display == True & correct_show == True:
			
			#plot guess
			plt.scatter(check[0], check[1], color = 'green', marker='X')

		else:
			#plot wrong guess
			plt.scatter(check[0], check[1], color = 'purple', marker='x')
			#plot real thing
			plt.scatter(s_x[i], s_y[i], color = 'red', marker='o')

	#show the nodes if the display setting is on
	if display == True:
		#display listening nodes
		for i, point in enumerate(nodes):
			mark = '$' + str(i) + '$'
			plt.scatter(point[0], point[1], color = 'black', marker=mark)
		
		plt.show()

	# return proportion of correct guesses
	return (iterations - incorrect_guesses) / iterations


# returns predictions about the source of the sonic event  
# nodes is formatted as follows: [[x1, y1], [x2, y2], [x3, y3] ... [xn, yn]]
# times is formatted as follows: [t1, t2, t3 ... tn]
def MLZ(nodes, times, v):

	x,y = sp.symbols('x y', real=True)

	eqs = []

	# loop through each node and each node that comes behind it in the order to get every 2 point combination as an equation
	for i, point1 in enumerate(nodes[:-1]):

		# loop through all nodes after the current node to combine them for equations
		for j, point2 in enumerate(nodes[(i+1):]):

			# the value that will be used to reference the second point for some purposes
			j_temp = i + j + 1;

			# name the equation in Sympy, tbh idk why you have to do this but it breaks if you don't
			eq_name = 'eq-' + str(i) + '-' + str(j_temp)
			func = sp.Function(eq_name)

			# equation to be solved
			func = Eq( (1/v) * ( ( (x - point1[0])**2 + (y - point1[1])**2 )**0.5 - ( (x - point2[0])**2 + (y - point2[1])**2 )**0.5 ) + times[j_temp], times[i])

			# equation to the list of equations
			eqs.append(func)

			#print the equation just appended to the list of equations
			print('Equation #', str(i), '-', str(j_temp), ': ', func)

	#print('equations:', eqs)

	# solve numerically
	result_numeric = sp.nsolve(eqs, (x, y), (0, 0))
	print('Numeric Predictions: ', result_numeric)

	#solve symbolically
	#result_symbolic = sp.solve(eqs, (x, y))
	#print('Symbolic Predictions: ', result_symbolic)

	return result_numeric #, result_symbolic  <-------------------------- uncomment for symbolic solutions

# returns predictions about the source of the sonic event with error specified 
# nodes is formatted as follows: [[x1, y1], [x2, y2], [x3, y3] ... [xn, yn]]
# times is formatted as follows: [t1, t2, t3 ... tn]
def noisy_MLZ(nodes, times, v, error_scale):
	#convert the scale of an error to its absolute value
	error_abs = error_scale * find_farthest_points(nodes)

	# create a normal distribution based on the error scale and apply it to each node's timestamp
	for i, t in enumerate(times):
		print('before: ', times[i])
		times[i] = t + np.random.normal(0, error_abs)
		print('after:  ', times[i])


	output = MLZ(nodes, times, v)
	return output

# helper function to calcualte times that sound will arrive at each node for testing
# nodes is formatted as follows: [[x1, y1], [x2, y2], [x3, y3] ... [xn, yn]]
def calcTimes (Sx, Sy, nodes, v):
	times = []

	for point in nodes:
		times.append((((Sx - point[0])**2 + (Sy - point[1])**2 )**0.5 ) / v)

	return times


def find_farthest_points(points):
	
	distances = []

	for combo in combinations(points, 2):  # 2 for pairs
		distances.append(math.dist(combo[0], combo[1]))
	
	return max(distances)

# returns predictions about the source of the sonic event  
# nodes is formatted as follows: [[x1, y1], [x2, y2], [x3, y3] ... [xn, yn]]
# times is formatted as follows: [t1, t2, t3 ... tn]
def LST(nodes, times, v):

	#print(nodes)
	#print(times)

	#print(np.array(nodes))
	#print(np.array([times]).T)

	nodes = np.concatenate((np.array(nodes), np.array([times]).T), axis=1).tolist()
	#nodes = zip(nodes,times)
	print(nodes)
	def equations(guess):
			x, y, r = guess

			output = []
			
			#print(nodes)
			for combo in combinations(nodes, 2):
				#print('COMBO: ', combo)
				n1 = combo[0]
				n2 = combo[1]
				x1 = n1[0]
				y1 = n1[1]
				x2 = n2[0]
				y2 = n2[1]
				tdoa = n1[2] - n2[2]
				ddoa = v * tdoa
				output.append((math.sqrt((x - x1)**2 + (y - y1)**2) - math.sqrt((x - x2)**2 + (y - y2)**2)) - (ddoa - r))

			return tuple(output)

	mean_x = np.mean([row[0] for row in nodes])
	mean_y = np.mean([row[1] for row in nodes])

	inital_guess = (mean_x, mean_y, 0)

	final_guess = least_squares(equations, inital_guess)
	print(final_guess)

	return [final_guess.x[0], final_guess.x[1]]

#convert list of distances to a triangle matrix of their tdoas
#

def dist2tdoa(dist_list):
	tdoa_list = 0*[len(dist_list)][len(dist_list)]

	for i, d1 in enumerate(dist_list):
		for j, d2 in enumerate(dist_list[i+1:]):
			tdoa_list[i][j] = d1 - d2

	return tdoa_list

def noisy_LST(nodes, times, v, error_scale):
	#convert the scale of an error to its absolute value
	error_abs = error_scale * find_farthest_points(nodes) / v
	

	# create a normal distribution based on the error scale and apply it to each node's timestamp
	for i, t in enumerate(times):
		#print('before: ', times[i])
		times[i] = t + np.random.normal(0, error_abs)
		#print('after:  ', times[i])

	output = LST(nodes, times, v)
	return output


def variable_noisy_LST(nodes, times, v, error_scale):
	#convert the scale of an error to its absolute value
	ffp = find_farthest_points(nodes) / v

	# create a normal distribution based on the error scale and apply it to each node's timestamp
	for i, t in enumerate(times):
		print('before: ', times[i])
		error_abs = error_scale[i] * ffp
		times[i] = t + np.random.normal(0, error_abs)
		print('after:  ', times[i])

	output = LST(nodes, times, v)
	return output


#nodes is formatted as follows: [[x1, y1], [x2, y2], [x3, y3] ... [xn, yn]]
def test_LST(iterations, nodes, display, error_scale, acceptable_error):
	
	# speed of sound
	s_o_s = 1

	#calculate the middle of the triangle
	mean_x = np.mean([row[0] for row in nodes])
	mean_y = np.mean([row[1] for row in nodes])

	# calculate the standard deviation from the center of the triangle
	std_x = np.std([row[0] for row in nodes])
	std_y = np.std([row[1] for row in nodes])

	# generate a guassian distro of guesses
	s_x = np.random.normal(mean_x, std_x, iterations)
	s_y = np.random.normal(mean_y, std_y, iterations)

	# find the farthest nodes from each other and their distance
	checker_err = find_farthest_points(nodes) * acceptable_error

	output = []
	incorrect_guesses = 0

	# loop through each sonic source
	for i in range(iterations):

		times = calcTimes(s_x[i], s_y[i], nodes, s_o_s) 

		# see if an error get generated
		#output.append(LST(nodes, times, s_o_s))
		output.append(noisy_LST(nodes, times, s_o_s, error_scale))

		print("completed equation ", (i+1), " of ", iterations)

	# for each prediction, check if the output is correct
	for i, check in enumerate(output):
		
		correct_show = True

		#check for wrong guesses
		if not (math.isclose(s_x[i], check[0], rel_tol=0.0, abs_tol= checker_err) & math.isclose(s_y[i], check[1], rel_tol=0.0, abs_tol= checker_err)):
			
			print("incorrect guess:")
			print("real : ", s_x[i], ", ", s_y[i])
			print("guess: ", check[0], ", ", check[1])

			correct_show = False

			incorrect_guesses += 1

		#if the display graph setting is on then add current node to plot of guesses and answers
		if(display == True):
			#plot the real location of the sound
			plt.scatter(s_x[i], s_y[i], color = 'grey', marker='o')

			# display the guesses and sound sources
			if correct_show == True:
				#plot right guess
				plt.scatter(check[0], check[1], color = 'green', marker='x')

			else:
				#plot wrong guess
				plt.scatter(check[0], check[1], color = 'red', marker='x')


			#display listening nodes
			for i, point in enumerate(nodes):
				mark = '$' + str(i) + '$'
				plt.scatter(point[0], point[1], color = 'black', marker=mark)
		
		#show the whole plot
	if(display == True):
		plt.xlim([-10, 10])
		plt.ylim([-10, 10])
		plt.show()

	# return proportion of correct guesses
	return (iterations - incorrect_guesses) / iterations


def test1():
	# change variables here
	speed_of_sound = 1

	# edit the locations of each microphone
	# format: [[x1, y1], [x2, y2], [x3, y3] ... [xn, yn]]
	node_list = [[0, 1], [6, -3], [-6, 5], [3, 7], [-3, -5]]

	# set number of points to check
	iterations = 10

	# set whether or not a graph of outputs is shown
	display_results = True

	# set the size of noise error with regard to maximum distance between nodes
	error_scale = 0.015

	acceptable_error = 0.02

	#display the accuracy of the test
	accuracy = test_LST(iterations, node_list, display_results, error_scale, acceptable_error)

	print("Accuracy: ", (accuracy*100), '%')

	print('done!')

def test2():
	# change variables here
	speed_of_sound = 343

	# true source of sound
	true_sound_list = [[0, 0], [15, 0], [7.5, 0], [0, 0], [10, 0]]
	tdoa_list = []


	# edit the locations of each microphone
	# format: [[x1, y1], [x2, y2], [x3, y3] ... [xn, yn]]
	#fake_node = [5, 8]
	fake_node = [0, 15]
	node_list_list = [
		[[0, 0], [15, 0], fake_node], 
		[[0, 0], [15, 0], fake_node], 
		[[0, 0], [15, 0], fake_node], 
		[[0, 0], [20, 0], fake_node], 
		[[0, 0], [10, 0], fake_node]
	]

	#spoof a node
	fake_node_true_time = []
	for t in true_sound_list:
		d = math.sqrt((t[0] - fake_node[0])**2 + (t[1] - fake_node[1])**2)
		fake_time = d/speed_of_sound
		fake_node_true_time.append(fake_time)

	print(fake_node_true_time[0])
	# time of arrival list
	# format: [[a1, a2, a3], [b1, b2, b3], [c1, c2, c3], ]
	
	toa_list = [
		[0, 0.05230, fake_node_true_time[0]],
		[0.04177, 0, fake_node_true_time[1]],
		[0, 0.00030, fake_node_true_time[2]],
		[0, 0.06565, fake_node_true_time[3]],
		[0.02614, 0, fake_node_true_time[4]]
	]

	# set number of points to check
	iterations = 100

	# set whether or not a graph of outputs is shown
	display_results = True

	# set the size of noise error with regard to maximum distance between nodes
	error_scale_list = [0, 0, 0.00]



	for i, node_list in enumerate(node_list_list):
		pred = variable_noisy_LST(node_list, toa_list[i], speed_of_sound, error_scale_list)
		
		plt.scatter(node_list[0][0], node_list[0][1], color = 'black', marker='$A$')
		plt.scatter(node_list[1][0], node_list[1][1], color = 'black', marker='$B$')
		plt.scatter(node_list[2][0], node_list[2][1], color = 'black', marker='$C$')

		plt.scatter(true_sound_list[i][0], true_sound_list[i][1], color = 'red', marker='o')
		plt.scatter(pred[0], pred[1], color = 'green', marker='x')
		plt.show()
		print('prediction of sample ', (i + 1), ': ', pred)



# run tests

test1()