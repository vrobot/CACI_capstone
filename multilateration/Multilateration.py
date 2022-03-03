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

def LST(nodes, times, v):

	def equations(guess):
			x, y, r = guess

			output = []

			for i, n in enumerate(nodes):
				x1 = n[0]
				y1 = n[1]
				dist1 = v * times[i]
				output.append((x - x1)**2 + (y - y1)**2 - (dist1 - r)**2)

			return tuple(output)

	mean_x = np.mean([row[0] for row in nodes])
	mean_y = np.mean([row[1] for row in nodes])

	inital_guess = (mean_x, mean_y, 0)

	final_guess = least_squares(equations, inital_guess)
	print(final_guess)

	return [final_guess.x[0], final_guess.x[1]]

def noisy_LST(nodes, times, v, error_scale):
	#convert the scale of an error to its absolute value
	error_abs = error_scale * find_farthest_points(nodes)

	# create a normal distribution based on the error scale and apply it to each node's timestamp
	for i, t in enumerate(times):
		print('before: ', times[i])
		times[i] = t + np.random.normal(0, error_abs)
		print('after:  ', times[i])

	output = LST(nodes, times, v)
	return output

def test_LST(iterations, nodes, display, error_scale):
	
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
	checker_err = find_farthest_points(nodes) * error_scale * 2

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
		plt.show()

	# return proportion of correct guesses
	return (iterations - incorrect_guesses) / iterations

# change variables here
speed_of_sound = 1

# edit the locations of each microphone
# format: [[x1, y1], [x2, y2], [x3, y3] ... [xn, yn]]
node_list = [[0, 1], [8, -3], [-1, 5], [3, 7], [-3, -4]]

# set number of points to check
iterations = 100

# set whether or not a graph of outputs is shown
display_results = True

# set the size of noise error with regard to maximum distance between nodes
error_scale = 0.01

#display the accuracy of the test
accuracy = test_LST(iterations, node_list, display_results, error_scale)

print("Accuracy: ", (accuracy*100), '%')

print('done!')