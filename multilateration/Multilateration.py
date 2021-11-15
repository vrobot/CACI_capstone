import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
import random
import math

from sympy import Eq, solve_linear_system, Matrix
from sympy.interactive import printing
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
			output.append(MLZ(nodes, times, s_o_s))
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

		# display the guesses and sound sources, display the 
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



# helper function to calcualte times that sound will arrive at each node for testing
# nodes is formatted as follows: [[x1, y1], [x2, y2], [x3, y3] ... [xn, yn]]
def calcTimes (Sx, Sy, nodes, v):
	times = []

	for point in nodes:
		times.append((((Sx - point[0])**2 + (Sy - point[1])**2 )**0.5 ) / v)

	return times


'''
nodeA_time, nodeB_time, nodeC_time = calcTimes(source_x, source_y, nodeA_x, nodeA_y, nodeB_x, nodeB_y, nodeC_x, nodeC_y, speed_of_sound) 

guess_n, guess_s = MLZ(nodeA_x, nodeA_y, nodeA_time,    nodeB_x, nodeB_y, nodeB_time,    nodeC_x, nodeC_y, nodeC_time,    speed_of_sound)


# plot all nodes, the sound source, and each prediciton
# Black dots are microphones, red Dots are sonic origin points and blue x's are predictions
plt.scatter(nodeA_x, nodeA_y, color = 'black')
plt.scatter(nodeB_x, nodeB_y, color = 'black', marker='^')
plt.scatter(nodeC_x, nodeC_y, color = 'black', marker='s')
plt.scatter(source_x, source_y, color = 'red', marker='o')
for i in guess_n:
	plt.scatter(guess_n[0], guess_n[1], color = 'blue', marker='x')

for i in range(len(guess_s)):
	plt.scatter(guess_s[i][0], guess_s[i][1], color = 'green', marker='+')

plt.show()
'''

# change variables here
speed_of_sound = 1

# edit the locations of each microphone
# format: [[x1, y1], [x2, y2], [x3, y3] ... [xn, yn]]
node_list = [[0, 1], [8, -3], [-1, 5], [3, 7], [-3, -4]]

# set number of points to check
iterations = 100

# set whether or not a graph of outputs is shown
display_results = True

#display the accuracy of the test
accuracy = test_MLZ(iterations, node_list, display_results)
print("Accuracy: ", (accuracy*100), '%')

print('done!')
