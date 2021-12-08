import matplotlib.pyplot as plt
import sys

def read(filename):
	file = open(filename)
	result = []
	for line in file:
		split = line.split(",")
		try:
			result.append(int(split[0]))
		except:
			pass
	plt.hist(result, bins="auto")
	plt.show()

read(sys.argv[1])
	
