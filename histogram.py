import matplotlib.pyplot as plt
import sys

def read(filename):
	file = open(filename)
	result = []
	for line in file:
		try:
			result.append(int(line))
		except:
			pass
	plt.hist(result, bins="auto")
	plt.show()

read(sys.argv[1])
	