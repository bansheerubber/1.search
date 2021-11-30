import os
import re
import sys
import time

nodes_extractor = re.compile(r"expanded: ([0-9]+)")
cost_extractor = re.compile(r"cost of ([0-9]+)")
def extract_results(output):
	nodes = nodes_extractor.search(output).group(1)
	cost = cost_extractor.search(output).group(1)
	return (nodes, cost)

def write_results(result, search, filename):
	file = open(f"./results/{search}.stats.{filename}", "a")
	file.write(f"{result[0]},{result[1]}\n")
	file.close()

print(f"Using program '{sys.argv[1]}' to generate maze to file './layouts/{sys.argv[2]}.lay', outputting results to './results/*.stats.{sys.argv[3]}'")
time.sleep(1)

for _ in range(0, 10000): # run 10000 times, adjust this if needed
	maze = os.popen(f"python {sys.argv[1]}").read() # randomly generate the maze
	file = open(f"./layouts/{sys.argv[2]}.lay", "w")
	file.write(maze)
	file.close()

	# run the searches
	dfs = extract_results(os.popen(f"python pacman.py -l {sys.argv[2]} -z .5 -p SearchAgent -a fn=dfs -q").read())
	bfs = extract_results(os.popen(f"python pacman.py -l {sys.argv[2]} -z .5 -p SearchAgent -a fn=bfs -q").read())
	ucs = extract_results(os.popen(f"python pacman.py -l {sys.argv[2]} -z .5 -p SearchAgent -a fn=ucs -q").read())
	astar = extract_results(os.popen(f"python pacman.py -l {sys.argv[2]} -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic -q").read())
	bi = extract_results(os.popen(f"python pacman.py -l {sys.argv[2]} -z .5 -p SearchAgent -a fn=bi,heuristic=manhattanHeuristic -q").read())

	print(dfs, bfs, ucs, astar, bi)

	# write the results
	write_results(dfs, "dfs", sys.argv[3])
	write_results(bfs, "bfs", sys.argv[3])
	write_results(ucs, "ucs", sys.argv[3])
	write_results(astar, "astar", sys.argv[3])
	write_results(bi, "bi", sys.argv[3])