import os
import re
import sys
import time

nodes_extractor = re.compile(r"expanded: ([0-9]+)")
cost_extractor = re.compile(r"cost of ([0-9]+)")
def extract_results(output):
	match1 = nodes_extractor.search(output)
	if match1:
		nodes = match1.group(1)
	else:
		return None

	match2 = cost_extractor.search(output)
	if match2:
		cost = match2.group(1)
	else:
		return None
	return (nodes, cost)

def write_results(result, search, filename):
	if result != None:
		file = open(f"./results/{search}.stats.{filename}", "a")
		file.write(f"{result[0]},{result[1]}\n")
		file.close()

print(f"Using program '{sys.argv[1]}' to generate maze to file './layouts/{sys.argv[2]}.lay', outputting results to './results/*.stats.{sys.argv[3]}'")
time.sleep(1)

corners = False
if "corners" in sys.argv[1]:
	corners = True

for _ in range(0, 2500): # run 10000 times, adjust this if needed
	maze = os.popen(f"python {sys.argv[1]}").read() # randomly generate the maze
	file = open(f"./layouts/{sys.argv[2]}.lay", "w")
	file.write(maze)
	file.close()

	# run the searches
	if not corners:
		dfs = extract_results(os.popen(f"python pacman.py -l {sys.argv[2]} -z .5 -p SearchAgent -a fn=dfs -q").read())
		bfs = extract_results(os.popen(f"python pacman.py -l {sys.argv[2]} -z .5 -p SearchAgent -a fn=bfs -q").read())
		ucs = extract_results(os.popen(f"python pacman.py -l {sys.argv[2]} -z .5 -p SearchAgent -a fn=ucs -q").read())
		astar = extract_results(os.popen(f"python pacman.py -l {sys.argv[2]} -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic -q").read())
		bi = extract_results(os.popen(f"python pacman.py -l {sys.argv[2]} -z .5 -p SearchAgent -a fn=bi,heuristic=manhattanHeuristic -q").read())
	else:
		dfs = extract_results(os.popen(f"python pacman.py -l {sys.argv[2]} -z .5 -p SearchAgent -a fn=dfs,prob=CornersProblem -q").read())
		bfs = extract_results(os.popen(f"python pacman.py -l {sys.argv[2]} -z .5 -p SearchAgent -a fn=bfs,prob=CornersProblem -q").read())
		ucs = extract_results(os.popen(f"python pacman.py -l {sys.argv[2]} -z .5 -p SearchAgent -a fn=ucs,prob=CornersProblem -q").read())
		astar = extract_results(os.popen(f"python pacman.py -l {sys.argv[2]} -z .5 -p AStarCornersAgent -q").read())
		bi = extract_results(os.popen(f"python pacman.py -l {sys.argv[2]} -z .5 -p BiDirectionalCornersAgent -q").read())

	print(dfs, bfs, ucs, astar, bi)

	# write the results
	write_results(dfs, "dfs", sys.argv[3])
	write_results(bfs, "bfs", sys.argv[3])
	write_results(ucs, "ucs", sys.argv[3])
	write_results(astar, "astar", sys.argv[3])
	write_results(bi, "bi", sys.argv[3])