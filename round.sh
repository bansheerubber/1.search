!/bin/sh
while :
do
	echo "Mazed"
	python maze2.py > layouts/$1.lay
	echo "Doing round..."
	python pacman.py -l $1 -z .5 -p SearchAgent -a fn=dfs -q | grep -oE "([0-9]*)" | tail -n 1 >> dfs.stats.2
	python pacman.py -l $1 -z .5 -p SearchAgent -a fn=bfs -q | grep -oE "([0-9]*)" | tail -n 1 >> bfs.stats.2
	python pacman.py -l $1 -z .5 -p SearchAgent -a fn=ucs -q | grep -oE "([0-9]*)" | tail -n 1 >> ucs.stats.2
	python pacman.py -l $1 -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic -q | grep -oE "([0-9]*)" | tail -n 1 >> astar.stats.2
	python pacman.py -l $1 -z .5 -p SearchAgent -a fn=bi,heuristic=manhattanHeuristic -q | grep -oE "([0-9]*)" | tail -n 1 >> bi.stats.2
done