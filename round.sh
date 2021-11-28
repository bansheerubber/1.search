#!/bin/sh
while :
do
	echo "Mazed"
	python $1.py > layouts/$2.lay
	echo "Doing round..."
	# python pacman.py -l $2 -z .5 -p SearchAgent -a fn=dfs -q | grep -oE "([0-9]*)" | tail -n 1 >> dfs.stats.$1
	# python pacman.py -l $2 -z .5 -p SearchAgent -a fn=bfs -q | grep -oE "([0-9]*)" | tail -n 1 >> bfs.stats.$1
	# python pacman.py -l $2 -z .5 -p SearchAgent -a fn=ucs -q | grep -oE "([0-9]*)" | tail -n 1 >> ucs.stats.$1
	# python pacman.py -l $2 -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic -q | grep -oE "([0-9]*)" | tail -n 1 >> astar.stats.$1
	# python pacman.py -l $2 -z .5 -p SearchAgent -a fn=bi,heuristic=manhattanHeuristic -q | grep -oE "([0-9]*)" | tail -n 1 >> bi.stats.$1
	
	result=$(python pacman.py -l $2 -z .5 -p SearchAgent -a fn=dfs,prob=CornersProblem -q | grep -oE "([0-9]*)")
	[ ! -z "$result" ] && echo "$(echo $result | awk '{print $4}'),$(echo $result | awk '{print $1}')" >> ./results/dfs.stats.$1

	result=$(python pacman.py -l $2 -z .5 -p SearchAgent -a fn=bfs,prob=CornersProblem -q | grep -oE "([0-9]*)")
	[ ! -z "$result" ] && echo "$(echo $result | awk '{print $4}'),$(echo $result | awk '{print $1}')" >> ./results/bfs.stats.$1

	result=$(python pacman.py -l $2 -z .5 -p SearchAgent -a fn=ucs,prob=CornersProblem -q | grep -oE "([0-9]*)")
	[ ! -z "$result" ] && echo "$(echo $result | awk '{print $4}'),$(echo $result | awk '{print $1}')" >> ./results/ucs.stats.$1

	result=$(python pacman.py -l $2 -z .5 -p AStarCornersAgent -q | grep -oE "([0-9]*)")
	[ ! -z "$result" ] && echo "$(echo $result | awk '{print $4}'),$(echo $result | awk '{print $1}')" >> ./results/astar.stats.$1
	
	result=$(python pacman.py -l $2 -z .5 -p BiDirectionalCornersAgent -q | grep -oE "([0-9]*)")
	[ ! -z "$result" ] && echo "$(echo $result | awk '{print $4}'),$(echo $result | awk '{print $1}')" >> ./results/bi.stats.$1
done