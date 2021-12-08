### CSE571 Team Project: Bi-directional Search in Pacman Domain
by Jacob Watson, Steve Lin, Zeel Prajapati, Prerak Shah

---

Our team implemented Bi-directional search in the Pacman domain. To run,
```
# python pacman.py -l bigMaze -z 0.5 -p SearchAgent -a fn=bi,heuristic=manhattanHeuristic
```
![](https://bansheerubber.com/i/f/TVEn5.png)  

---
We included several random level generators (based on Rosetta code implementations) to test Pacman against. To generate the mazes:
```
# python big-maze.py > ./layouts/[output name].lay
# python sparse-maze.py > ./layouts/[output name].lay
# python big-corners-maze-generator.py > ./layouts/[output name].lay
# python big-corners-maze-generator-sparse.py > ./layouts/[output name].lay
```
You can then run Pacman against the layouts by replacing the `-l` argument's value with the generated `.lay` filename. For the corner maze, a different command is needed to run Pacman:
```
# python pacman.py -l bigCorners -z 0.5 -p BiDirectionalCornersAgent
```
---
To compute statistics, we tested the searches thousands of times on randomly generated mazes. To automate the testing, run:
```
# python round.py [maze generator .py] [.lay filename] [results filename]
```

For example,
```
# python round.py big-maze.py r1 b-m
```
will generate a maze using `big-maze.py`, write the maze to `./layouts/r1.lay`, run all searches on it writing their results to `./results/[search type].stats.b-m`, then rinse and repeat.

Results will be written to the `./results` directory. Each line is a sample from a random maze, with nodes expanded as the first comma-delimited number and path cost as the second. You can run several instances of this program at once to speed up testing if the `.lay` filename is different between them.

---
To plot a histogram of a results file:
```
# python histogram.py ./results/[results filename]
```

For example,
```
# python histogram.py ./results/bi.stats.big-corners-maze-generator
```
![](https://bansheerubber.com/i/f/Tn5fz.png)

---
The results files used for our report are also included in this repository. To run ANOVA on the results, run:
```
# python anova_test.py
```