from random import shuffle, randrange
 
def make_maze(w = 18, h = 18):
	char = "%"
	vis = [[0] * w + [1] for _ in range(h)] + [[1] * (w + 1)]
	ver = [[char + " "] * w + [char] for _ in range(h)] + [[]]
	hor = [[char + char] * w + [char] for _ in range(h + 1)]

	def walk(x, y):
		if randrange(10) < 2:
			vis[y][x] = 1

		d = [(x - 1, y), (x, y + 1), (x + 1, y), (x, y - 1)]
		shuffle(d)
		for (xx, yy) in d:
			if vis[yy][xx]: continue
			if xx == x: hor[max(y, yy)][x] = char + " "
			if yy == y: ver[y][max(x, xx)] = "  "
			walk(xx, yy)

	walk(randrange(w), randrange(h))

	ver[17][0] =  "%."
	ver[0][17] =  " ."
	ver[17][17] =  " ."
	ver[0][0] =  "%."
	hor[9][9] = "P "

	s = ""
	for (a, b) in zip(hor, ver):
		s += ''.join(a + ['\n'] + b + ['\n'])
	return s.strip()

if __name__ == '__main__':
	print(make_maze())