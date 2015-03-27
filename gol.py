import os
import time
import random

def iterate(Z):
	shape = len(Z), len(Z[0])
	N  = [[0,]*(shape[0]+2)  for i in range(shape[1]+2)]
	# Compute number of neighbours for each cell
	for x in range(1,shape[0]-1):
		for y in range(1,shape[1]-1):
			N[x][y] = Z[x-1][y-1]+Z[x][y-1]+Z[x+1][y-1] \
					+ Z[x-1][y]            +Z[x+1][y]   \
					+ Z[x-1][y+1]+Z[x][y+1]+Z[x+1][y+1]
	# Update cells
	for x in range(1,shape[0]-1):
		for y in range(1,shape[1]-1):
			if Z[x][y] == 0 and N[x][y] == 3:
				Z[x][y] = 1
			elif Z[x][y] == 1 and not N[x][y] in [2,3]:
				Z[x][y] = 0
	return Z

def display(Z):
	shape = len(Z), len(Z[0])
	for x in range(1,shape[0]-1):
		for y in range(1,shape[1]-1):
			print Z[x][y],
		print
	print

def generate(x,y):
	for x in range (0,x+2):
		Z.append([])
		for y in range(0,y+2):
			Z[x].append(random.randint(0,1))

Z = []
generate(8,8)
while True:
	os.system('clear')
	display(Z)
	iterate(Z)
	time.sleep(0.3)
