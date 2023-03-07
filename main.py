from loadscen import *
from ReplanPPSIPPS import *
import pygame as pg
import threading
import multiprocessing as mp
from concurrent.futures import thread
import time
import copy

def drawCell(surface, x, y, colour, cellSize, width=0):
	pg.draw.rect(surface, colour, (x*cellSize, y*cellSize, cellSize, cellSize), width)

def drawBigCell(surface, x, y, colour, cellSize, width=0):
	pg.draw.rect(surface, colour, (x*cellSize-2, y*cellSize-2, cellSize+4, cellSize+4), width)

def drawPath(path, colour, mapSize, cellSize, width=0):
	pathSurface = pg.Surface(mapSize)
	pathSurface.set_colorkey(pg.Color(0, 0, 0))
	top = path[0][1]
	bottom = path[0][1]
	left = path[0][0]
	right = path[0][0]
	for i in range(1, len(path)-1):
		drawCell(pathSurface, path[i][0], path[i][1], colour, cellSize, width)
		top = min(top, path[i][1])
		bottom = max(bottom, path[i][1])
		left = min(left, path[i][0])
		right = max(right, path[i][0])
	return (pathSurface, top, bottom+1, left, right+1)

def recv_animation_directives(directivesQueue, history, mapSurface, cellSize, pathColours, starts):
	history[0][3] = mapSurface.copy()
	stop = False
	index = 0
	oldCollisions = []
	lastImprovedStatic = None
	while stop == False:
		if index >= len(directivesQueue):
			if directivesQueue[-1] == 'done':
				return
		else:
			directive = directivesQueue[index]
			index+=1
			if directive == 'done':
				stop = True
				break

			currStep = copy.copy(history[-1])
			history.append(currStep)

			if directive[0] == 'initial paths':

				for agentPath in currStep[2]:
					currStep[0][agentPath['agent']] = {
						'path': agentPath['path'], 
						'surface': agentPath['surface']
					}
				currStep[2] = []
				lastImprovedStatic = currStep[3].copy()
				currStep[4] = None
				

			if directive[0] == 'neighbourhood':
				currStep[1] = {'neighbourhood': directive[2], 'ALNSweights': directive[3]}
				#create new static
				newStatic = mapSurface.copy()
				# draw paths outside neighbourhood not including starts and goals
				for i in range(len(currStep[0])):
					if i not in currStep[1]['neighbourhood']:
						temp = currStep[0][i]['surface']
						newStatic.blit(temp[0], (temp[3]*cellSize, temp[1]*cellSize), (temp[3]*cellSize, temp[1]*cellSize, (temp[4]-temp[3])*cellSize, (temp[2]-temp[1])**cellSize))

				oldCollisions = currStep[5]
				#filter collisions without neighbourood paths
				currStep[5] = []
				for collision in oldCollisions:
					if collision[1] not in currStep[1]['neighbourhood'] and collision[2] not in currStep[1]['neighbourhood']:
						currStep[5].append(collision)
				print("outer neighbourhood collisions", len(currStep[5]))
				#draw red boxes around temp collisions

				currStep[3] = newStatic

			if directive[0] == 'pathing for agent':
				#setup Surface to draw isocontour progression from OPEN/CLOSED lists
				exploredSurface = pg.Surface(mapSurface.get_size())
				exploredSurface.set_colorkey(pg.Color(0, 0, 0))
				currStep[4] = [exploredSurface, 0, 0, 0, 0] #top, bottom, left, right
			
			if directive[0] == 'isocost contours': #[1]: [(newOpen, newClosed), ...]
				prevStep = currStep
				for i in range(len(directive[1])):
					nextStep = copy.copy(prevStep)
					#draw new contour step
					contourSurface = nextStep[4][0].copy()
					nextStep[4] = [contourSurface, nextStep[4][1], nextStep[4][2], nextStep[4][3], nextStep[4][4]]
					#draw dark blue CLOSED
					colour = pg.Color((0, 0, 100))
					for loc in directive[1][i][1]: #newClosed
						drawCell(contourSurface, loc[0], loc[1], colour, cellSize)
					#draw light blue OPEN
					colour = pg.Color((0, 0, 200))
					for loc in directive[1][i][0]: #newOpen
						drawCell(contourSurface, loc[0], loc[1], colour, cellSize)
						#check for new area boundary
						if loc[1] < nextStep[4][1]:
							nextStep[4][1] = loc[1]
						if loc[1]+1 > nextStep[4][2]:
							nextStep[4][2] = loc[1]+1
						if loc[0] < nextStep[4][3]:
							nextStep[4][3] = loc[0]
						if loc[0]+1 > nextStep[4][4]:
							nextStep[4][4] = loc[0]+1
					nextStep[4][0] = contourSurface
					history.append(nextStep)
					prevStep = history[-1]

			if directive[0] == 'path done':
				#add to completed paths
				newPath = {'agent': directive[1], 
					'path': directive[2],
					'surface': drawPath(directive[2], pathColours[directive[1]], mapSurface.get_size(), cellSize)
				}
				#copy path references from previous step into new list
				currStep[2] = []
				for completedPath in history[-2][2]:
					currStep[2].append(completedPath)

				currStep[4] = [history[-2][4][0].copy(), history[-2][4][1], history[-2][4][2], history[-2][4][3], history[-2][4][4]]

				#draw new path to static
				colour = pathColours[newPath['agent']]
				temp = newPath['surface']
				newStatic = currStep[3].copy()
				newStatic.blit(temp[0], (temp[3]*cellSize, temp[1]*cellSize), (temp[3]*cellSize, temp[1]*cellSize, (temp[4]-temp[3])*cellSize, (temp[2]-temp[1])**cellSize))
				currStep[3] = newStatic
				#currStep[4][0].blit(temp[0], (temp[3], temp[1]), (temp[3], temp[1], temp[4]-temp[3], temp[2]-temp[1]))
				currStep[4] = None

				#check for collisions and add to collisions and draw to static
				currStep[5] = copy.copy(currStep[5])
				for priorPath in currStep[2]:
					for i in range(min(len(priorPath['path']), len(newPath['path']))):
						if newPath['path'][i] == priorPath['path'][i]:
							collision = (newPath['path'][i], newPath['agent'], priorPath['agent'])
							currStep[5].append(collision)
							drawCell(currStep[3], collision[0][0], collision[0][1], pg.Color((200, 0, 0)), cellSize, 1)

				#add new path to neighbourhood completed paths
				currStep[2].append(newPath)

			if directive[0] == 'improved solution':
				#add new paths to current solution
				newStatic = currStep[3].copy()
				for agentPath in currStep[2]:
					currStep[0][agentPath['agent']] = {
						'path': agentPath['path'], 
						'surface': agentPath['surface']
					}
				currStep[2] = []

				#reset neighbourhood
				currStep[1] = None

				#static carries over new paths and collisions
				#set new lastImprovedStatic
				lastImprovedStatic = currStep[3].copy()

				#reset isocontour surface
				currStep[4] = None

				#add new collisions
				#currStep[5] = newCollisions
				#newCollisions = []
				oldCollisions = currStep[5]
				print("new collision total", len(currStep[5]))

			if directive[0] == 'no improvement':
				#revert to previous solution and static
				#current solution remains unchanged

				#reset neighbourhood
				currStep[1] = None

				#reset completed paths
				currStep[2] = []

				#reset static
				currStep[3] = lastImprovedStatic

				#remove isocontour progression
				currStep[4] = None

				#clear new collisions
				currStep[5] = oldCollisions
				print("old collision total", len(currStep[5]))



if __name__ == '__main__':
	neighbourhoodSize = 4
	numAgents = 20
	scenarioFile = 'Boston_0_256-even-1.scen'
	instanceMap, starts, goals = loadScen(scenarioFile, numAgents)

	#setup animation resources
	pg.init()
	mapHeight = len(instanceMap)
	mapWidth = len(instanceMap[0])
	cellSize = (1080/mapHeight)//1
	pg.display.set_mode((mapWidth*cellSize, mapHeight*cellSize))
	displaySurface = pg.display.get_surface()

	mapSurface = pg.Surface((mapWidth*cellSize, mapHeight*cellSize))
	for i in range(mapWidth):
		for j in range(mapHeight):
			if instanceMap[j][i] == True:
				pg.draw.rect(mapSurface, (220, 220, 220), (i*cellSize, j*cellSize, cellSize, cellSize))
			else:
				pg.draw.rect(mapSurface, (70, 70, 70), (i*cellSize, j*cellSize, cellSize, cellSize))

	agentColours = []
	for i in range(numAgents): #70 - 220 Yellow
		agentColours.append(pg.Color((220-(i/numAgents)*150, 220-(i/numAgents)*150, 0)))

	goalColours = [] 
	for i in range(numAgents): #70 - 220 Green
		goalColours.append(pg.Color((0, 220-(i/numAgents)*150, 0)))

	pathColours = []
	for i in range(numAgents): #70 - 220 purple
		pathColours.append(pg.Color((220-(i/numAgents)*150, 0, 220-(i/numAgents)*150)))

	closedColour = (0, 0, 100)
	openColour = (0, 0, 220)

	#draw initial frame
	for i in range(numAgents):
		drawCell(mapSurface, starts[i][0], starts[i][1], agentColours[i], cellSize)
		#drawCell(mapSurface, starts[i][0], starts[i][1], pg.Color((200, 0, 0)), cellSize, 1)
		drawCell(mapSurface, goals[i][0], goals[i][1], goalColours[i], cellSize)
	displaySurface.blit(mapSurface, (0, 0))
	pg.display.update()


	directivesQueue = []
	executor = thread.ThreadPoolExecutor()
	t1 = executor.submit(LNS2PP, neighbourhoodSize, instanceMap, starts, goals, directivesQueue)

	#current solution, neighbourhood, PP completed agents, static, current agent path exploration, collisions
	eventHistory = [ [ [None]*numAgents, set(list(range(numAgents))), [], mapSurface, None, [] ] ]
	t2 = executor.submit(recv_animation_directives, directivesQueue, eventHistory, mapSurface, cellSize, pathColours, starts)
	executor.shutdown(False)

	def renderFrame(eventHistory, eventHistory_idx):
		if eventHistory_idx < len(eventHistory):
			#draw static
			displaySurface.blit(eventHistory[eventHistory_idx][3], (0, 0))

			#draw current path progress
			if eventHistory[eventHistory_idx][4] != None:
				temp = eventHistory[eventHistory_idx][4]
				top = temp[1]
				left = temp[3]
				width = temp[4] - left
				height = temp[2] - top
				displaySurface.blit(temp[0], (left*cellSize, top*cellSize), 
					(left*cellSize, top*cellSize, width*cellSize, height*cellSize))

			#draw collisions
			red = pg.Color((220, 0, 0))
			if len(eventHistory[eventHistory_idx][5]) > 0:
				#print(eventHistory[eventHistory_idx][5])
				for collision in eventHistory[eventHistory_idx][5]:
					#print(collision)
					drawBigCell(displaySurface, collision[0][0], collision[0][1], red, cellSize)
		pg.display.update()

	running = True
	pause = False
	lastDrawnFrame = -1
	eventHistory_idx = 0
	frameRate = 60
	while running:
		#animate using event history
		time = pg.time.get_ticks()
		currFrame = time * frameRate // 1000
		if currFrame != lastDrawnFrame and pause == False:
			#print(len(directivesQueue), len(eventHistory))
			#draw next frame
			lastDrawnFrame = currFrame
			renderFrame(eventHistory, eventHistory_idx)
			eventHistory_idx = min(eventHistory_idx + 1, len(eventHistory)-1)

		for event in pg.event.get():
			if event.type == pg.QUIT:
				t1.cancel()
				t2.cancel()
				running = False
			if event.type == pg.KEYDOWN:
				if event.key == pg.K_SPACE:
					pause = pause ^ True
				if event.key == pg.K_LEFT:
					eventHistory_idx = max(eventHistory_idx - 1, 0)
					if event.mod & pg.KMOD_LSHIFT:
						eventHistory_idx = max(eventHistory_idx - 9, 0)
					renderFrame(eventHistory, eventHistory_idx)
				if event.key == pg.K_RIGHT:
					eventHistory_idx = min(eventHistory_idx + 1, len(eventHistory)-1)
					if event.mod & pg.KMOD_LSHIFT:
						eventHistory_idx = min(eventHistory_idx + 9, len(eventHistory)-1)
					renderFrame(eventHistory, eventHistory_idx)

