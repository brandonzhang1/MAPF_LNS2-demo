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


#initial paths: move completed agents to current solution
#improved solution: move completed agents to current solution
#no improvement: do nothing

#neighbourhood: create new static for outside agents (draw outline on agents)
#pathing for agent: note agent number 
#isocost contour: create new frontier Surface from previous step
#path done: (draw path), add to PP completed agents, create new static from neighbourhood static and PP completed agents

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
	newCollisions = []
	lastImprovedStatic = None
	while stop == False:
		if index >= len(directivesQueue):
			if directivesQueue[-1] == 'done':
				return
			#print('debug2')
			#time.sleep(0.2)
		else:
			#print('debug')
			directive = directivesQueue[index]
			index+=1
			if directive == 'done':
				#draw last frame with path solutions
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
				newCollisions = []
				currStep[1] = {'neighbourhood': directive[2], 'ALNSweights': directive[3]}
				#create new static
				newStatic = mapSurface.copy()
				# draw paths outside neighbourhood not including starts and goals
				for i in range(len(currStep[0])):
					if i not in currStep[1]['neighbourhood']:
						temp = currStep[0][i]['surface']
						newStatic.blit(temp[0], (temp[3]*cellSize, temp[1]*cellSize), (temp[3]*cellSize, temp[1]*cellSize, (temp[4]-temp[3])*cellSize, (temp[2]-temp[1])**cellSize))
				#draw purple boxes around neighbourhood agents
				colour = pg.Color((200, 0, 200))
				for agent in currStep[1]['neighbourhood']:
					drawCell(newStatic, starts[agent][0], starts[agent][1], colour, cellSize, 1)

				#filter collisions without neighbourood paths
				for collision in currStep[5]:
					if collision[1] not in currStep[1]['neighbourhood'] and collision[2] not in currStep[1]['neighbourhood']:
						newCollisions.append(collision)
				#draw red boxes around temp collisions
				colour = pg.Color((200, 0, 0))
				for collision in newCollisions:
					drawCell(newStatic, collision[0][0], collision[0][1], colour, cellSize, 1)

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
						if loc[1] > nextStep[4][2]:
							nextStep[4][2] = loc[1]
						if loc[0] < nextStep[4][3]:
							nextStep[4][3] = loc[0]
						if loc[0] > nextStep[4][4]:
							nextStep[4][4] = loc[0]
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
				for priorPath in currStep[2]:
					for i in range(min(len(priorPath['path']), len(newPath['path']))):
						if newPath['path'][i] == priorPath['path'][i]:
							collision = (newPath['path'][i], newPath['agent'], priorPath['agent'])
							newCollisions.append(collision)
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
				currStep[5] = newCollisions
				newCollisions = []

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
				newCollisions = []




if __name__ == '__main__':
	neighbourhoodSize = 4
	numAgents = 7
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


	executor = thread.ThreadPoolExecutor()

	#setup parallel processes and pipes
	#parentPipe, childPipe = mp.Pipe()
	directivesQueue = []
	executor.submit(LNS2PP, neighbourhoodSize, instanceMap, starts, goals, directivesQueue)

	#todos
	#for solution step animation directives, aggregate in steppable list using seperate thread to allow input commands
	#recieve animation directives for each distinct cost level of SIPPS (integrate pipe to SIPPS.py)
	#	post new slice of closed list, post new open list

	#animation directives
	# selecting neighbourhood: selected strategy, updated probabilities
	# greying outside agent paths & clearing selected agent paths
	# SIPPS isocost steps for agent
	# soft collision detected colouring
	# path found colouring
	# solution not found, return to previous replanning paths


	#for pathfinding isocost contours, can use a common static of map + outside and previous agents
	# between steps for an agent's pathing, copy Surface of previous frontier on transparent background and add new cells

	#general: when to compose new static
	#neighbourhood selection: map + outside agents
	#during prioritized planning, map + outside agents + prior agents
	#create a new static for each sequence of frames, re-reference
	#to create new static, maintain: map, current solution, neighbourhood, PP completed agents

	#initial paths: move completed agents to current solution
	#improved solution: move completed agents to current solution
	#no improvement: do nothing

	#neighbourhood: create new static for outside agents (draw outline on agents)
	#pathing for agent: note agent number 
	#isocost contour: create new frontier Surface from previous step
	#path done: (draw path), add to PP completed agents, create new static from neighbourhood static and PP completed agents

	#current solution, neighbourhood, PP completed agents, static, current agent path exploration, collisions
	eventHistory = [ [ [None]*numAgents, set(list(range(numAgents))), [], mapSurface, None, [] ] ]
	executor.submit(recv_animation_directives, directivesQueue, eventHistory, mapSurface, cellSize, pathColours, starts)
	executor.shutdown(False)


	def renderFrame(eventHistory, eventHistory_idx):
		if eventHistory_idx < len(eventHistory):
			#draw static
			displaySurface.blit(eventHistory[eventHistory_idx][3], (0, 0))
			'''
			#draw completed paths
			temp = eventHistory[eventHistory_idx][2]
			for agentPath in temp:
				for i in range(1, len(agentPath['path'])-1):
					drawCell(displaySurface, agentPath['path'][i][0], agentPath['path'][i][1], pathColours[agentPath['agent']], cellSize)
			'''

			#drawn current path progress
			if eventHistory[eventHistory_idx][4] != None:
				temp = eventHistory[eventHistory_idx][4]
				top = temp[1]
				left = temp[3]
				width = temp[4] - left
				height = temp[2] - top
				displaySurface.blit(temp[0], (left*cellSize, top*cellSize), 
					(left*cellSize, top*cellSize, width*cellSize, height*cellSize))
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

