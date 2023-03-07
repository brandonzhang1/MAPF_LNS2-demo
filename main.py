from loadscen import *
from ReplanPPSIPPS import *
import pygame as pg
import threading
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

def recv_animation_directives(directivesQueue, history, mapSurface, cellSize, pathColours, starts):
	stop = False
	index = 0
	tempCollisions = []
	while stop == False:
		if index >= len(directivesQueue):
			time.sleep(0.2)
		else:
			directive = directivesQueue[index]
			index+=1
			if directive == 'done':
				#draw last frame with path solutions
				'''
				currStep = copy.copy(history[-1])
				currStep[4] = None;
				history.append(currStep)
				finalStatic = mapSurface.copy()
				#draw paths
				for i in range(len(currStep[0])):
					for j in range(1, len(currStep[0][i])-1):
						drawCell(finalStatic, currStep[0][i][j][0], currStep[0][i][j][1], pathColours[i], cellSize)
				currStep[3] = finalStatic
				'''
				stop = True
				break

			currStep = copy.copy(history[-1])
			history.append(currStep)

			if directive[0] == 'initial paths':
				currStep[2] = history[-2][2]
				for agentPath in currStep[2]:
					currStep[0][agentPath['agent']] = agentPath['path']
				currStep[2] = []
				newStatic = mapSurface.copy()
				for i in range(len(currStep[0])):
					for j in range(1, len(currStep[0][i])-1):
						drawCell(newStatic, currStep[0][i][j][0], currStep[0][i][j][1], pathColours[i], cellSize)
				currStep[3] = newStatic
				currStep[4] = None
				#draw static for initial plan
			if directive[0] == 'improved solution':
				#remove isocontour progression
				#add new collisions
				currStep[4] = None
				currStep[5] = tempCollisions
				tempCollisions = []
				#add new paths to current solution
				#draw to new static new paths and collisions
				newStatic = currStep[3].copy()
				for agentPath in currStep[2]:
					currStep[0][agentPath['agent']] = agentPath['path']
					for i in range(1, len(agentPath['path'])-1):
						drawCell(newStatic, agentPath['path'][i][0], agentPath['path'][i][1], pathColours[agentPath['agent']], cellSize)
				
				#draw new collisions
				colour = pg.Color((200, 0, 0))
				for collision in tempCollisions:
					drawCell(newStatic, collision[0][0], collision[0][1], colour, cellSize, 1)
				currStep[3] = newStatic
				currStep[2] = []
				currStep[1] = None

			if directive[0] == 'no improvement':
				#remove isocontour progression
				#clear new collisions
				currStep[4] = None
				tempCollisions = []
				#draw static for old paths of neighbourhood
				newStatic = currStep[3].copy()
				#draw old solutions for neighbourhood agents
				for agent in currStep[1]['neighbourhood']:
					for i in range(1, len(currStep[0][agent])-1):
						drawCell(newStatic, currStep[0][agent][i][0], currStep[0][agent][i][1], pathColours[agent], cellSize)
				#draw old collisions
				colour = pg.Color((200, 0, 0))
				for collision in currStep[5]:
					drawCell(newStatic, collision[0][0], collision[0][1], colour, cellSize, 1)

				currStep[3] = newStatic
				currStep[2] = []
				currStep[1] = None
			if directive[0] == 'neighbourhood':
				tempCollisions = []
				currStep[1] = {'neighbourhood': set(directive[2]), 'ALNSweights': directive[3]}
				#create new static
				newStatic = mapSurface.copy()
				# draw paths outside neighbourhood not including starts and goals
				for i in range(1, len(currStep[0])-1):
					if i not in currStep[1]['neighbourhood']:
						for loc in currStep[i]:
							drawCell(newStatic, loc[0], loc[1], pathColours[i], cellSize)
				#draw purple boxes around neighbourhood agents
				colour = pg.Color((200, 0, 200))
				for agent in currStep[1]['neighbourhood']:
					drawCell(newStatic, starts[agent][0], starts[agent][1], colour, cellSize, 1)
				
				#filter collisions without neighbourood paths
				for collision in currStep[5]:
					if collision[1] not in currStep[1]['neighbourhood'] and collision[2] not in currStep[1]['neighbourhood']:
						tempCollisions.append(collision)

				#draw red boxes around temp collisions
				colour = pg.Color((200, 0, 0))
				for collision in tempCollisions:
					drawCell(newStatic, collision[0][0], collision[0][1], colour, cellSize, 1)

				currStep[3] = newStatic

			if directive[0] == 'pathing for agent':
				#setup Surface to draw isocontour progression from OPEN/CLOSED lists
				exploredSurface = pg.Surface(mapSurface.get_size())
				exploredSurface.set_colorkey(pg.Color(0, 0, 0))
				currStep[4] = [exploredSurface, 0, 0, 0, 0] #top, bottom, left, right
			
			if directive[0] == 'isocost contour':
				currStep[4] = [history[-2][4][0].copy(), history[-2][4][1], history[-2][4][2], history[-2][4][3], history[-2][4][4]]

				#draw light blue OPEN
				color = pg.Color((0, 0, 200))
				for loc in directive[1]: #newOpen
					drawCell(currStep[4][0], loc[0], loc[1], color, cellSize)
					#check for new area boundary
					if loc[1] < currStep[4][1]:
						currStep[4][1] = loc[1]
					if loc[1] > currStep[4][2]:
						currStep[4][2] = loc[1]
					if loc[0] < currStep[4][3]:
						currStep[4][3] = loc[0]
					if loc[0] > currStep[4][4]:
						currStep[4][4] = loc[0]

				#draw dark blue CLOSED
				color = pg.Color((0, 0, 100))
				for loc in directive[2]: #newClosed
					drawCell(currStep[4][0], loc[0], loc[1], color, cellSize)

			if directive[0] == 'path done':
				#add to completed paths
				temp = {'agent': directive[1], 'path': directive[2]}
				#copy path references from previous step into new list
				currStep[2] = []
				for completedPath in history[-2][2]:
					currStep[2].append(completedPath)

				currStep[4] = [history[-2][4][0].copy(), history[-2][4][1], history[-2][4][2], history[-2][4][3], history[-2][4][4]]

				#draw new path to prog static
				colour = pathColours[temp['agent']]
				for i in range(1, len(temp['path'])-1):
					drawCell(currStep[4][0], temp['path'][i][0], temp['path'][i][1], colour, cellSize)

				#check for collisions and draw to prog static
				for priorPath in currStep[2]:
					for i in range(min(len(priorPath['path']), len(temp['path']))):
						if temp['path'][i] == priorPath['path'][i]:
							newCollision = (temp['path'][i], temp['agent'], priorPath['agent'])
							tempCollisions.append(newCollision)
							drawCell(currStep[4][0], newCollision[0][0], newCollision[0][1], pg.Color((200, 0, 0)), cellSize, 1)

				#add new path to neighbourhood completed paths
				currStep[2].append(temp)





if __name__ == '__main__':
	neighbourhoodSize = 4
	numAgents = 5
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
		drawCell(mapSurface, goals[i][0], goals[i][1], goalColours[i], cellSize)
	displaySurface.blit(mapSurface, (0, 0))
	pg.display.update()


	#setup parallel processes and pipes
	#parentPipe, childPipe = mp.Pipe()
	directivesQueue = []
	t1 = threading.Thread(target=LNS2PP, args=(neighbourhoodSize, instanceMap, starts, goals, directivesQueue,))
	t1.run()

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
	eventHistory = [[[None]*numAgents, set(list(range(numAgents))), [], mapSurface, None, []]]
	t2 = threading.Thread(target=recv_animation_directives, args=(directivesQueue, eventHistory, mapSurface, cellSize, pathColours, starts))
	t2.run()

	running = True
	lastDrawnFrame = -1
	eventHistory_idx = 0
	frameRate = 60
	while running:

		#animate using event history
		time = pg.time.get_ticks()
		currFrame = time * frameRate // 1000
		if currFrame != lastDrawnFrame:
			#draw next frame
			lastDrawnFrame = currFrame
			if eventHistory_idx < len(eventHistory):
				#draw static
				displaySurface.blit(eventHistory[eventHistory_idx][3], (0, 0))
				#draw completed paths
				temp = eventHistory[eventHistory_idx][2]
				for agentPath in temp:
					for i in range(1, len(agentPath['path'])-1):
						drawCell(displaySurface, agentPath['path'][i][0], agentPath['path'][i][1], pathColours[agentPath['agent']], cellSize)

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
			eventHistory_idx += 1



		for event in pg.event.get():
			if event.type == pg.QUIT:
				running = False

