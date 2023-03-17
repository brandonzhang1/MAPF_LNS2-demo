from loadscen import *
from ReplanPPSIPPS import *
import pygame as pg
import threading
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
	#return (pathSurface, top, bottom+1, left, right+1)
	return (pathSurface, (left*cellSize, top*cellSize), (left*cellSize, top*cellSize, (right+1-left)*cellSize, (bottom+1-top)*cellSize))

def drawCollisions(surface, collisions, cellSize):
	red = pg.Color((220, 0, 0))
	for c in collisions:
		drawBigCell(surface, c[0][0], c[0][1], red, cellSize)
		if len(c) == 4:
			drawBigCell(surface, c[1][0], c[1][1], red, cellSize)

def recv_animation_directives(directivesQueue, history, mapSurface, cellSize, pathColours, starts, event):
	stop = False
	index = 0
	oldCollisions = []
	oldSolutionStatic = None
	while stop == False:
		if event.is_set():
			return
		if index >= len(directivesQueue):
			if directivesQueue[-1] == 'done':
				return
		else:
			directive = directivesQueue[index]
			index+=1
			if directive == 'done':
				stop = True
				break
			
			def copyLastEvent():
				currStep = copy.copy(history[-1])
				history.append(currStep)
				return currStep
			
			if directive[0] == 'initial paths':
				currStep = copyLastEvent()
				currStep['caption'] = "initial rough solution"
				currStep['collisions'] = directive[1]
				currStep['neighbourhood']['weights'] = directive[2]
				currStep['solution'] = [None]*len(currStep['neighbourhood']['completedPaths'])
				currStep['static'] = history[-2]['static'].copy()
				for agent in currStep['neighbourhood']['completedPaths'].keys():
					path = currStep['neighbourhood']['completedPaths'][agent]
					currStep['solution'][agent] = path
				drawCollisions(currStep['static'], currStep['collisions'], cellSize)

			if directive[0] == 'neighbourhood':
				currStep = copyLastEvent()
				#currStep['caption'] = directive[1] + " | " + " ".join([str(a) for a in directive[3]])
				currStep['caption'] = directive[1] + " neighbourhood: " + " ".join([str(a) for a in directive[2]])
				oldCollisions = currStep['collisions']
				currStep['neighbourhood'] = {'agents': directive[2], 'weights': directive[3], 'completedPaths': {}, 'currentPath': None}
				currStep['collisions'] = [c for c in oldCollisions if 
			      	(len(c) == 3 and (c[1] not in directive[2] and c[2] not in directive[2])) 
					or (len(c) == 4 and (c[2] not in directive[2] and c[3] not in directive[2]))]
				oldSolutionStatic = currStep['static']
				currStep['static'] = mapSurface.copy()
				for agent in range(len(currStep['solution'])):
					if agent not in directive[2]:
						path = currStep['solution'][agent]
						currStep['static'].blit(path[0], path[1], path[2])
				drawCollisions(currStep['static'], currStep['collisions'], cellSize)

			if directive[0] == 'pathing for agent':
				currStep = copyLastEvent()
				currStep['caption'] = "agent " + str(directive[1])
				currStep['neighbourhood'] = copy.copy(history[-2]['neighbourhood'])
				currStep['neighbourhood']['currentPath'] = {'agent': directive[1], 'progression': [],}

			if directive[0] == 'isocost contours': #[1]: [(newOpen, newClosed), ...]
				currStep = history[-1]
				currStep['neighbourhood']['currentPath']['progression'] = directive[1]
				
			if directive[0] == 'path done':
				currStep = copyLastEvent()
				currStep['caption'] = "path found"
				path = drawPath(directive[2], pathColours[directive[1]], mapSurface.get_size(), cellSize)
				currStep['neighbourhood'] = copy.copy(history[-2]['neighbourhood'])
				currStep['neighbourhood']['completedPaths'][directive[1]] = path
				currStep['neighbourhood']['currentPath'] = None
				currStep['static'] = history[-2]['static'].copy()
				currStep['static'].blit(path[0], path[1], path[2])

			if directive[0] == 'improved solution':
				currStep = copyLastEvent()
				currStep['caption'] = "improved solution"
				currStep['collisions'] = directive[1]
				currStep['solution'] = copy.copy(history[-2]['solution'])
				for agent in currStep['neighbourhood']['completedPaths'].keys():
					currStep['solution'][agent] = currStep['neighbourhood']['completedPaths'][agent]
				currStep['neighbourhood']['completedPaths']
				currStep['neighbourhood'] = {'agents': set(), 'weights': directive[2], 'completedPaths': {}, 'currentPath': None}
				currStep['static'] = history[-2]['static'].copy()
				drawCollisions(currStep['static'], currStep['collisions'], cellSize)
				
			if directive[0] == 'no improvement':
				currStep = copyLastEvent()
				currStep['caption'] = "no improvement"
				currStep['neighbourhood'] = {'agents': set(), 'weights': directive[1], 'completedPaths': {}, 'currentPath': None}
				currStep['collisions'] = oldCollisions
				currStep['static'] = oldSolutionStatic


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
	pg.display.set_mode((mapWidth*cellSize+200, mapHeight*cellSize))
	displaySurface = pg.display.get_surface()
	#displaySurface.fill((255, 255, 255))

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

	#setup concurrent thread for running solver and creating directives for animation threads
	executor = thread.ThreadPoolExecutor()
	threadEvent = threading.Event()
	directivesQueue = []
	t1 = executor.submit(LNS2PP, neighbourhoodSize, instanceMap, starts, goals, directivesQueue, threadEvent)
	
	eventHistory = [{ 'caption': "initial pathfinding", 'solution':[], 'neighbourhood': {'agents': set(), 'weights': [], 'completedPaths': {}, 'currentPath': None}, 'collisions': [] , 'static': mapSurface}]
	t2 = executor.submit(recv_animation_directives, directivesQueue, eventHistory, mapSurface, cellSize, pathColours, starts, threadEvent)
	executor.shutdown(False)

	def renderFrame(eventHistory, eventHistory_idx, progIdx, progSurface):
		#draw static
		displaySurface.blit(eventHistory[eventHistory_idx]['static'], (0, 0))

		#handle path prog
		#draw new closed and new open to progSurface
		colour = pg.Color((0, 0, 100))
		if eventHistory[eventHistory_idx]['neighbourhood']['currentPath'] != None:
			isoContour = eventHistory[eventHistory_idx]['neighbourhood']['currentPath']['progression'][progIdx]
			for newClosed in isoContour[1]:
				drawCell(progSurface, newClosed[0], newClosed[1], colour, cellSize)
			colour = pg.Color((0, 0, 220))
			for newOpen in isoContour[0]:
				drawCell(progSurface, newOpen[0], newOpen[1], colour, cellSize)

		displaySurface.blit(progSurface, (0, 0))
		pg.display.update()

	running = True
	pause = False
	lastDrawnFrame = -1
	eventHistory_idx = 0
	progIdx = 0
	progSurface = pg.Surface(mapSurface.get_size())
	progSurface.set_colorkey(pg.Color(0, 0, 0))
	frameRate = 60

	solutionDirectory = {}
	directoryCheckIdx = 0
	textGenerator = pg.font.Font()
	directorySurface = pg.Surface((200, mapHeight*cellSize))
	#directorySurface.fill((255, 255, 255))
	directoryYcoord = 0
	while running:
		#animate using event history
		time = pg.time.get_ticks()
		currFrame = time * frameRate // 1000
		if currFrame != lastDrawnFrame:
			if pause == False:
				#draw next frame
				lastDrawnFrame = currFrame
				renderFrame(eventHistory, eventHistory_idx, progIdx, progSurface)
				#handle indices
				progIdx += 1
				currPath = eventHistory[eventHistory_idx]['neighbourhood']['currentPath']
				if currPath == None or progIdx >= len(currPath['progression']):
					eventHistory_idx = min(eventHistory_idx + 1, len(eventHistory)-2)
					progIdx = 0
					progSurface = pg.Surface(mapSurface.get_size())
					progSurface.set_colorkey(pg.Color(0, 0, 0))
					progSurfaceDims = [0, 0, 0, 0]

			#always draw new directory listings
			displaySurface.blit(directorySurface, ((mapWidth+1)*cellSize + 20, 12))
			pg.display.update()

		#check history for new captions to add to directory
		if directoryCheckIdx < len(eventHistory)-1:
			caption = eventHistory[directoryCheckIdx]['caption']
			surface = textGenerator.render(caption, True, (0, 0, 0), (200, 200, 200))
			#12px per line
			directorySurface.blit(surface, (0, directoryYcoord*12))
			#find last agent
			solutionDirectory[directoryYcoord] = [(0, directoryCheckIdx, surface.get_size())]
			
			if caption[0] == 'a':	#put path found on same line as agent path prog
				#print(caption)
				directoryCheckIdx+=1
				caption = eventHistory[directoryCheckIdx]['caption']
				xoffset = surface.get_size()[0] + 10 #from prev surface
				surface = textGenerator.render(caption, True, (0, 0, 0), (200, 200, 200))
				directorySurface.blit(surface, (xoffset, directoryYcoord*12))
				solutionDirectory[directoryYcoord].append((xoffset, directoryCheckIdx, surface.get_size()))

			directoryCheckIdx+=1
			directoryYcoord+=1

		

		for event in pg.event.get():
			if event.type == pg.QUIT:
				threadEvent.set()
				t1.cancel()
				t2.cancel()
				running = False
			if event.type == pg.KEYDOWN:
				if event.key == pg.K_SPACE:
					pause = pause ^ True
				if event.key == pg.K_LEFT:
					eventHistory_idx = max(eventHistory_idx - 1, 0)
					if event.mod & pg.KMOD_LSHIFT:
						eventHistory_idx = max(eventHistory_idx - 4, 0)
					progIdx = 0
					progSurface = pg.Surface(mapSurface.get_size())
					progSurface.set_colorkey(pg.Color(0, 0, 0))
					progSurfaceDims = [0, 0, 0, 0]
					renderFrame(eventHistory, eventHistory_idx, progIdx, progSurface)
				if event.key == pg.K_RIGHT:
					eventHistory_idx = min(eventHistory_idx + 1, len(eventHistory)-2)
					if event.mod & pg.KMOD_LSHIFT:
						eventHistory_idx = min(eventHistory_idx + 4, len(eventHistory)-2)
					progIdx = 0
					progSurface = pg.Surface(mapSurface.get_size())
					progSurface.set_colorkey(pg.Color(0, 0, 0))
					progSurfaceDims = [0, 0, 0, 0]
					renderFrame(eventHistory, eventHistory_idx, progIdx, progSurface)
			if event.type == pg.MOUSEBUTTONDOWN:
				#handle positionality
				pos = event.pos
				#hash into solution directory
				if pos[0] > (mapWidth+1)*cellSize + 20:
					x = pos[0] - (mapWidth+1)*cellSize - 20
					y = (pos[1]-12)//12
					if y in solutionDirectory:
						boxes = solutionDirectory[y]
						for item in boxes:
							if x >= item[0] and x < (item[0] + item[2][0]):
								eventHistory_idx = item[1]
								progIdx = 0
								progSurface = pg.Surface(mapSurface.get_size())
								progSurface.set_colorkey(pg.Color(0, 0, 0))
								renderFrame(eventHistory, eventHistory_idx, progIdx, progSurface)
								break
				
					