from loadscen import *
from ReplanPPSIPPS import *
#import pygame as pg
import multiprocessing as mp

if __name__ == '__main__':
	neighbourhoodSize = 4
	scenarioFile = 'Boston_0_256-even-1.scen'
	instanceMap, starts, goals = loadScen(scenarioFile, 20)

	#setup parallel processes and pipes
	parentPipe, childPipe = mp.Pipe()
	p = mp.Process(target=LNS2PP, args=(neighbourhoodSize, instanceMap, starts, goals, childPipe,))
	p.start()
	solutionStep = parentPipe.recv()
	while solutionStep != 'done':
		print(len(solutionStep))
		solutionStep = parentPipe.recv()

	#todo: animation code




