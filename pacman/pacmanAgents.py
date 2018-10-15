# pacmanAgents.py
# ---------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from pacman import Directions
from game import Agent
from heuristics import *
import random

class RandomAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # get all legal actions for pacman
        actions = state.getLegalPacmanActions()
        # returns random action from all the valide actions
        return actions[random.randint(0,len(actions)-1)]

class OneStepLookAheadAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # get all legal actions for pacman
        legal = state.getLegalPacmanActions()
        # get all the successor state for these actions
        successors = [(state.generatePacmanSuccessor(action), action) for action in legal]
        # evaluate the successor states using scoreEvaluation heuristic
        scored = [(admissibleHeuristic(state), action) for state, action in successors]
        # get best choice
        bestScore = min(scored)[0]
        # get all actions that lead to the highest score
        bestActions = [pair[1] for pair in scored if pair[0] == bestScore]
        # return random action from the list of the best actions
        return random.choice(bestActions)

class BFSAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # TODO: write BFS Algorithm instead of returning Directions.STOP
        visited = set()
		# if generatePacmanSuccessor fails the flagSuccessorFail is set to true
        flagSuccessorFail = False 	
        visited.add(state)
        next_next_state=state
		#As we start from level 2
        depth=1 	
		#implementing Queue using Lists
        bfsQueue=[] 	
        if state.isWin() or state.isLose():
			return Directions.STOP
        else:
            legal=state.getLegalPacmanActions()
            for action in legal:
				# get all the successor state for these actions
                next_state=state.generatePacmanSuccessor(action)
                if  next_state not in visited:
                    bfsQueue.append((next_state,action,depth))
                    visited.add(next_state)

            while len(bfsQueue)>0:
				#Remove Data from queue
				next_state, action,depthOfNode = bfsQueue.pop(0)
				if flagSuccessorFail== True:
					break
				#path to win state found, Thus return the 1st action required to reach there.
				elif(next_state.isWin()):
					return action
				# no further branching available for this node
				elif(next_state.isLose()):
					continue
				else:
					legal_2=next_state.getLegalPacmanActions()
					#Adding 1 to the depth of parent node
					depthOfNode=depthOfNode+1
					for action_2 in legal_2:
						next_next_state = next_state.generatePacmanSuccessor(action_2)
						#if generatePacmanSuccessor fails
						if(next_next_state== None):
							flagSuccessorFail=True
							break
						#Checking if the node is already visited, if not the add to the Queue and visited Set
						if next_next_state not in visited:
							#adding future states with old actions
							bfsQueue.append((next_next_state,action,depthOfNode))
							visited.add(next_next_state)
		#if the generatePacmanSuccessor fails and no Win state is located do following
        minAction=Directions.STOP
        if(len(bfsQueue)==0): # check if Queue is Empty
            return Directions.STOP
        else:
			minValue= admissibleHeuristic(bfsQueue[0][0])+bfsQueue[0][2]
			for que in bfsQueue:
				#Using Heuristics to find the best available option by f(n)=h(n)+g(n)
				score=admissibleHeuristic(que[0])+que[2];
				#Found it difficult to use so, did not use ->array.sort(key=lambda x: admissibleHeuristic(x)).
				#Finding the minimum value linearly
				if minValue>=score:
					minValue=score
					minAction=que[1]
				
        return minAction

class DFSAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # TODO: write BFS Algorithm instead of returning Directions.STOP
        visited = set()
		# if generatePacmanSuccessor fails the flagSuccessorFail is set to true
        flagSuccessorFail = False
        visited.add(state)
        next_next_state=state
		#As we start from level 2
        depth=1
		#implementing Stack using Lists
        dfsStack=[] 
        if state.isWin() or state.isLose():
			return Directions.STOP
        else:
            legal=state.getLegalPacmanActions()
            for action in legal:
				# get all the successor state for these actions
                next_state=state.generatePacmanSuccessor(action)
                if  next_state not in visited:
                    dfsStack.append((next_state,action,depth))
                    visited.add(next_state)

            while len(dfsStack)>0:
				#Remove Data from Stack using -1 index
				next_state, action,depthOfNode = dfsStack.pop(-1)
				if flagSuccessorFail== True:
					break
				#path to win state found, Thus return the 1st action required to reach there.
				elif(next_state.isWin()): 
					return action
				# no further branching available for this path
				elif(next_state.isLose()):
					continue
				else:
					legal_2=next_state.getLegalPacmanActions()
					#Adding 1 to the depth of parent node
					depthOfNode=depthOfNode+1
					for action_2 in legal_2:
						# get all the successor state for these actions
						next_next_state = next_state.generatePacmanSuccessor(action_2)
						#if generatePacmanSuccessor fails
						if(next_next_state== None):
							flagSuccessorFail=True
							break
						
						#Checking if the node is already visited, if not the add to the Stack and visited Set
						if next_next_state not in visited:
							#adding future states with old actions
							dfsStack.append((next_next_state,action,depthOfNode))
							visited.add(next_next_state)
		#if the generatePacmanSuccessor fails and no Win state is located do following
        minAction=Directions.STOP
        if(len(dfsStack)==0):#Check if the stack is empty
			return Directions.STOP
        else:
			minValue= admissibleHeuristic(dfsStack[0][0])+dfsStack[0][2] #10000
			for que in dfsStack:
				#Using Heuristics to find the best available option by f(n)=h(n)+g(n)
				score=admissibleHeuristic(que[0])+que[2];
				#Found it difficult to use so, did not use ->array.sort(key=lambda x: admissibleHeuristic(x)).
				#Finding the minimum value linearly
				if minValue>=score:
					minValue=score
					minAction=que[1]

        return minAction

class AStarAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # TODO: write BFS Algorithm instead of returning Directions.STOP
        visited = set() #closed list
        visited.add(state)
		# if generatePacmanSuccessor fails the flagSuccessorFail is set to true
        flagSuccessorFail = False  
        next_next_state=state

        successor_list=[] #open list
        if state.isWin() or state.isLose():
			return Directions.STOP
        else:
            legal=state.getLegalPacmanActions()
            depth=1
            for action in legal:
                next_state=state.generatePacmanSuccessor(action)
                if  next_state not in visited:
					#f(n)=h(n)+g(n)
					fofn=admissibleHeuristic(next_state)+depth #taking gOfn as depth of the current node.
					# get all the successor state for these actions
					successor_list.append((fofn,next_state,action,depth))
					visited.add(next_state)
                 
            if not flagSuccessorFail:
                while len(successor_list)>0:
					#Did not understand how to use ->array.sort(key=lambda x: admissibleHeuristic(x)). Thus getting the min value using min()
					#get the node with the minimum F(n)
                    bestValue=min(successor_list)[0]
                    for node in successor_list:
						if node[0] == bestValue:
							bestNode = node
							break
                    minIndex=successor_list.index(bestNode)
                    fofn,next_state, action,depthOfNode = successor_list.pop(minIndex)
					#Adding 1 to the depth of parent node
                    depthOfNode=depthOfNode+1 
                    if flagSuccessorFail== True:
                        break
                    if(next_state.isWin()):
                        return action
                    # no further branching available for this path
                    elif(next_state.isLose()):
                        continue
                    else:
                        legal_2=next_state.getLegalPacmanActions()
                        for action_2 in legal_2:
                            next_next_state = next_state.generatePacmanSuccessor(action_2)
                            if(next_next_state== None):
                                flagSuccessorFail=True
                                break
                            if next_next_state not in visited:
								#f(n)=h(n)+g(n)
								fofn=admissibleHeuristic(next_next_state)+depthOfNode	#taking gOfn as depth of the current node.
								#adding future states with old actions
								successor_list.append((fofn,next_next_state,action,depthOfNode))
								visited.add(next_next_state)
			
			#if the generatePacmanSuccessor fails and no Win state is located do following
			if(len(successor_list)>0):
				#get the least heuristic value available and select the action acquainted with it
				bestValue=min(successor_list)[0]
				for node in successor_list:
					if node[0] == bestValue:
						bestActions = node[2]
						break#getting the first action found and ending the loop
			else:
				bestActions=Directions.STOP
				
            return bestActions