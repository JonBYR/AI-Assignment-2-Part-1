#pragma once
//======================================================================================
//Header file for bots: initially cBotRandom is defined here, but other bot classes
//can go here too
//
//(c) Patrick Dickinson, University of Lincoln, School of Computer Science, 2020
//======================================================================================

#include "botbase.h"
#include <vector>
#include <math.h>
class cBotRandom : public cBotBase
{
	virtual void ChooseNextGridPosition();
};
class cDijkstra 
{
public:
	bool closed[GRIDWIDTH][GRIDHEIGHT]; //whether or not location is closed
	float cost[GRIDWIDTH][GRIDHEIGHT]; //cost of each weight
	int linkX[GRIDWIDTH][GRIDHEIGHT]; //link x coord
	int linkY[GRIDWIDTH][GRIDHEIGHT]; //link y coord
	bool inPath[GRIDWIDTH][GRIDHEIGHT]; //whether or not location is in the final path
	bool completed;
	cDijkstra() { completed = false; }
	virtual void Build(cBotBase& bot);
};
extern cDijkstra gDijkstra;
class cAStar : public cDijkstra 
{
public:
	std::vector<int> pathX;
	std::vector<int> pathY; //used to create the path
	virtual void Build(cBotBase& bot, int h); //used to build the path
};
class cBotAStar : public cBotBase 
{
public:
	cBotAStar() { atPlayer = true; }; //initialise atPlayer to true so the pathfinding can start
	virtual void ChooseNextGridPosition(); //Bot that will use the A Star pathing algorithm
};
extern cAStar gAStar; //needed to call the functions and variables of the A star class outside of the class


