#include "bots.h"
void cDijkstra::Build(cBotBase &bot) //Dijkstras which AStar is a variant of
{
	completed = false;
	for (int i = 0; i < GRIDWIDTH; i++) 
	{
		for (int j = 0; j < GRIDHEIGHT; j++) 
		{
			closed[i][j] = false;
			cost[i][j] = 10000000.0f;
			linkX[i][j] = -1;
			linkY[i][j] = -1;
			inPath[i][j] = false;
		}
	}
	cost[bot.PositionX()][bot.PositionY()] = 0;
	while (completed == false) 
	{
		float tempCost = 100000000.0f;
		int tempWidth;
		int tempHeight;
		for (int i = 0; i < GRIDWIDTH; i++) 
		{
			for (int j = 0; j < GRIDHEIGHT; j++) 
			{
				
				if (cost[i][j] <= tempCost && closed[i][j] == false && gLevel.isValid(i, j)) 
				{
					tempCost = cost[i][j];
					tempWidth = i;
					tempHeight = j;
				}
			}
		}
		closed[tempWidth][tempHeight] = true;
		if (gTarget.PositionX() == tempWidth && gTarget.PositionY() == tempHeight) break;
		if (gLevel.isValid(tempWidth, tempHeight - 1) && closed[tempWidth][tempHeight - 1] == false) 
		{
			if (cost[tempWidth][tempHeight - 1] > tempCost + 1) 
			{
				cost[tempWidth][tempHeight - 1] = tempCost + 1;
				linkX[tempWidth][tempHeight - 1] = tempWidth;
				linkY[tempWidth][tempHeight - 1] = tempHeight;
			}
		}
		if (gLevel.isValid(tempWidth, tempHeight + 1) && closed[tempWidth][tempHeight + 1] == false)
		{
			if (cost[tempWidth][tempHeight + 1] > tempCost + 1) 
			{
				cost[tempWidth][tempHeight + 1] = tempCost + 1;
				linkX[tempWidth][tempHeight + 1] = tempWidth;
				linkY[tempWidth][tempHeight + 1] = tempHeight;
			}
		}
		if (gLevel.isValid(tempWidth - 1, tempHeight - 1) && closed[tempWidth - 1][tempHeight - 1] == false)
		{
			if (cost[tempWidth - 1][tempHeight - 1] > tempCost + 1.4f) 
			{
				cost[tempWidth - 1][tempHeight - 1] = tempCost + 1.4f;
				linkX[tempWidth - 1][tempHeight - 1] = tempWidth;
				linkY[tempWidth - 1][tempHeight - 1] = tempHeight;
			}
		}
		if (gLevel.isValid(tempWidth + 1, tempHeight - 1) && closed[tempWidth + 1][tempHeight - 1] == false)
		{
			if (cost[tempWidth + 1][tempHeight - 1] > tempCost + 1.4f) 
			{
				cost[tempWidth + 1][tempHeight - 1] = tempCost + 1.4f;
				linkX[tempWidth + 1][tempHeight - 1] = tempWidth;
				linkY[tempWidth + 1][tempHeight - 1] = tempHeight;
			}
		}
		if (gLevel.isValid(tempWidth - 1, tempHeight + 1) && closed[tempWidth - 1][tempHeight + 1] == false)
		{
			if (cost[tempWidth - 1][tempHeight + 1] > tempCost + 1.4f) 
			{
				cost[tempWidth - 1][tempHeight + 1] = tempCost + 1.4f;
				linkX[tempWidth - 1][tempHeight + 1] = tempWidth;
				linkY[tempWidth - 1][tempHeight + 1] = tempHeight;
			}
		}
		if (gLevel.isValid(tempWidth + 1, tempHeight + 1) && closed[tempWidth + 1][tempHeight + 1] == false)
		{
			if (cost[tempWidth + 1][tempHeight + 1] > tempCost + 1.4f) 
			{
				cost[tempWidth + 1][tempHeight + 1] = tempCost + 1.4f;
				linkX[tempWidth + 1][tempHeight + 1] = tempWidth;
				linkY[tempWidth + 1][tempHeight + 1] = tempHeight;
			}
		}
		if (gLevel.isValid(tempWidth - 1, tempHeight) && closed[tempWidth - 1][tempHeight] == false)
		{
			if (cost[tempWidth - 1][tempHeight] > tempCost + 1) 
			{
				cost[tempWidth - 1][tempHeight] = tempCost + 1;
				linkX[tempWidth - 1][tempHeight] = tempWidth;
				linkY[tempWidth - 1][tempHeight] = tempHeight;
			}
		}
		if (gLevel.isValid(tempWidth + 1, tempHeight) && closed[tempWidth + 1][tempHeight] == false)
		{
			if (cost[tempWidth + 1][tempHeight] > tempCost + 1) 
			{
				cost[tempWidth + 1][tempHeight] = tempCost + 1;
				linkX[tempWidth + 1][tempHeight] = tempWidth;
				linkY[tempWidth + 1][tempHeight] = tempHeight;
			}
		}
	}
	bool done = false; //sets to true when back at the bots position
	int nextClosedX = gTarget.PositionX(); //start of paths x coordinate
	int nextClosedY = gTarget.PositionY(); //start of paths y coordinate
	while (!done) 
	{
		inPath[nextClosedX][nextClosedY] = true;
		int tempX = nextClosedX;
		int tempY = nextClosedY;
		nextClosedX = linkX[tempX][tempY];
		nextClosedY = linkY[tempX][tempY];
		if (nextClosedX == bot.PositionX() && nextClosedY == bot.PositionY()) done = true;
	}
	completed = true;
}
cDijkstra gDijkstra;
void cAStar::Build(cBotBase& bot, int h)
{
	completed = false;
	for (int i = 0; i < GRIDWIDTH; i++)
	{
		for (int j = 0; j < GRIDHEIGHT; j++)
		{
			closed[i][j] = false; //all nodes must first be closed
			cost[i][j] = 10000000.0f; //the cost must initially be a very high value
			linkX[i][j] = -1;
			linkY[i][j] = -1; //initialise the horizontal and vertical node links
			inPath[i][j] = false; //none of the nodes have yet been included in the path
			//initialised the members of the grid which are inherited from the dijkstra class
		}
	}
	cost[bot.PositionX()][bot.PositionY()] = 0; //initilises the cost of the start position of the bot to be 0 as we haven't traveled anywhere
	while (completed == false) //while the path is still being made
	{
		float tempCost = 100000000.0f; //initilised to a very high value
		int tempWidth;
		int tempHeight;
		float heuristic;
		for (int i = 0; i < GRIDWIDTH; i++)
		{
			for (int j = 0; j < GRIDHEIGHT; j++)
			{
				if (h == 1) heuristic = (fabs(gTarget.PositionX() - i) + fabs(gTarget.PositionY() - j)); //manhattan distance which is the sum of the horizontal difference
				//plus the sum of the vertical distance
				else if (h == 2) heuristic = sqrt(pow(fabs(gTarget.PositionX() - i), 2) + pow(fabs(gTarget.PositionY() - j), 2));
				//euclidean distance which is the horizontal difference squared plus the vertical difference squared and then the total square rooted
				else if (h == 3) heuristic = fabs(gTarget.PositionX() - i) + fabs(gTarget.PositionY() - j) + (-0.6 * (std::min(fabs(gTarget.PositionX() - i), fabs(gTarget.PositionY() - j))));
				//diagonal distance which is D1 (in our case 1) multiplied by the manhattan distance (which is just the same as manhattan distance)
				//added by D2 - 2 * 1 (D2 is 1.4 in our case which subtracted by 2 and multiplied by 1 is -0.6) 
				//which is the multiplied by the minimum of the difference horizontally or vertically
				if ((cost[i][j] + heuristic) < tempCost && closed[i][j] == false && gLevel.isValid(i, j))
				{ //checks the cost added with the heuristic is smaller than what is currently in tempcost, as well as if the node is open and is not a wall
					//if these conditions are met then tempCost is updated to reflect the new cost
					tempCost = cost[i][j] + heuristic;
					tempWidth = i;
					tempHeight = j;
				}
			}
		}
		closed[tempWidth][tempHeight] = true; //move to a new node that is now closed
		tempCost = cost[tempWidth][tempHeight]; //tempCost is given the new cost
		if (gTarget.PositionX() == tempWidth && gTarget.PositionY() == tempHeight) break; //if we reach the player then lowest cost path has been found
		//check the cost of the neighbours
		if (gLevel.isValid(tempWidth, tempHeight - 1) && closed[tempWidth][tempHeight - 1] == false) //going up
		{
			if (cost[tempWidth][tempHeight - 1] > tempCost + 1) //if the current cost is greater than the tempCost plus the distance it
				//takes to travel to this node
			{
				cost[tempWidth][tempHeight - 1] = tempCost + 1; //replace the cost with the new cost
				linkX[tempWidth][tempHeight - 1] = tempWidth; //create a link to the node's x and y coordinates
				linkY[tempWidth][tempHeight - 1] = tempHeight;
			}
		}
		if (gLevel.isValid(tempWidth, tempHeight + 1) && closed[tempWidth][tempHeight + 1] == false) //going down
		{
			if (cost[tempWidth][tempHeight + 1] > tempCost + 1)
			{
				cost[tempWidth][tempHeight + 1] = tempCost + 1;
				linkX[tempWidth][tempHeight + 1] = tempWidth;
				linkY[tempWidth][tempHeight + 1] = tempHeight;
			}
		}
		if (gLevel.isValid(tempWidth - 1, tempHeight - 1) && closed[tempWidth - 1][tempHeight - 1] == false) //going top left
		{
			if (cost[tempWidth - 1][tempHeight - 1] > tempCost + 1.4f) //the cost of travelling diagonally is 1.4 as opposed to 1
			{
				cost[tempWidth - 1][tempHeight - 1] = tempCost + 1.4f;
				linkX[tempWidth - 1][tempHeight - 1] = tempWidth;
				linkY[tempWidth - 1][tempHeight - 1] = tempHeight;
			}
		}
		if (gLevel.isValid(tempWidth + 1, tempHeight - 1) && closed[tempWidth + 1][tempHeight - 1] == false) //going top right
		{
			if (cost[tempWidth + 1][tempHeight - 1] > tempCost + 1.4f)
			{
				cost[tempWidth + 1][tempHeight - 1] = tempCost + 1.4f;
				linkX[tempWidth + 1][tempHeight - 1] = tempWidth;
				linkY[tempWidth + 1][tempHeight - 1] = tempHeight;
			}
		}
		if (gLevel.isValid(tempWidth - 1, tempHeight + 1) && closed[tempWidth - 1][tempHeight + 1] == false) //going bottom left
		{
			if (cost[tempWidth - 1][tempHeight + 1] > tempCost + 1.4f)
			{
				cost[tempWidth - 1][tempHeight + 1] = tempCost + 1.4f;
				linkX[tempWidth - 1][tempHeight + 1] = tempWidth;
				linkY[tempWidth - 1][tempHeight + 1] = tempHeight;
			}
		}
		if (gLevel.isValid(tempWidth + 1, tempHeight + 1) && closed[tempWidth + 1][tempHeight + 1] == false) //going bottom right
		{
			if (cost[tempWidth + 1][tempHeight + 1] > tempCost + 1.4f)
			{
				cost[tempWidth + 1][tempHeight + 1] = tempCost + 1.4f;
				linkX[tempWidth + 1][tempHeight + 1] = tempWidth;
				linkY[tempWidth + 1][tempHeight + 1] = tempHeight;
			}
		}
		if (gLevel.isValid(tempWidth - 1, tempHeight) && closed[tempWidth - 1][tempHeight] == false) //going left
		{
			if (cost[tempWidth - 1][tempHeight] > tempCost + 1)
			{
				cost[tempWidth - 1][tempHeight] = tempCost + 1;
				linkX[tempWidth - 1][tempHeight] = tempWidth;
				linkY[tempWidth - 1][tempHeight] = tempHeight;
			}
		}
		if (gLevel.isValid(tempWidth + 1, tempHeight) && closed[tempWidth + 1][tempHeight] == false) //going right
		{
			if (cost[tempWidth + 1][tempHeight] > tempCost + 1)
			{
				cost[tempWidth + 1][tempHeight] = tempCost + 1;
				linkX[tempWidth + 1][tempHeight] = tempWidth;
				linkY[tempWidth + 1][tempHeight] = tempHeight;
			}
		}
	}
	bool done = false; //sets to true when back at the bots position
	int nextClosedX = gTarget.PositionX(); //start of paths x coordinate
	int nextClosedY = gTarget.PositionY(); //start of paths y coordinate
	while (!done)
	{
		pathX.push_back(nextClosedX);
		pathY.push_back(nextClosedY); //push the x and y path coordinates to the corresponding vector
		inPath[nextClosedX][nextClosedY] = true; //states that this is a node that is in the path
		int tempX = nextClosedX;
		int tempY = nextClosedY;
		nextClosedX = linkX[tempX][tempY];
		nextClosedY = linkY[tempX][tempY]; //states the coordinates of the next node in the path
		if (nextClosedX == bot.PositionX() && nextClosedY == bot.PositionY()) //if we get back to the bot then the path has been completed and we are done
		{ 
			done = true;
		}

	}
	completed = true; //A star has been completed
}
cAStar gAStar; //create a global A Star object
void cBotAStar::ChooseNextGridPosition() //bot which follows AStar
{
	atPlayer = false; //bot is not at the player at the start of the path
	if (gAStar.pathX.size() == 0) {
		atPlayer = true; //bot is at player at the start of the path
		return; }//if we reach the end of the list then we return
	bool done = false;
	done = SetNext(gAStar.pathX.back(), gAStar.pathY.back(), gLevel); //gets the last element in the list which will be the next position for the bot to go to
	//assigned a bool as SetNext returns a bool
	gAStar.pathX.pop_back();
	gAStar.pathY.pop_back(); //removes this element from the list as the bot has now travelled there
}