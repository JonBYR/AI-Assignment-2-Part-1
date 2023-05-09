//======================================================================================
//Source code for main function = main control loop for programme
//
//(c) Patrick Dickinson, University of Lincoln, School of Computer Science, 2020
//======================================================================================

#include "game.h"
#include "level.h"
#include "dynamic.h"
#include "bots.h"

//======================================================================================
//Globals
//======================================================================================
SDL_Window *gMainWindow = NULL;
SDL_Renderer *gMainRenderer = NULL;
SDL_Surface *tileSurface = NULL;
SDL_Surface* tileBlockedSurface = NULL;
SDL_Surface* targetSurface = NULL;
SDL_Surface* botSurface = NULL;
SDL_Surface* tileClosedSurface = NULL;
SDL_Surface* tileRouteSurface = NULL;
SDL_Texture* tileTexture = NULL;
SDL_Texture* tileBlockedTexture = NULL;
SDL_Texture* targetTexture = NULL;
SDL_Texture* botTexture = NULL;
SDL_Texture* tileClosedTexture = NULL;
SDL_Texture* tileRouteTexture = NULL;
bool gQuit;


//======================================================================================
//Main function
//======================================================================================
int main(int argc, char* argv[])
{
	gQuit = false;

    //======================================================================================
    //Initialise SDL
    //======================================================================================
    SDL_Init(SDL_INIT_EVERYTHING);
	gMainWindow = SDL_CreateWindow
	("Pathfinder", // window's title
		30, 50, // coordinates on the screen, in pixels, of the window's upper left corner
		640, 640, // window's length and height in pixels  
		SDL_WINDOW_OPENGL);
    gMainRenderer = SDL_CreateRenderer(gMainWindow, -1, SDL_RENDERER_ACCELERATED);

    //======================================================================================
    //Load graphics for tiles, bot and target
    //======================================================================================
    tileSurface = SDL_LoadBMP("tile.bmp");
    tileBlockedSurface = SDL_LoadBMP("tile-blocked.bmp");
    targetSurface = SDL_LoadBMP("target.bmp");
    botSurface = SDL_LoadBMP("bot.bmp");
    tileClosedSurface = SDL_LoadBMP("tile-closed.bmp");
    tileRouteSurface = SDL_LoadBMP("tile-route.bmp");
    
    tileTexture = SDL_CreateTextureFromSurface(gMainRenderer, tileSurface);
    tileBlockedTexture = SDL_CreateTextureFromSurface(gMainRenderer, tileBlockedSurface);
    targetTexture = SDL_CreateTextureFromSurface(gMainRenderer, targetSurface);
    botTexture = SDL_CreateTextureFromSurface(gMainRenderer, botSurface);
    tileClosedTexture = SDL_CreateTextureFromSurface(gMainRenderer, tileClosedSurface);
    tileRouteTexture = SDL_CreateTextureFromSurface(gMainRenderer, tileRouteSurface);

    //======================================================================================
    //Load the map and set target position
    //======================================================================================
    gLevel.Load("maps/1.txt");
    gTarget.SetCurrent(30, 20, gLevel);

    //======================================================================================
    //Locals variables fro key presses and frame timer
    //======================================================================================
    SDL_Event event;
    const Uint8* keystate;  
    int timerMS = SDL_GetTicks();
    int deltaTimeMS = 0;
    bool botAtPlayer = false;
    //======================================================================================
    // Create Bot
    //======================================================================================
    cBotBase* pBot = new cBotAStar();
    pBot->SetCurrent(10, 20, gLevel);

    //======================================================================================
    //Main loop
    //======================================================================================
    while(!gQuit)
	{   
        //======================================================================================
        //Poll events for quit
        //======================================================================================
        while(SDL_PollEvent(&event))
        {
            switch (event.type)
            {
            case SDL_QUIT:
                gQuit = true;
                break;

            case SDL_KEYDOWN:
                switch (event.key.keysym.sym)
                {
                case SDLK_ESCAPE:
                case SDLK_q:
                    gQuit = true;
                    break;
                }
                break;
            }
        }
        
        //======================================================================================
        //Keyboard input for target control 
        //======================================================================================
        keystate = SDL_GetKeyboardState(NULL);
        int offsetX = 0;
        int offsetY = 0;
        if (keystate[SDL_SCANCODE_UP]) offsetY -= 1;
        if (keystate[SDL_SCANCODE_DOWN]) offsetY += 1;
        if (keystate[SDL_SCANCODE_LEFT]) offsetX -= 1;
        if (keystate[SDL_SCANCODE_RIGHT]) offsetX += 1;
        if (!botAtPlayer)
        {
            
            static bool e_down = false;
            static bool m_down = false;
            static bool d_down = false;
            if (keystate[SDL_SCANCODE_M]) //should the player press M then the manhatten distance is followed 
            {
                if ((pBot->PositionX() == gTarget.PositionX()) && (pBot->PositionY() == gTarget.PositionY())) continue; //should the bot be at the player already then there is no reason to build a path so continue will the code to next iteration
                //this means that until the bot has finished the path, a new path cannot be made
                if (pBot->atPlayer == false) continue; 
                if (!m_down)
                {
                    gAStar.Build(*pBot, 1); //takes a pointer reference to the bot and a number that assigns the heuristic to follow
                    m_down = true;
                    d_down = false;
                    e_down = false;
                }
                else m_down = false;
            }
            if (keystate[SDL_SCANCODE_E]) //should the player press E then the euclidean distance is followed
            {
                if ((pBot->PositionX() == gTarget.PositionX()) && (pBot->PositionY() == gTarget.PositionY())) continue;
                if (pBot->atPlayer == false) continue;
                if (!e_down)
                {
                    gAStar.Build(*pBot, 2);
                    e_down = true;
                    d_down = false;
                    m_down = false;
                }
            }
            else e_down = false;
            if (keystate[SDL_SCANCODE_D]) //should the player press D then the diagonal distance is followed
            {
                if ((pBot->PositionX() == gTarget.PositionX()) && (pBot->PositionY() == gTarget.PositionY())) continue;
                if (pBot->atPlayer == false) continue; //should bot not be at the player then wait until the bot is at player before a new heuristic can be chosen. 
                //This is because issues arised with the bots pathfinding when the user could pick what heuristic could be used while a path has not yet been completed
                if (!d_down)
                {
                    gAStar.Build(*pBot, 3);
                    d_down = true;
                    m_down = false;
                    e_down = false;
                }
                else d_down = false;
            }
        }
        if ((offsetX != 0) || (offsetY != 0))
        {
            gTarget.SetNext((gTarget.PositionX() + offsetX), (gTarget.PositionY() + offsetY), gLevel);
        }

        //======================================================================================
        //Start render for this frame
        //======================================================================================
        SDL_SetRenderDrawColor(gMainRenderer, 200, 200, 255, 255);
        SDL_RenderClear(gMainRenderer);

        //======================================================================================
        //Compute time in miliseconds of last update cycle
        //======================================================================================
        int newFrameTimeMS = SDL_GetTicks();
        deltaTimeMS = newFrameTimeMS - timerMS;
        if (deltaTimeMS < 0) deltaTimeMS = 0;
        timerMS = newFrameTimeMS;

        //======================================================================================
        //Update moving objects
        //======================================================================================
        gTarget.Update(deltaTimeMS);
        pBot->Update(deltaTimeMS);

        //======================================================================================
        //Draw the level grid, then target, then bot
        //======================================================================================
        gLevel.Draw();
        gTarget.Draw(targetTexture);
        pBot->Draw(botTexture);

        //======================================================================================
        //Finalise render for this frame
        //======================================================================================
        SDL_RenderPresent(gMainRenderer);
	}

    //======================================================================================
    //Clean up
    //======================================================================================
	SDL_DestroyWindow(gMainWindow);
	SDL_Quit();
	return 0;
}

