# EngineForAnimationCourse
Graphic Engine based on Libigl

For compiling:
1. Clone or download the project
2. Download Cmake from the link in the site
3. Run Cmake gui. choose the project folder and destination folder for the cpp project files. choose after pressing configure choose compiler (VS2019 for example). After finish configuration successfully, press configure again and after it finishes press generate. 
4. If everything pass successfully got to the destination folder and launch the project. 
5. Copy configuration.txt from tutorial/sandBox to build/tutorial/sandBox sandBox as a startup project and compile the project (it could take few mineutes);   


About our project:
We made the 3D snake skinning and motion in game space-using the main keys: up, down, left, right. 
We also implemented two points of view – 
First, is a static top view(regular view). Second, is from head of the snake(moves dynamically with the snake movement).
Change of camera view is by using the keys: ']' or '['.

We created an interactive menu showing all the time the properties: 
current level, score, timer and pause/resume button.
when level achieved the menu showing a next level button("level up")  and restart button for the same level.
when the game over it show's the 'play again' button.
For each level the player must obtain a growing number of points before the timer is up, otherwise he lost the game. 

There are 4 movement types:
 - Basic movement linear movement (red color).
 - Bouncing and Gravity movement (blue color). 
 - Bezier curve (green color).
 - Static object (white color).
The player earns points by colliding with target objects (the implementation is using bounding object for each snake link).
For hitting(collide) of the snake in each target objects, we get a different number of points:  
Basic movement gives the smallest amount of points while the Bezier curve gives the highest score.
Points order: Basic << Bouncy and Gravity << Bezier.

- Special power- when the snake collides in the static object(white bunny). The snake get special power- he grows up and move faster for 5 seconds.
 (Special power is may be achieved only from level 3).
 
•	 When losing in any level- the game over and the player can start over from the first level.

Bonuses: 
1. moving target object according to Bezier curve.
2. Interactive user interface using ImGui.
3. special bunny target object, when snake collide in it the snake grow up, and move faster for 5 seconds.
4. sound when level complete, game over, when collide in a target and when a snake hit special bunny energy power.
5. special font for the Menu.  
6. gravity and bouncing objects.
7. cube map.
8. height map.
9. texture.

Difficulties:

We had several difficulties for each bonus and for the main project itself.
1.	One of our biggest struggles that took the most time is the entire dealing with the snake from the start of 'taking it apart' all the way through adding the second camera view. And understanding how to move the joints and the skinning.
2.	We also needed to learn and understand the ImGui classes, functions and callbacks in order to create a good looking and interactive menu.
3.	The Bezier curve algorithm was difficult to understand and implement, because in addition for the regular algorithm, we made the position and speed of the objects totally random and we made it as much as possible within the main frame of the game.
4.	For the shaders(cube map, height map) we needed to look for and learn from an online tutorial in order to make it all in the most professional way.
5.	For different bonuses we needed to download several different external files which took longer than expected due to our little experience with such things.
