--------------------------------------------------------------------------------------------------------------------------------------------
Name : Vishak Nag Ashoka
---------------------------
USC ID : 9791117566
---------------------------
COURSE : CSCI599
---------------------------
SUBMITTED ON : 02/07/2011
---------------------------

--------------------------------------------------------------------------------------------------------------------------------------------
SUBMISSION PACKAGE CONSISTS OF THE FOLLOWING
--------------------------------------------------------------------------------------------------------------------------------------------

	> 	Animation Jpegs.zip - consists of al the 300 jpeg images for the animation
	>	README		 - How to execute my implementation
	>	599JelloCube.zip	 - Complete project package zipped

--------------------------------------------------------------------------------------------------------------------------------------------
IMPLEMENTATION
--------------------------------------------------------------------------------------------------------------------------------------------

	-	A soft and fantastic Jello Cube is implemented by constructing a mass points network consisting of 512 nodes. 
	There are three kinds of springs namely, structural, shear and bend. 

	My implementation works for the all the files
	rotate.w - 	using the default values provided in the starter package
	moveleft.w - 	using the default values provided in the starter package
	gravity.w - 	using the default values provided in the starter package
	skewedCorner - 	using the default values provided in the starter package
	jello.w - 		using my values of the hooks constant and damping constant 
			which are quite different from what is provided in the starter package.
			It works for the default values as well but I feel it works best with my values 
			for the constants. 
			
			jello->kElastic=475;
  
			jello->dElastic=0.25;
  
			jello->kCollision=80.0;
			jello->dCollision=0.4;

	vishak.w -	This is my own world file which I have submitted with my solution on the blackboard.
			
--------------------------------------------------------------------------------------------------------------------------------------------
WHAT HAVE I DONE
--------------------------------------------------------------------------------------------------------------------------------------------

	>	I have Implemented the mass spring network.
	>	Texture mapped the walls of the bounding box.
	>	Texture mapped the surface of the jello cube
	>	Implemented the collision detection with the walls of the bounding box and the response 
		of the jello cube for the collision
	>	Implemented the Force field computation through trilinear interpolation of the force vectors
		in the force field.
	>	I have placed a sphere object in my scene and a collision detection is implemented between
		the jello cube and the sphere
	>	I worked with Recep siyli and we both got the idea of scaling the jello cube dynamically,
		So we both have implemented the scaling of the jello cube. It is possible to increase the size of
		the jello cube and also to decrease its size dynamically.
	>	I have implemented the menu for controlling the simulation, this menu is done using the GLUT 
		APIs. I have a on screen menu used to select the input file for the simulation, lighting in the
		simulation, toggling the force field and rendering the jello cube in either wireframe of triangle mode
	>	As soon as the force field is activated through the screen menu, the texture mapped onto the 
		bounding box will display a message saying that "FORCE FIELD ACTIVATED"

--------------------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------------------
HOW TO EXECUTE MY IMPLEMENTATION
--------------------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------------------

--------------------------------------------------------------------------------------------------------------------------------------------
							MOUSE CONTROLS
--------------------------------------------------------------------------------------------------------------------------------------------

	>	Once the solution is compiled and executed
		
		CONTROLS

		MOUSE-LEFT-CLICK	-	MENU APPREARS ON THE SCREEN
		
		Following is the hierarchy of the controls present in the main menu and their description 

		MAIN MENU
			WORLD FILES	
				moveleft.w	- Selecting any of the following files will make the simulation stop running with the current 
				rotate.w		- world file and the simulation will automatically restart with the new world file selected
				jello.w		
				gravity.w
				vishak.w

			LIGHTING
				Shininess		- This control option toggles the shininess property of the light sources to toggle ON/OFF
				Specularity	- This control option toggles the specularity property of the light sources to toggle ON/OFF
				Emissivity	- This control option toggles the emissivity property of the light sources to toggle ON/OFF

			FORCE FIELD
				SWITCH ON	- Option used to switch on the force field , This means that the code for interpolating the
						- forces in the force field is invoked
						- Scene displays the message that "FORCE FIELD ACTIVATED" on the bounding box walls
				SWITCH OFF	- Option used to switch off the force field.

			RENDERING
				Wireframe	- Option used to change the jello cube rendering in spring network with the mass points
				Triangle Mode	- Option used to display the jello cube with the triangles being filled and texture mapped.

--------------------------------------------------------------------------------------------------------------------------------------------
						KEYBOARD CONTROLS
--------------------------------------------------------------------------------------------------------------------------------------------

	>	Scaling of the jello cube can be controlled using the following keys

	key 'r'	-	increases the scale of the jello cube by 10% 
	key 'd'	-	decreases the scale of the jello cube by 10%

	>	The Sphere in the scene can be removed completely with the collision effects and the response due to it
		by using the following keys

	key 'o'	-	Toggles the appearance and effects of the sphere present in the scene.

--------------------------------------------------------------------------------------------------------------------------------------------
						THANKS A MILLION TO
--------------------------------------------------------------------------------------------------------------------------------------------

>	Prof. Jernej Barbic
>	TA Yili Zhao
--------------------------------------------------------------------------------------------------------------------------------------------


	

		