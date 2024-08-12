# gam3r - Glitchy Animation Machine 3D Renderer

Using pygame to make a 3d renderer with free move camera and scene building.
Main goal is to be able to build a robot game, hence Machine in the module title.


# Features Developed:

- Engine class (store member variables to use for game state)
- Constants file
- Game loop
- Startup function (pygame screen)
- Camera:
    - Camera definition (position, orientation, FOV)
    - Camera movement functions (translation, relationship between tilt/turn and direction vectors)
    - Camera movement inputs (pairs of keys to move front back, left right, up down)
    - Camera orbiting around a point (distance, rotation matrix around a point)
- Render Pipeline:
    - Get triangles from the world list
    - Clip the triangles based on the camera planes
    - Remove any triangles not facing the camera
    - Calculate and sort triangles based on distance from center of the triangle to the camera
    - Convert the triangle coordinates from the 3D world frame to the 2D camera frame
    - Update the lighting of the triangle if the light or triangle has changed position
    - Draw the triangle using draw.polygon()
- Keyboard and mouse input handling
- UI Buttons and Text updates
- Simple objects composed of triangles
    - making them by hand
    - loading OBJ files
    - populate_world() function to load all the objects
- Complex objects with motion: Robots
    - Defining the joints, arm lengths, and angles
    - Generate the mesh with forward kinematics (joint angles, arm positions)
    - Controlling the robot with the user inputs (swinging joints, making joint speed more intuitive by making the bigger joints move slower)
- Improving performance (making a basic profiling system)
- Making a simple game with the engine.