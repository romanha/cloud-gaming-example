Cloud Gaming Example
==========

Description
-----------

This project is a very basic example for cloud gaming.  
It shows the combination of different technologies to create a simple physics based 3D game, which will be run on a server.  
The rendered scene will be saved to a texture, encoded and sent via UDP to a client.  
The client will decode and display the received texture and send possible user input back to the server.  

Video link showing the final project: https://www.youtube.com/watch?v=spVV9t4IyTM

Technologies
------------

**Graphics engine:** Ogre3D - www.ogre3d.org  
**Physics engine:** Bullet Physics - www.bulletphysics.org  
**Video codec:** Xvid - www.xvid.org  
**Network protocol:** UDP

Game features
------------

Aiming with crosshair (aim with arrow keys).  
3 different cube sizes (change size with 1, 2 and 3).  
3 different throwing spots (circulate with TAB).  
Free movement mode (enabled as 4th throwing spot, activated per TAB) (control with WASDQE).  
Reset scenery (activated with R).  
Adjustable throwing force (hold SPACE to throw).  
Simple HUD showing crosshair, selected cube type, movement status and throwing force.
