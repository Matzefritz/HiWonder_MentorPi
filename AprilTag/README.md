For the use of Apriltags, the Tag-pose (Transform between Tag and world-frame) is epected to be prior knowledge. The transformation from Camera to Tag then can be extended to a Transformation from Camera to the world-frame.

The Tag-poses of every Tag is listed in tags.yaml. The orientation is given in quaternions. 
This array of Tag-Poses can be created with the maze_builder.py in /maze_setup.

The settings.yaml referrs to the AprilTag-Settings. The current settings should work, but feel free to play around!
