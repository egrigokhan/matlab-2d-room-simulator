# matlab-2d-room-simulator
A 2D room simulator which simulates point cloud sampling and depth map acquisition given a 2D floor plan

## Instructions

1. Change the file path `fp2.png` to the floor plan of your choice. The floor plan needs to be **grayscale** with **walls:black** and **space:white**.
2. Change the magic number `6` (**Line 26, main.m**) to the number of discrete wall segments in your floor plan. (I'll try and find a way to get this automatically, but in the mean time please hardcode it in)
3. Define the camera positions and camera directions/normals in `CP` (**Line 61, main.m**) and `CN` (**Line 65, main.m**) respectively to your desired camera positions and directions/normals.

61 65
   * Mixed
   * Mixed  
3. Item
