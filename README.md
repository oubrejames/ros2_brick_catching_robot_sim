# ME495 Embedded Systems Homework 2
Author: James Oubre

This package simulates a robot that can detect a brick falling and go and catch it.

## Quickstart
1. Use `ros2 launch turtle_brick turtle_arena.launch.py ` to start the arena and turtle simulation
2. Use `ros2 service call /place turtle_brick_interfaces/srv/Place` to place a brick and `ros2 service call /drop std_srvs/srv/Empty` to drop a brick
3. Here is a video of the turtle when the brick is within catching range

   [Screencast from 10-20-2022 05:38:56 AM.webm](https://user-images.githubusercontent.com/46512429/196927253-b09a41a3-2822-457f-8609-e07d8078b034.webm)

4. Here is a video of the turtle when the brick cannot be caught
   
   [Screencast from 10-20-2022 05:39:30 AM.webm](https://user-images.githubusercontent.com/46512429/196927361-de74d4f9-5642-4af8-a7ba-a7e73ad32803.webm)

Worked With: Shantao Cao, David Dorf, Allan Garcia-casal , Ritika Ghosh , Katie Hughes, Elizabeth Metzger , Nicolas Morales, Marno Nel, Rintaroh Shima , Megan Sindelar , Ava Zahedi , Dilan Wijesinghe
