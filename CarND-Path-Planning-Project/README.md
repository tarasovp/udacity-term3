# Path planning project

## 1)  I've just three states - following the lane at max speed, go left and go right.  

Of corse for good moving in a trafic we need more states - at least avaliabilitry to slow down and then change lane. Because otherwise we can stuck in the line while other line will be free.

## 2) How to decide to change the lane?

### I've very simple solution - find the line where the next car is farther and chose it.
### How it have to be done - using hybrid A* algoritm we can select a complex path between cars, and sometimes it can be better to use other path, not one from my "greedy" strategy

## 3) How to find a trajectroy to change the line?

### Well, I've awfull solution from example video. Just using splines and points with s+30 coordinates fina a smooth trajectory.
### How it have to be done - using jerk minimizing trajectories and cost functions for each trajectory. I've made it in Python only (see py folder for some drafts), because it takes too much time to debug in in c++ using only simulator.

## 4) How to find best speed for the line?

### I've found it using very simple conditions. We can go as fast, as possible, but we have to be able to slow down to next car speed not breaking max acceleration limit. Se the solution in get_max_speed function. 

### Of course, if using jerk minimizing trajectories from part 3 we will have better conditions for maximal speed when changin lines

# Video - https://youtu.be/BzsIvYI_Bi4
