# CarND-Path-Planning-Project

## Goal of the project
The goal is to create a list of points separated by 0.02s that the simulated car will follow. The generated trajectory must respect several constraints:

- Safe driving: avoid collision with other cars
- Legal driving: stay below the speed limit (but close to it, except if trafic prevent it), drive in one of the 3 right lanes, not on the left side or outside of the road
- Confortable driving: avoid large jerk and accelerations
- Reasonable driving: stay in one lane if there is no need to change, but change lane (safely) when needed to overtake cars

## Overview of the model

On the top level, the model can be divided in two part. One part of the model implement a very basic state machine with four states:

- Normal mode: the car drives freely on one lane at maximum speed (or accelerating toward this speed;
- Car In Front mode: a car is detected in front of our car and close to it, in the same lane. In this mode, we will slow down to match (with a small margin) the speed of that car, and we will try to see if it can go in one of the overtake modes;
- Overtake left: if we are in "Car In Front Mode", check the left lane (if there is one) to see if it's safe to change lane to overtake. If it's possible, generate a smooth trajectory for it;
- Overtake right: same, but on the other side.

The second part of the code is the trajectory generation. This trajectory need several inputs:
- the previous trajectory (calculated in the previous step) or the car state, at the begining of the simulations
- the target speed (depending on the mode)
- the target road lateral position (the current lane, for normal and "car in front" modes, and something a little more advanced for the overtake modes

Let's dive a little more in the details of these two parts.

## State machine
The state machine is implemented between line 310 and 405. The state machine input is composed of two parts:

- our car state (position, velocity, and state machine mode)
- all the other car states

The output of the state machine is 
- the new mode
- the target speed
- the target lateral position (and the trajectory to follow in time for this position)

The initial mode is the Normal mode. In this mode, we will simply increase the speed to reach 49 mph (the cruise speed) and ask the trajectory generator to simply try to keep the current lateral position. The code for this mode is the speed change (line 404) and the transition checks to change mode (line 316 to 338), verifying if there is a car in front of us.

The state machine will only leave the normal mode if there is a car in front of us (actually if there is a car that will be in the future in front of our predicted trajectory) In that case, we transition to the next mode, the "Car In Front" mode. (Note: there could be a direct transition in the same cycle to one of the overtake mode, more on this later) In the "Car In Front" mode, we keep the same lane, but we change the target speed to reach the speed of the car in front of us (minus a margin). This is done to avoid a rear end collision, of course. From this mode, there are three possible outcomes: we stay in this mode (there is still a car in front and we still need to decelerate), we go back in normal mode (either the car changed lane, or we decelerated to much and we can accelerate again), or we move to one of the two overtake mode.

The code for the "car in front" mode is mainly the code checking if we can move to overtake (340 to 400, but some of those lines are more related to the overtake modes, it's a little bit mixed), and also the deceleration code (line 402)

This transition to overtake mode occurs only under two conditions:
- there is a lane on the target side (we don't go left if we are on the left lane...)
- the lane is "safe": there is no car at our level or in front of us in the target lane

When the transition to overtake mode occurs, the code remember the current S position of the car (actually not the current one, but the planned one at the end of the previous trajectory planning, but conceptually the idea is the same), and also the current speed (same remark) (lines 366-370 and lines 393-397) We also remember the current lane and the target lane. The s position and the speed will be used to generate a smooth line change trajectory (we will generate d as a function of s **d(s)**, changing over a distance depending on the speed. If the car is fast, the lane change will be over a larger s distance, to avoid large acceleration, but if the speed is low, we will do it over a smaller distance. The goal is to change lane in a small time)

Note that we don't compute this **d(s)** trajectory here, we just memorize everything so that the trajectory planner can access this information. The final part of the state machine is the code allowing us to leave the overtake modes, line 311-312. We go back in normal mode once we know that we have reach the end of the maneuver.

## Trajectory planner
The trajectory planner is the rest of the code, from line 409 to 490. It is based mainly on the idea discussed in the classroom, in the project walkthrough video.

The first thing that I do is to get the last two points of the previous trajectory plan (at the initial step, I just use the current car position as a "last point" and generate a fake "second to last" point using the yaw angle). I put these two points in a list (lines 409-433)

One note here: I will only generated a few new points, I mainly reuse the previously generated trajectory. This help to keep a smooth trajectory. That's why I use these last two points.

The next thing I do is to generate a few long term points (lines 436-448). I just use 3 values separated by 30 meters for the s values, and for the d values, I use either:
- a constant d in the center of the current lane, for Normal and "Car In Front" mode
- a d value function of the s (the think that I have called **d(s)** in the previous section), if I am in one of the overtake mode (lines 439-444)

I add theses long term points in the list containing the last two points of the previous trajectory, and I transform these lists into local car coordinate (instead of global (x,y) coordinates) to make computations easier. (452-458)

Lines 461 and 462, I create a spline function fitted on these points.

Now I need to use this spline to generated the desired points, taking into account the fact that they must be spaced by 0.02s (50 points a second) and that I have a target speed coming from the state machine. I need first to compute the dx corresponding to my target speed, 0.02s and my spline function. This is done with a linearisation and using pythagore (473-478), and then I use this dx and the spline to compute the (x,y) point (and there is also the transformation back from local car coordinate to global x,y coordinates)(lines 481 to 490)

That's it.

## Result

Here is a video of the result:

[![Result in video](https://img.youtube.com/vi/93Q2h11JhgE/0.jpg)](https://www.youtube.com/watch?v=93Q2h11JhgE)


## Possible improvements

- Refactor the state machine, could be three states, and the code could be more separated from the rest. And I could modularise the code of the different mode into different functions or methods.
- Implement an hysteresis between "Normal mode" and "Car in Front mode", to avoid toggling between the two when we are juste at the limit.
- Accelerate (or decelerate) faster. My current strategy for speed changing is really basic, there is room for improvements while still staying in the confort zone (you can see it with the initial acceleration, it could be a little faster)
- Implement a long term strategy for overtaking. The current simple algorithm is looking "one car ahead", and tries to change lane to avoid that car, without looking if there are cars further in the new selected lane (it only looks if it safe to change, not if it's good to increase the speed in the long run. It's a better strategy in average than staying in the lane behind one slow car, but it's not necessarly always a better strategy, and there is a lot of room for improvements here)
- Review the code architecture, use command line options to change parameters, ... (for now it's more a first prototype code, nothing really well organised, just "go enough" for small simulations)
