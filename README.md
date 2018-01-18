# PID Controller Project Writeup
Self-Driving Car Engineer Nanodegree Program

---

[video0]: ./videos/Turn2-30mph-noadjust.mp4 "no adjustment"

This is an implementation of a PID controller for the Udacity Self-Driving Car Nanodegree Program.

This project implemented (in the order of development):

1. a PID controller
2. the Twiddle algorithm for optimization
3. Speed-dependent steering adjustment
4. Steering-dependent throttle adjustment

These will be explained below.

## Project Sections

### The PID Controller

The PID controller algorithm was implemented following the lecture implementation.  We update the PID controller with the cross track error (CTE) on every update.

There are two PID controllers that were implemented. The first controller is the steering controller, which determines the steering angle. This utilizes all of the hyperparameters.

The second controller was used for the throttle (to maintain a constant speed).  This was implemented only using the P parameter.


### The Twiddle implementation in the PID controller

The Twiddle algorithm was implemented using the basic outline given in the lectures.  However, the algorithm was implemented as a state machine.  This let me keep the interface the same (so the code in main.cpp stays roughly the same for training and for the actual run).

### Speed-dependent steering adjustment
During development, the car had difficulty at the last hard left turn (especially at higher speeds ~40mph).  This would show up as the car oscillating wildly around that turn.

The car successfully ran the track at ~30mph, but the oscillations are clearly visible.  These oscillations were occurring at the last sharp left turn.

[Video: Taking the last turn at 30mph with no adjustment](./videos/Turn2-30mph-noadjust.mp4)

At higher speeds, the oscillations occur all the way around the track.  This video starts just before entering the first turn after the start.

[Video: Oscillations at the first turn at 50mph with no adjustment](./videos/Turn1-40mph-noadjust.mp4)

Due to the high speeds and the control lag, the car would adjust for the turn, but it would take too long and the car would overshoot. And this would cause the car to oscillate.

As an attempt to fix this, as the car goes faster, the smaller the steer value allowed.

```
 // Adjust the steer value by the speed, the faster
 // the car is moving, the smaller the steer value allowed
 if (fabs(speed) > 11)
   steer_value = steer_value / (1.2*sqrt(fabs(speed-10)));

```

The car is now clearly more stable running through the track. Here is the second turn at 30 mph.

[Video: The last turn at 30mph with speed-dependent steer adjustment](./videos/Turn2-30mph-adjust1.mp4)

Here is the first turn at 50mph.  

[Video: The first turn at 50mph with speed-dependent steer adjustment](./videos/Turn2-50mph-adjust1.mp4)

However, the car is still taking the turns too quickly.  So the car will still overshoot the sharp turns.  To fix this, I turned to reducing the throttle when making a sharp turn in the next section.

### Steering-dependent throttle adjustment
Similarly, depending on how sharp a turn is, the car's throttle is adjusted.  This allows the car to run at higher speeds, since it makes the car slow down as it takes a turn.

This is clear in the sharp turns (the first hard left after the bridge and the other hard left at the end of the run).  At higher speeds (~60mph), the car gets very close to jumping the side of the road (and sometimes it does, but didn't in the video).

[Video: 60mph with only speed-dependent steer adjustment](./videos/Turns-60mph-adjust1.mp4)


This is implemented as a lookup table.

```
 // Depending on the steer value, adjust the throttle
 // (the tighter the turn the less throttle)
 // throttle-steer curve
 //                            0     1     2     3     4     5     6     7      8       9       10
 double throttle_adjust[] = { 1.00, 1.00, 0.70, 0.50, 0.30, 0.15, 0.10, 0.01, -0.001, -0.002, -0.007 };
 double throttle = speed_pid.TotalError();
 if (speed > 20)
   throttle *= throttle_adjust[static_cast<int>(fabs(constrained_steer_value*10))];
```

After the adjustment, here is the same turn

[Video: 60mph with steer-dependent throttle adjustment](./videos/Turns-60mph-adjust2.mp4)


This still has problems as it still enters the first turn after the bridge too fast (capturing the video also adds some lag to the response).  This throttle-steer curve was hand adjusted.

## Hyperparameter selection
The initial parameter set (with just the PID algorithm for steering) was selected using Twiddle with an initial parameter set (Kp, Ki, Kd) = (0.5, 0.0001, 1) with dp = (1, 1, 1). I let this run for 5000 steps and skipping the first 500 steps (the code has different values since I had to adjust as I switched to a lower resolution in the simulator).  5000 steps was enough to usually get one full lap of the track.

This gave me the following values:

```
	Kp: 0.603622 Ki: 0.0001 Kd: 9.92264
```

However I would still see oscillation at the last hard left turn.  To mitigate this, I reduced Kp and increased Kd.  This helped to decrease the amount of oscillation although it did tend to increase the error slightly.

To reduce the oscillation and increased the speed, I added the additional controls on the steering and throttle.  This helped, but still required some additional manual tweaking of the hyperparameters.

In the end, after the manual tweaking, the final hyperparameters chosen were:

```
    Kp: 0.453622 Ki: 0.00019805 Kd: 11.9
```

## Future improvements

The throttle-steer curve is broken up into 10 parts, so it's breaking the value into 10% buckets.  This leads to some large jumps in the throttle response, so it may be better to use more buckets (say 100 parts, so each bucket is 1%).

The current code adjusts the steer value and the throttle separately. Maybe it would be better to merge the throttle-steer and steer-speed controls into one 3-D.  Given the speed and the requested steer, provide the new throttle and steer values.

A basic problem with higher speeds is that this system is reactive.  We only react after we've entered the curve.  Given the steering lag, it makes it difficult to make the curve.  Some way of looking ahead for a curve would be helpful in slowing down before entering the curve (rather than after entering the curve).


