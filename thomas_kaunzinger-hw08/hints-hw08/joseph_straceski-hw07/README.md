
# joseph_straceski-hw07

This code is from https://github.com/jstraceski/cs5335hw-gazebo/tree/hw07

Joseph says:

Here are the caveats: My code is a bit messy, additionally my code works best
with the previous implementation of the error model for drive systems because
the variance from the last state is at max an error of about 0.2-0.4 rather than
the randomly based one which could produce a max variance of 4 (from -2 to 2).
Because of how random() works.

Additionally I’m not sure that my implementation is the best method for
attacking this problem. I very much tuned my error correction to the type of
error present in the model. Although this can be done in the real world as well.

I believe that the error model presented, especially for the drive motors made
dead reckoning very difficult unless you could compensate for it like i did.

# Gazebo Simulation HW

This contains a simulation setup and sample control program for assignments in a
university robotics course.

This requires [Gazebo](http://gazebosim.org/) and is intended to work on a
(non-virtualized) Ubuntu 20.04 or Debian 10 system.
