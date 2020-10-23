# Homework 5
**Thomas Kaunzinger**
*CS4610*

## YouTube Video
`https://youtu.be/nbCPMU4FiXo`

## Comments
This program uses a strategic wall crawl
that also happens to ascertain which row the
robot currently resides in. This wall crawl
works by scanning the outer perimeter, and
when the robot detects it's inside the goal
row, it scans the entire row until it
reaches a wall, continuing the wall crawl
if the program does not terminate (goal
reached).

To achieve this, some measures were
necessary to get a vague sense of distance.
The robot achieves this by calibrating at
the beginning to find out the number of ticks
it takes to travel approximately one meter.

The robot can use this distance in order to
detect how far down some wall it had traveled
and can update the current row counter based
on if it's going east or west. It is also
used when the robot has left the proximity of
a wall and for aligning in the target row
scanning.

The target row scan ocurrs when the counter
updates to the target row, and the robot moves
a little bit into the middle of the row and
then scans the entire hall. This approach will
always work, as the wall crawl guarantees the
robot will end up in the hallway both on the
north and south sides, and since there is only
one wall per row, if the end is obstructed from
one direction, it is guaranteed to be reachable
from the other.
