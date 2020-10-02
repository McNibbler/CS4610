////////////////////////////////////////////
// Thomas Kaunzinger                      //
//                                        //
// CS 4610                                //
// 9/24/2020                              // 
// Launching a rocket into orbit          //
////////////////////////////////////////////

// Imports
runoncepath("stager.ks").   // Credit /u/nuggreat on reddit
set mapview to false.

// kOS is disgusting
clearscreen.
print "========== Orbit Script ==========" at (0, 3).

// Cute little status indicator
print "status: " at (0,0).
print "increasing throttle...        " at (8,0).
lock throttle to 1.0.
wait 1.

// Fun countdown ripped from the demo
print "countdown...                  " at (8,0).
from {local countdown is 3.} until countdown = 0 step {set countdown to countdown -1. } do {
    print countdown at (0,1).
    wait 1.
}
print "                    " at (0,1).


set warpmode to "PHYSICS".
set warp to 3.


// Rotate to 45 degrees
wait until ship:altitude > 7000.
print "rotating to 45 degrees...     " at (8,0).
set shipdirection to heading(90,90).
lock steering to shipdirection.
until ship:altitude > 10000 {
    local angle is 90 - (((ship:altitude - 7000) / 3000) * 45).
    set shipdirection to heading(90, angle).
    print round(angle) at (0,1).
}
print "                    " at (0,1).

// Increases the apoapsis to a safe height out of the atmosphere
print "increasing apoapsis height... " at (8,0).
until ship:apoapsis > 80000 {
    print round(ship:apoapsis) at (0,1).
}
set throttle to 0.0.

set warp to 0.
set warpmode to "RAILS".
wait 5.

set warp to 2.
wait until ship:altitude > 70000.
set warp to 0.
wait 5.
set mapview to true.

run circularize.    // Credit /u/gufoe on reddit

