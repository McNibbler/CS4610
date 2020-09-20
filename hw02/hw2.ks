////////////////////////////////////////////
// Thomas Kaunzinger                      //
//                                        //
// CS 4610                                //
// Homework 2                             //
// 9/24/2020                              // 
// Launching a rocket into circular orbit //
////////////////////////////////////////////

// kOS is disgusting
clearscreen.

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

// Begin staging the rocket and launching
lock steering to up.
when maxthrust = 0 then {
    print "staging...                    " at (8,0).
    wait 0.5.
    stage.
    preserve.
}

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

// This happens pretty much immediately after the rotation step
wait until stage:solidfuel < 0.1.
stage.

// Increases the apoapsis to a safe height out of the atmosphere
print "increasing apoapsis height... " at (8,0).
until ship:apoapsis > 80000 {
    print round(ship:apoapsis) at (0,1).
}
set throttle to 0.0.

// Keeps heading parallel to planet
print "approaching apoapsis...       " at (8,0).
set shipdirection to heading(90,0).

// Waits until approaching periapsis
// The wait was approximated using the maneuver tool
wait until eta:apoapsis < 35.
print "increasing periapsis height..." at (8,0).
until ship:periapsis > min(80000, ship:altitude - 1000) {
    set shipdirection to heading(90,0).
    lock throttle to 1.0. 
    print round(ship:periapsis) at (0,1).
}
lock throttle to 0.0.

print "approaching periapsis...      " at (8,0).
print "                    " at (0,1).

// Corrects the apoapsis
// The wait was approximated using the maneuver tool
wait until eta:periapsis < 5.
print "flipping to retrograde...     " at (8,0).
set shipdirection to ship:retrograde.
wait 1.5.
print "correcting apoapsis...        " at (8,0).
until abs(ship:periapsis - ship:apoapsis) < 1000 {
    set shipdirection to ship:retrograde.
    local throt is min(1.0, 1 - ship:periapsis / ship:apoapsis).
    lock throttle to throt.
    print "Apo: " at (0,1).
    print round(ship:apoapsis) at (5,1).
    print "Per: " at (0,2).
    print round(ship:periapsis) at (5,2).
}
print "                    " at (0,1).
print "                    " at (0,2).

lock throttle to 0.

print "circularization succesful!   " at (8,0).

until false {
    print "Apo: " at (0,1).
    print round(ship:apoapsis) at (5,1).
    print "Per: " at (0,2).
    print round(ship:periapsis) at (5,2).
    wait 1. 
}












