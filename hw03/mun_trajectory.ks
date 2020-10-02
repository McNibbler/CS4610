// Executes the orbital transfer from kerbin to the mun

runoncepath("stager.ks").   // Credit /u/nuggreat on reddit
runoncepath("angles.ks").   // Credit Nat Tuck

set mapview to true.

clearscreen.

// Silly little screen clearing
declare local function reset_screen {
    clearscreen.
    print "status: " at (0, 0).
    print "========== Mun Trajectory ==========" at (0, 3).
}

reset_screen().


// Finds the altitude of the mun that we want the apoapsis to be
local target_apo is Body("Mun"):altitude. 

// Creates a node that determines how long it takes to get the apoapsis to that height

// Start with approximately correct delta v and then damp down to apoapsis = target
// then get time to apo with that orbit
print("finding delta v...") at (8, 0).

// Brainblasts my way to the right delta v by using a damping function
local running_delta_v is 860.
local running_node is node(eta:periapsis, 0, 0, running_delta_v).
add(running_node).
local running_apo is running_node:orbit:apoapsis.
until abs(running_apo - target_apo) <= 100000 {
    if running_apo > target_apo {
        set running_delta_v to (running_delta_v - 0.01).
    }
    else {
        set running_delta_v to (running_delta_v + 0.01).
    }
    remove(running_node).
    set running_node to node(eta:periapsis, 0, 0, running_delta_v). 
    add(running_node).
    set running_apo to running_node:orbit:apoapsis.
    print("dv:                  ") at (0, 1).
    print("apo:                 ") at (0, 2).

    print round(running_delta_v) at (4, 1).
    print round(running_apo) at (5, 2).
}
local travel_time is (running_node:orbit:period / 2).
remove(running_node).

clearscreen.

// Finds the position of the mun at the time that it would take to reach apoptosis
local mun_init_angle is absang(body("Mun"):orbit). 
local mun_target_angle is mod(((360 * (travel_time / body("Mun"):orbit:period)) + mun_init_angle), 360).

// creates a schmaneuver node at the position around kerbin that will push the apoapsis to that location
local ship_target_angle is mod((mun_target_angle + 180), 360).
local ship_current_angle is absang(ship:orbit).
local ship_deg_to_target is ang_diff(ship_target_angle, ship_current_angle).
local time_for_node is (ship:orbit:period * (ship_deg_to_target / 360)).

set running_node to node(time:seconds + time_for_node, 0, 0, running_delta_v).

// executes the schmaneuver node
warpto(max(time:seconds, (time:seconds + time_for_node) - 30)).
add(running_node).
run exec_node.

