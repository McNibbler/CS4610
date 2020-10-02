runoncepath("stager.ks").       // Credit to /u/nuggreat on reddit
runoncepath("math.ks").         // Credit to Nat Tuck
runoncepath("lib_slope.ks").    // Credit to ElWanderer on GitHub

set mapview to true.

// Warp to SOI
warpto(time:seconds + ship:orbit:nextpatcheta).
wait until body = body("Mun").
wait until warp = 0 and ship:unpacked.
wait 5. // Why is kOS really bad with this?

// Finds periapsis location
local peri_eta is ship:orbit:nextpatcheta / 2.

// Warps to periapsis if it's not in the middle of the mun
if (ship:orbit:periapsis > 10000) {
    warpto(time:seconds + peri_eta - 5).
    wait (peri_eta - 5).
}


// Makes it so that you're effectively falling straight down
lock steering to srfretrograde.
wait 3.

lock throttle to 1.0.
clearscreen.
until (ship:orbit:eccentricity > 0.999) and (ship:orbit:periapsis < 0) {
    print "Increasing eccentricity..." at (0,0).
    lock steering to srfretrograde.
    set throttle to 1.0.
}
set throttle to 0.
set mapview to false.

gear on.

lock steering to heading(90, 90).

// Why isn't g0 just a thing for all bodies smh
local mun_grav is constant:g * (body("Mun"):mass / ((body("Mun"):radius) ^ 2)).
lock ship_max_decel to (ship:availablethrust / ship:mass) - mun_grav.
lock suicide_distance to (ship:verticalspeed^2) / (ship_max_decel * 2).

lock ground_distance to min(ship:altitude - body("Mun"):geopositionof(ship:position):terrainheight - 5, alt:radar * 0.95).
lock land_time to ground_distance / abs(ship:verticalspeed).
local init_land_time is land_time.

wait 5.

// Turbo speed if you're really high up
until ground_distance < suicide_distance + 200000 {
    if ground_distance > 350000 {
        set warp to 5.
    }
    else {
        set warp to 2.
    }
}

// Absolutely destroy all horizontal speed (this uses a lot of fuel)
set warp to 0.
wait 5.
clearscreen.
until abs(ship:groundspeed) < 0.1 {
    print "Adjusting ground speed to near zero." at (0,0).
    print ship:groundspeed at (0,1).
    lock steering to srfretrograde.
    set throttle to 0.25.
}
set throttle to 0.

// Warp until 20k til burn
until ground_distance < suicide_distance + 20000 {
    set warp to 2.
}

clearscreen.
set warp to 0.

wait until ground_distance < suicide_distance + 300.

// very gently touches the ground
until (ground_distance < 1 and ship:verticalspeed > -1) or (ship:status = "LANDED") {
    print "Dist: " at (0,0).
    print "vSpd: " at (0,1).

    print ground_distance at (6,0).
    print ship:verticalspeed at (6,1).

    // surface retrograde prevents me from gaining horizontal speed
    lock steering to srfretrograde.

    if ship:verticalspeed < -1 {
        lock throttle to clamp(0.25, suicide_distance / ground_distance, 1).
    }
    else if ship:verticalspeed >= -0.1 {
        lock throttle to 0.
    }

}

// The end
lock throttle to 0.

clearscreen.
until ship:status = "LANDED" { 
    lock steering to slopedetails(ship:geoposition:lat, ship:geoposition:lng, 5)[3].
}
print "Landed!".







