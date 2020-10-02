// Credit to /u/gufoe for writing a better circularizer than me

print "Waiting apoapsis".
lock steering to heading(90,0). // Look at east (90), zero degrees above the horizon
wait eta:apoapsis-.1. // Wait to reach apoapsis
lock throttle to 1. // Full power

set oldEcc to orbit:eccentricity.
until (oldEcc < orbit:eccentricity) { // Exists when the eccentricity stop dropping
    set oldEcc to orbit:eccentricity.
    
    set power to 1.
    if (orbit:eccentricity < .1) {
        // Lower the power when eccentricity < 0.1
        set power to max(.02, orbit:eccentricity*10).
    }
    
    // Radius is altitude plus planet radius
    set radius to altitude+orbit:body:radius.
    
    // Gravitational force
    set gForce to constant:G*mass*orbit:body:mass/radius^2.
    
    // Centripetal force
    set cForce to mass*ship:velocity:orbit:mag^2/radius.
    
    // Set total force
    set totalForce to gForce - cForce.
    
    // Current stage ended?
    until (maxThrust > 0) {
        stage.
    }
    set thrust to power*maxThrust.
    
    // Check if the thrust is enough to keep the v. speed at ~0m/s
    if (thrust^2-totalForce^2 < 0) {
        print "The vessel hasn't enough thrust to reach a circular orbit.".
        break.
    }
    
    // The angle above the horizon is the angle 
    set angle to arctan(totalForce/sqrt(thrust^2-totalForce^2)).
    
    // Adjust new values for throttle and steering
    lock throttle to power.
    lock steering to heading(90,angle).
    
    // Print stats
    clearscreen.
    print "Attraction:  "+gForce.
    print "Centripetal: "+cForce.
    
    // Wait one tenth of a second
    wait .1.
}

// Shut down engines
lock throttle to 0.
print "Orbit reached, eccentricity: "+orbit:eccentricity.
