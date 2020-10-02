// Angle logic by Nat Tuck from lecture

//clearscreen.
//until False {
//    set a_m to absang(body("Mun"):orbit).
//    set a_s to absang(ship:orbit).
//    print a_m at (0, 2).
//    print a_s at (0, 3).
//    print ang_diff(a_m, a_s) at (0, 4).
//}

function norm {
    parameter ang.
    return mod(ang, 360).
}

function absang {
    parameter obt.
    return mod(obt:lan + obt:argumentofperiapsis + obt:trueanomaly, 360).
}

function ang_diff {
    parameter a0.
    parameter a1.
    set a0 to norm(a0).
    set a1 to norm(a1).
    if (a0 > a1) {
        return a0 - a1.
    }
    else {
        return a0 - a1 + 360.
    }
}
