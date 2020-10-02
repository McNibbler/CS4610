// Credit to /u/nuggreat on reddit for this stage checker
function stage_check {
    parameter enable_stage is true.
    local need_stage is false.
    if enable_stage and stage:ready {
        local engine_list is list().
        list engines in engine_list.
        set need_stage to maxthrust < 0.1.
        for engine in engine_list {
            if need_stage { break.}
            set need_stage to need_stage or engine:flameout.
        }
        //if need_stage { stage. }
    }
    return need_stage.
}


// Persistent staging (by me)
when stage_check(true) then {
    wait 0.5.
    stage.
    preserve.
}
