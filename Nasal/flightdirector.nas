 #############################################################################
#
# Citation Bravo Flight Director/Autopilot controller.
#
# Written by Syd Adams
#Modification of Curtis Olson's flight director.
# Started 30 Jan 2006.


#
#############################################################################
# 1 meter = 0.000539956803 nm
#
#
#############################################################################
# Global shared variables
#############################################################################

# 0 - Off: v-bars hidden
# lnav -0=off,1=HDG,2=NAV,3=APR,4=BC
# vnav - 0=off,1=GS,2=ALT SELECT,3=VS,4=IAS


lnav = 0;
vnav=0;
lnav_last = 0;
vbar_roll = 0.0;
vbar_pitch = 0.0;
vbar_rol_propl = 0.0;
vbar_pitch_prop = 0.0;
nav_dist = 0.0;
last_nav_dist = 0.0;
last_nav_time = 0.0;
tth_filter = 0.0;
alt_select = 0.0;
current_alt=0.0;
current_heading = 0.0;
n_offset = 0.0;
alt_offset = 0.0;
fd_on = 0;
ap_on = 0.0;
alt_alert = 0.0;
course = 0.0;
course_offset=0.0;
nav_hdg_offset=0.0;
nav_mag_brg=0.0;
gs_active = 0.0;
to_flag = 0;
slaved = 0;
#############################################################################
# Use tha nasal timer to call the initialization function once the sim is
# up and running
#############################################################################

INIT = func {
    # default values
    print("Initializing Flight Director");
    setprop("/instrumentation/flightdirector/lnav", 0.0);
    setprop("/instrumentation/flightdirector/vnav", 0.0);
    setprop("/instrumentation/flightdirector/vbar-pitch", 0.0);
    setprop("/instrumentation/flightdirector/vbar-roll", 0.0);
    setprop("/instrumentation/flightdirector/alt-offset", 0.0);
    setprop("/instrumentation/flightdirector/autopilot-on",0.0);
	setprop("/instrumentation/flightdirector/fd-on",0.0);
    setprop("/instrumentation/flightdirector/alt-alert", alt_alert);
    setprop("/instrumentation/flightdirector/course", 0.0);
    setprop("/instrumentation/flightdirector/dtk", 0.0);
    setprop("/instrumentation/flightdirector/nav-hdg", 0.0);
    setprop("/instrumentation/flightdirector/gs-pitch", 0.0);
    setprop("/instrumentation/flightdirector/nav-mag-brg", 0.0);
    setprop("/instrumentation/flightdirector/course-offset", course_offset);
    setprop("/instrumentation/flightdirector/target-inhg", 29.92);
    setprop("/instrumentation/flightdirector/to-flag",0);
    setprop("/instrumentation/flightdirector/from-flag",0);
    setprop("/autopilot/settings/heading-bug-deg",0);
    setprop("/autopilot/settings/target-altitude-ft",0);
    setprop("/instrumentation/nav/slaved-to-gps",slaved);
current_alt = getprop("/instrumentation/altimer/indicated-altitude-ft");
alt_select = getprop("/autopilot/settings/target-altitude-ft");
}
settimer(INIT, 0);


#############################################################################
# handle KC 290 Mode Controller inputs, and compute correct mode/settings
#############################################################################

handle_inputs = func {
# Autopilot  activate
    lnav = getprop("/instrumentation/flightdirector/lnav");
    vnav = getprop("/instrumentation/flightdirector/vnav");
   ap_on = getprop("/instrumentation/flightdirector/autopilot-on");

if(ap_on==1){
if(lnav == 0 or lnav ==nil){setprop("autopilot/locks/heading","wing-leveler");}
if(lnav == 1){setprop("autopilot/locks/heading","dg-heading-hold");}
if(lnav == 2){setprop("autopilot/locks/heading","nav1-hold");}
if(lnav == 3){setprop("autopilot/locks/heading","appr-hold");}
if(lnav == 4){setprop("autopilot/locks/heading","bc-hold");}

if(vnav == 0 or vnav == nil){setprop("autopilot/locks/altitude","");}
if(vnav == 1){if(getprop("/instrumentation/nav/has-gs")!=0){
setprop("autopilot/locks/altitude","gs1-hold");}}
if(vnav == 2){setprop("autopilot/locks/altitude","altitude-hold");}
if(vnav == 3){setprop("autopilot/locks/speed","vertical-speed-hold");}
if(vnav == 4){setprop("autopilot/locks/speed","speed-with-throttle");}
if(vnav == 5){setprop("autopilot/locks/speed","dcs-hold");}
if(vnav == 6){setprop("autopilot/locks/speed","climb-hold");}
maxroll = getprop("/orientation/roll-deg");
if(maxroll > 45 or maxroll < -45){ap_on = 0;}
maxpitch = getprop("/orientation/pitch-deg");
if(maxpitch > 45 or maxpitch < -45){ap_on = 0;}
#if(getprop("/position/altitude-agl-ft") < 50){ap_on = 0;}
setprop("/instrumentation/flightdirector/autopilot-on",ap_on);
}else{
setprop("autopilot/locks/heading","");
setprop("autopilot/locks/altitude","");
setprop("autopilot/locks/speed","");
  }
}


#############################################################################
# track and update mode
#############################################################################

update_mode = func {
    lnav = getprop("/instrumentation/flightdirector/lnav");

    # compute elapsed time since last iteration
    nav_time = getprop("/sim/time/elapsed-sec");
    nav_dt = nav_time - last_nav_time;
    last_nav_time = nav_time;

    inrange = getprop("/instrumentation/nav/in-range");
    if ( inrange ) {
        # compute distance to nav heading intercept
        nav_dist = getprop("/instrumentation/nav/crosstrack-error-m");

        # compute time to heading (tth)
        nav_rate = (last_nav_dist - nav_dist) / nav_dt;
        if ( abs(nav_rate) > 0.00001 ) {
            tth = nav_dist / nav_rate;
        } else {
            tth = 9999.9;
        }
      #  print("nav-dist = ", nav_dist, " tth = ", tth);

        tth_filter = 0.9 * tth_filter + 0.1 * tth;
        last_nav_dist = nav_dist;
    }        

    if ( lnav == 2 ) {
        curhdg = getprop("/orientation/heading-magnetic-deg");
        tgtrad = getprop("/instrumentation/flightdirector/nav-hdg");
        if ( tgtrad == nil or tgtrad == "" ) {
            tgtrad = 0.0;
        }
        diff = tgtrad - curhdg;
        if ( diff < -180.0 ) {
            diff += 360.0;
        } elsif ( diff > 180.0 ) {
            diff -= 180.0;
        }

        # standard rate turn is 3 dec/sec
        roll_out_time_sec = abs(diff) / 3.0;

      #  print("tth = ", tth_filter, " hdgdiff = ", diff, " rollout = ", roll_out_time_sec );
        if ( roll_out_time_sec >= abs(tth_filter) ) {
            # switch from arm to cpld
           # lnav = 3;
        }

    }

setlistener("/sim/signals/fdm-initialized", func {
    setprop("/instrumentation/flightdirector/lnav", lnav);
});
}

#############################################################################
#get pitch from autopilot altitude setting
#############################################################################

get_altpitch = func(){
vnav = getprop("/instrumentation/flightdirector/vnav");
current_pitch = getprop("/orientation/pitch-deg");
if(vnav == 0){return(current_pitch);}
alt_offset = 0.0;

if(vnav == 6){return(5.0);}
if(vnav == 5){return(0.0);}
if(vnav == 4){return(0.0);}
if(vnav == 3){return(getprop("/autopilot/settings/vertical-speed-fpm"));}
if(vnav == 1){alt_select = getprop("/autopilot/settings/target-alt-hold");}
if(vnav == 2){alt_select = getprop("/autopilot/settings/target-altitude-ft");}
if(lnav == 3){return(getprop("/instrumentation/flightdirector/gs-pitch"));}

if ( alt_select == nil or alt_select == "" ){ alt_select = 0.0;
return(current_pitch);}

current_alt = getprop("/position/altitude-ft");
if(current_alt == nil){current_alt = 0.0;}
alt_offset = (alt_select-current_alt);
setprop("/instrumentation/flightdirector/alt-alert",alt_offset);
if(alt_offset > 500.0){alt_offset = 500.0;}
if(alt_offset < -500.0){alt_offset = -500.0;}
return(alt_offset * 0.012);
}

#############################################################################
#update nav gps or nav setting
#############################################################################

update_nav = func (){
slaved = getprop("/instrumentation/primus1000/dc550/fms");
current_heading = getprop("/orientation/heading-magnetic-deg");
if(slaved == nil){slaved = 0};

if(slaved == 0){
setlistener("/sim/signals/fdm-initialized", func {
    setprop("/instrumentation/flightdirector/to-flag",getprop("/instrumentation/nav/to-flag"));
});
desired_course = getprop("/instrumentation/nav/radials/selected-deg");
course_offset = getprop("/instrumentation/nav/heading-needle-deflection");
nav_mag_brg = getprop("/instrumentation/nav/heading-deg");
if (getprop("/instrumentation/nav/has-gs") != 0){
gs_offset=getprop("/instrumentation/nav/gs-needle-deflection");
if(gs_offset == nil){gs_offset = 0};
gs_active = gs_offset *1.0; 
if(gs_active > 30.0){gs_active = 30.0};
if(gs_active < -30.0){gs_active = -30.0};
setprop("/instrumentation/flightdirector/gs-pitch",gs_active * 100);}
}
else
{
setprop("/instrumentation/flightdirector/to-flag",getprop("/instrumentation/gps/wp/wp[1]/to-flag"));
desired_course = getprop("/instrumentation/gps/wp/wp[1]/desired-course-deg");
if(desired_course == nil){desired_course=0;}
desired_course -= getprop("/environment/magnetic-variation-deg");
nav_mag_brg = getprop("/instrumentation/gps/wp/wp[1]/bearing-mag-deg");
if(desired_course < 0){desired_course += 360;}
elsif(desired_course > 360){desired_course -= 360;}
course_offset = getprop("/instrumentation/gps/wp/wp[1]/course-deviation-deg");
if(course_offset > 10.0){course_offset = 10.0;}
if(course_offset < -10.0){course_offset = -10.0;}
}
setprop("/instrumentation/flightdirector/dtk",desired_course);

if(nav_mag_brg == nil){nav_mag_brg = 0;}
nav_mag_brg -= current_heading;
if(nav_mag_brg > 180){nav_mag_brg -= 360};
if(nav_mag_brg < -180){nav_mag_brg += 360};
#########    set radial offset from current heading ###########
desired_course -= current_heading;
if(desired_course < -180){desired_course += 360;}
elsif(desired_course > 180){desired_course -= 360;}
setprop("/instrumentation/flightdirector/course",desired_course);

##### adjust autopilot nav heading with deviation ###########
nav_adjust = ( course_offset * 4.5);
nav_hdg_offset = desired_course + nav_adjust;
if(nav_hdg_offset < -180){nav_hdg_offset += 360;}
elsif(nav_hdg_offset > 180){nav_hdg_offset -= 360;}


setprop("/instrumentation/flightdirector/nav-mag-brg",nav_mag_brg);
setprop("/instrumentation/flightdirector/course-offset",course_offset);
setprop("/instrumentation/flightdirector/nav-hdg",nav_hdg_offset);



}


#############################################################################
# update the FD vbar position for the various modes
#############################################################################

update_vbar = func {
    if ( lnav == 0 ) {
        # wings level maintain pitch at time of mode activation
        if ( lnav_last != 0 ) {
            vbar_roll = 0.0;
           vbar_pitch = getprop("/orientation/pitch-deg");
}
    } elsif ( lnav == 1) {
        # FIXME: at what angle off of the hdg bug do we start the rollout?
        # bank to track heading bug

        if ( lnav_last != "hdg" ) {

        }
        tgtrad = getprop("/autopilot/settings/heading-bug-deg");
        if ( tgtrad == nil or tgtrad == "" ) {
            tgtrad = 0.0;
        }
        curhdg = getprop("/orientation/heading-magnetic-deg");
        diff = tgtrad - curhdg;
        if ( diff < -180.0 ) {
            diff += 360.0;
        } elsif ( diff > 180.0 ) {
            diff -= 180.0;
        }
        # max bank = 30, so this means roll out begins at 15 dgs off target hdg
        bank = 2 * diff;
        if ( bank < -30.0 ) {
            bank = -30.0;
        }
        if ( bank > 30.0 ) {
            bank = 30.0;
        }
        vbar_roll = bank;
#        print("diff = ", diff);
} elsif ( lnav == 2 or lnav == 3) {
tgtrad = getprop("/instrumentation/flightdirector/nav-hdg") * 3;
        
if(tgtrad > 30){tgtrad = 30;} 
if(tgtrad < -30){tgtrad = -30;} 

vbar_roll = tgtrad;

    } else {
        # assume off if nothing else specified, and hide vbars
        vbar_roll = 0.0;
    }
vnav = getprop("/instrumentation/flightdirector/vnav");
if (vnav != 0 or vnav !=nil){vbar_pitch =  get_altpitch();}

lnav_last = lnav;
vbar_pitch_prop = (vbar_pitch - getprop("/orientation/pitch-deg"));
vbar_roll_prop = (getprop("/orientation/roll-deg") - vbar_roll);
if(vbar_roll_prop > 30.0){vbar_roll_prop = 30.0;}
if(vbar_roll_prop < -30.0){vbar_roll_prop = -30.0;}
if(vbar_pitch_prop > 15.0){vbar_pitch_prop = 15.0;}
if(vbar_pitch_prop < -15.0){vbar_pitch_prop = -15.0;}


setlistener("/sim/signals/fdm-initialized", func {
    setprop("/instrumentation/flightdirector/vbar-pitch",vbar_pitch_prop);
    setprop("/instrumentation/flightdirector/vbar-roll", vbar_roll_prop);
    setprop("/instrumentation/flightdirector/alt-offset", alt_offset);
    setprop("/instrumentation/flightdirector/current-alt", current_alt);
});
}


#############################################################################
# main update function to be called each frame
#############################################################################

update = func {
    handle_inputs();
    update_mode();
    update_nav();
   update_vbar();
 
    registerTimer();
}



#############################################################################
# Use tha nasal timer to call ourselves every frame
#############################################################################

registerTimer = func {
    settimer(update, 0);
}
registerTimer();
