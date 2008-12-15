 #############################################################################
#
# Citation Bravo Flight Director/Autopilot controller.
#
# Written by Syd Adams
#Modification of Curtis Olson's flight director.
# Started 30 Jan 2006.
#Modified by AirAlex (Dec 2007 - Feb 2008)


#
#############################################################################
# 1 meter = 0.000539956803 nm
#
#
#############################################################################
# Global shared variables
#############################################################################

# 0 - Off: v-bars hidden
# lnav -0=off,1=HDG,2=NAV,3=APR,4=BC,5-ROUTE
# vnav - 0=off,1=GS,2=ALT SELECT,3=VS,4=IAS

#trigonometric values for glideslope calculations
FD_TAN3DEG = 0.052407779283;
FD_SIN3DEG = 0.052335956243;
FD_COS3DEG = 0.998629534755;

FD_TAN15DEG = 0.2679491924311227332;
FD_SIN30DEG = 0.5;
FD_TAN2_5DEG = 0.04366094290851206068;
FD_SIN5DEG = 0.08715574274765816587;

DEG2RAD = 0.01745329251994329509;
RAD2DEG = 57.29577951308232311;
PI = 3.141592653589793116;
PI2 = 1.570796326794896558;
TWOPI=6.283185307179586232;

MSEC2KT=1.9438;
KT2MSEC=0.514;
MSEC2FPM=196.85;
FPM2MSEC=0.00508; 
MSEC2KMH=3.6;
KMH2MSEC=0.28;
NM2MTRS = 1852;
lur_koeff1 = 5.661872017348443498;#=g*tan(30deg)
tas_koeff = 0.0;
H760_m = 0.0;

turn_radius_m = 0.0;
turn_dist=0.0;
lur_m=0.0;
lur_nm = 0.0;
W_kt = 0.0;
wind_angle = 0.0;
wind_speed_kt = 0;
z_nm = 0.0;
TRK_M = 200;

US=0.0;#angle of drift
KUS=0.0;#heading angle of drift
FPU=0.0;#fact track angle
ZPU=0.0;#required track angle
We=0.0;
Wn=0.0;

trk_lck_mode = 0;
abs_z_nm = 0.0;
trk_range1 = 0.0;
trk_range2 = 0.0;
trk_range3 = 0.0;#lur 30-deg turn
trk_range4 = 0.0;#lur 5-deg turn
trk_lock_range = 0.0;#lock range = +-100 meters
intrcpt_hdg_deg = 0.0;
trk_intrcpt_hdg_err = 0.0;
trk_corr_err = 0.0;

trk_on = 0;
fpa_on = 0;
fpa_vs = 0.0;
fpa_deg=0.0;
lnav = 0;
vnav=0;
lnav_last = 0;
vbar_roll = 0.0;
vbar_pitch = 0.0;
vbar_rol_propl = 0.0;
vbar_pitch_prop = 0.0;
vbar_roll_prop = 0.0;
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
at_on=0;
alt_alert = 0.0;
course = 0.0;
course_offset=0.0;
nav_hdg_offset=0.0;
nav_mag_brg=0.0;
gs_active = 0.0;
to_flag = 0;
slaved = 0;
test = 0.0;
maxroll=0;
maxpitch=0;
nav_time=0;
nav_dt=0;
inrange=0;
nav_rate=0;
tth=0;
current_pitch=0;
desired_course=0;
nav_adjust=0;
tgtrad=0;
bank=0;
curhdg=0;
diff=0;
roll_out_time_sec=0;
#waypoint
wp_id="";
wp_longitude_deg=0.0;
wp_latitude_deg=0.0;
wp_altitude_ft=0;

Sorth=0.0;
Aorth=0.0;
wp1_lon=0.0;
wp1_lat=0.0;
wp2_lon=0.0;
wp2_lat=0.0;
NextWP_trueCourse=0.0;
currTrack_trueCourse=0.0;
currWP_trueCourse=0.0;
complete_turn = 0;
in_turn = 0;
GPS_GO=0;

setlistener("/autopilot/route-manager/wp/id", func {
 wpid = getprop("/autopilot/route-manager/wp/id");
 if(wpid == nil or wpid == 0) {return;}
 if(size(wpid)==0) {return;};
 for(i=1;i<6;i=i+1) {
   setprop("/autopilot/route-manager/wp/id-asc"~i~"", 0);
 }
 for(i=0;i<size(wpid);i=i+1) {
   setprop("/autopilot/route-manager/wp/id-asc"~(i+1)~"", wpid[i]);
 }
});

setlistener("/autopilot/route-manager/wp/eta", func {
 eta = getprop("/autopilot/route-manager/wp/eta");
 if(eta == nil or eta == 0) {return;}
 len = size(eta);
 if(len == 0) {return;}
 if(len<5) {
  setprop("/autopilot/route-manager/wp/eta-asc1", 0);
  setprop("/autopilot/route-manager/wp/eta-asc2", eta[0]-48);
 }
 else {
  setprop("/autopilot/route-manager/wp/eta-asc1", eta[0]-48);
  setprop("/autopilot/route-manager/wp/eta-asc2", eta[1]-48);
 };

 setprop("/autopilot/route-manager/wp/eta-asc4", eta[len-1]-48);
 setprop("/autopilot/route-manager/wp/eta-asc3", eta[len-2]-48);
});

setlistener("/autopilot/route-manager/wp[1]/id", func {
 wpid = getprop("/autopilot/route-manager/wp[1]/id");
 if(wpid == nil or wpid == 0) {return;}
 if(size(wpid)==0) {return;};
 for(i=1;i<6;i=i+1) {
   setprop("/autopilot/route-manager/wp[1]/id-asc"~i~"", 0);
 }
 for(i=0;i<size(wpid);i=i+1) {
   setprop("/autopilot/route-manager/wp[1]/id-asc"~(i+1)~"", wpid[i]);
 }
});

setlistener("/autopilot/route-manager/wp[1]/eta", func {
 eta = getprop("/autopilot/route-manager/wp[1]/eta");
 if(eta == nil or eta == 0) {return;}
 len = size(eta);
 if(len == 0) {return;}
 if(len<5) {
  setprop("/autopilot/route-manager/wp[1]/eta-asc1", 0);
  setprop("/autopilot/route-manager/wp[1]/eta-asc2", eta[0]-48);
 }
 else {
  setprop("/autopilot/route-manager/wp[1]/eta-asc1", eta[0]-48);
  setprop("/autopilot/route-manager/wp[1]/eta-asc2", eta[1]-48);
 };

 setprop("/autopilot/route-manager/wp[1]/eta-asc4", eta[len-1]-48);
 setprop("/autopilot/route-manager/wp[1]/eta-asc3", eta[len-2]-48);
});

setlistener("/autopilot/route-manager/wp-last/id", func {
 wpid = getprop("/autopilot/route-manager/wp-last/id");
 if(wpid == nil or wpid == 0) {return;}
 if(size(wpid)==0) {return;};
 for(i=1;i<6;i=i+1) {
   setprop("/autopilot/route-manager/wp-last/id-asc"~i~"", 0);
 }
 for(i=0;i<size(wpid);i=i+1) {
   setprop("/autopilot/route-manager/wp-last/id-asc"~(i+1)~"", wpid[i]);
 }
});

setlistener("/autopilot/route-manager/wp-last/eta", func {
 eta = getprop("/autopilot/route-manager/wp-last/eta");
 if(eta == nil or eta == 0) {return;}
 len = size(eta);
 if(len == 0) {return;}
 if(len<5) {
  setprop("/autopilot/route-manager/wp-last/eta-asc1", 0);
  setprop("/autopilot/route-manager/wp-last/eta-asc2", eta[0]-48);
 }
 else {
  setprop("/autopilot/route-manager/wp-last/eta-asc1", eta[0]-48);
  setprop("/autopilot/route-manager/wp-last/eta-asc2", eta[1]-48);
 };

 setprop("/autopilot/route-manager/wp-last/eta-asc4", eta[len-1]-48);
 setprop("/autopilot/route-manager/wp-last/eta-asc3", eta[len-2]-48);
});

setlistener("/autopilot/route-manager/route/wp[2]/id", func {
 wpid = getprop("/autopilot/route-manager/route/wp[2]/id");
 if(wpid == nil or wpid == 0) {return;}
 if(size(wpid)==0) {return;};
 for(i=1;i<6;i=i+1) {
   setprop("/autopilot/route-manager/route/wp[2]/id-asc"~i~"", 0);
 }
 for(i=0;i<size(wpid);i=i+1) {
   setprop("/autopilot/route-manager/route/wp[2]/id-asc"~(i+1)~"", wpid[i]);
 }
});

setlistener("/autopilot/route-manager/route/wp[3]/id", func {
 wpid = getprop("/autopilot/route-manager/route/wp[3]/id");
 if(wpid == nil or wpid == 0) {return;}
 if(size(wpid)==0) {return;};
 for(i=1;i<6;i=i+1) {
   setprop("/autopilot/route-manager/route/wp[3]/id-asc"~i~"", 0);
 }
 for(i=0;i<size(wpid);i=i+1) {
   setprop("/autopilot/route-manager/route/wp[3]/id-asc"~(i+1)~"", wpid[i]);
 }
});

setlistener("/autopilot/route-manager/route/wp[4]/id", func {
 wpid = getprop("/autopilot/route-manager/route/wp[4]/id");
 if(wpid == nil or wpid == 0) {return;}
 if(size(wpid)==0) {return;};
 for(i=1;i<6;i=i+1) {
   setprop("/autopilot/route-manager/route/wp[4]/id-asc"~i~"", 0);
 }
 for(i=0;i<size(wpid);i=i+1) {
   setprop("/autopilot/route-manager/route/wp[4]/id-asc"~(i+1)~"", wpid[i]);
 }
});

setlistener("/autopilot/route-manager/route/wp[5]/id", func {
 wpid = getprop("/autopilot/route-manager/route/wp[5]/id");
 if(wpid == nil or wpid == 0) {return;}
 if(size(wpid)==0) {return;};
 for(i=1;i<6;i=i+1) {
   setprop("/autopilot/route-manager/route/wp[5]/id-asc"~i~"", 0);
 }
 for(i=0;i<size(wpid);i=i+1) {
   setprop("/autopilot/route-manager/route/wp[5]/id-asc"~(i+1)~"", wpid[i]);
 }
});

setlistener("/autopilot/route-manager/route/wp[6]/id", func {
 wpid = getprop("/autopilot/route-manager/route/wp[6]/id");
 if(wpid == nil or wpid == 0) {return;}
 if(size(wpid)==0) {return;};
 for(i=1;i<6;i=i+1) {
   setprop("/autopilot/route-manager/route/wp[6]/id-asc"~i~"", 0);
 }
 for(i=0;i<size(wpid);i=i+1) {
   setprop("/autopilot/route-manager/route/wp[6]/id-asc"~(i+1)~"", wpid[i]);
 }
});

setlistener("/autopilot/route-manager/route/wp[7]/id", func {
 wpid = getprop("/autopilot/route-manager/route/wp[7]/id");
 if(wpid == nil or wpid == 0) {return;}
 if(size(wpid)==0) {return;};
 for(i=1;i<6;i=i+1) {
   setprop("/autopilot/route-manager/route/wp[7]/id-asc"~i~"", 0);
 }
 for(i=0;i<size(wpid);i=i+1) {
   setprop("/autopilot/route-manager/route/wp[7]/id-asc"~(i+1)~"", wpid[i]);
 }
});

setlistener("/instrumentation/gps/wp/wp/ID", func {
 wpid = getprop("/instrumentation/gps/wp/wp/ID");
 if(wpid == nil or wpid == 0) {return;}
 if(size(wpid)==0) {return;};
 for(i=1;i<6;i=i+1) {
   setprop("/instrumentation/gps/wp/wp/ID-asc"~i~"", 0);
 }
 for(i=0;i<size(wpid);i=i+1) {
   setprop("/instrumentation/gps/wp/wp/ID-asc"~(i+1)~"", wpid[i]);
 }
});

setlistener("/instrumentation/gps/wp/wp[1]/ID", func {
 wpid = getprop("/instrumentation/gps/wp/wp[1]/ID");
 if(wpid == nil or wpid == 0) {return;}
 if(size(wpid)==0) {return;};
 for(i=1;i<6;i=i+1) {
   setprop("/instrumentation/gps/wp/wp[1]/ID-asc"~i~"", 0);
 }
 for(i=0;i<size(wpid);i=i+1) {
   setprop("/instrumentation/gps/wp/wp[1]/ID-asc"~(i+1)~"", wpid[i]);
 }
});


#trueCourse calculating of current waypoint
calc_wp_heading = func {
 wp1_lat = getprop("/instrumentation/gps/indicated-latitude-deg");
 wp1_lon = getprop("/instrumentation/gps/indicated-longitude-deg");
 wp2_lat = getprop("/autopilot/route-manager/route/wp/latitude-deg");
 wp2_lon = getprop("/autopilot/route-manager/route/wp/longitude-deg");

 if(wp1_lat == nil or wp1_lon == nil) {#no route
  print("no current position - nothing to calc!");
  return;
 };
 if(wp2_lat == nil or wp2_lon == nil) {#end of route (last waypoint)
  print("no route (waypoint) - nothing to calc!");
  return;
 };

 wp1_lat *= DEG2RAD;
 wp1_lon *= DEG2RAD;
 wp2_lat *= DEG2RAD;
 wp2_lon *= DEG2RAD;
 
 sin_lat1 = math.sin(wp1_lat);
 cos_lat1 = math.cos(wp1_lat);
 sin_lat2 = math.sin(wp2_lat);
 cos_lat2 = math.cos(wp2_lat);
 dlon = wp2_lon-wp1_lon;
   
 Aorth = math.atan2(math.sin(dlon)*cos_lat2, cos_lat1*sin_lat2-sin_lat1*cos_lat2*math.cos(dlon));
 while ( Aorth >= TWOPI ) {Aorth -= TWOPI};
 if(Aorth<0) Aorth+= TWOPI;
 currWP_trueCourse = Aorth*RAD2DEG;
 setprop("/instrumentation/flightdirector/currWP_trueCourse-deg", currWP_trueCourse);
# print("currWP_trueCourse=", currWP_trueCourse," deg");
};

#calculation of orthodromic true course to next waypoint
calc_orthodromic_params = func {
 wp1_lat = getprop("/autopilot/route-manager/route/wp/latitude-deg");
 wp1_lon = getprop("/autopilot/route-manager/route/wp/longitude-deg");
 wp2_lat = getprop("/autopilot/route-manager/route/wp[1]/latitude-deg");
 wp2_lon = getprop("/autopilot/route-manager/route/wp[1]/longitude-deg");

 if(wp1_lat == nil or wp1_lon == nil) {#no route
  print("no route - nothing to calc!");
  return;
 };
 if(wp2_lat == nil or wp2_lon == nil) {#end of route (last waypoint)
  print("end of route (last waypoint) - nothing to calc!");
  return;
 };

 wp1_lat *= DEG2RAD;
 wp1_lon *= DEG2RAD;
 wp2_lat *= DEG2RAD;
 wp2_lon *= DEG2RAD;
 
 sin_lat1 = math.sin(wp1_lat);
 cos_lat1 = math.cos(wp1_lat);
 sin_lat2 = math.sin(wp2_lat);
 cos_lat2 = math.cos(wp2_lat);
 dlon = wp2_lon-wp1_lon;
   
 Aorth = math.atan2(math.sin(dlon)*cos_lat2, cos_lat1*sin_lat2-sin_lat1*cos_lat2*math.cos(dlon));
 while ( Aorth >= TWOPI ) {Aorth -= TWOPI};
 if(Aorth<0) Aorth+= TWOPI;
 NextWP_trueCourse = Aorth*RAD2DEG;
 setprop("/instrumentation/flightdirector/NextWP_trueCourse-deg", NextWP_trueCourse);
 print("NextWP_trueCourse=", NextWP_trueCourse," deg");
};

#calculation of track orthodromic true course
calc_initial_track_params = func {
 wp1_lat = getprop("/instrumentation/gps/wp/wp/latitude-deg");
 wp1_lon = getprop("/instrumentation/gps/wp/wp/longitude-deg");
 wp2_lat = getprop("/autopilot/route-manager/route/wp/latitude-deg");
 wp2_lon = getprop("/autopilot/route-manager/route/wp/longitude-deg");

 if(wp1_lat == nil or wp1_lon == nil) {#no route
  print("no route - nothing to calc!");
  return;
 };
 if(wp2_lat == nil or wp2_lon == nil) {#end of route (last waypoint)
  print("end of route (last waypoint) - nothing to calc!");
  return;
 };

 wp1_lat *= DEG2RAD;
 wp1_lon *= DEG2RAD;
 wp2_lat *= DEG2RAD;
 wp2_lon *= DEG2RAD;
 
 sin_lat1 = math.sin(wp1_lat);
 cos_lat1 = math.cos(wp1_lat);
 sin_lat2 = math.sin(wp2_lat);
 cos_lat2 = math.cos(wp2_lat);
 dlon = wp2_lon-wp1_lon;
   
 Aorth = math.atan2(math.sin(dlon)*cos_lat2, cos_lat1*sin_lat2-sin_lat1*cos_lat2*math.cos(dlon));
 while ( Aorth >= TWOPI ) {Aorth -= TWOPI};
 if(Aorth<0) Aorth+= TWOPI;
 currTrack_trueCourse = Aorth*RAD2DEG;
 setprop("/instrumentation/flightdirector/currTrack_trueCourse-deg", currTrack_trueCourse);
 print("currTrack_trueCourse=", currTrack_trueCourse," deg");
 if(trk_on == 1) {setprop("autopilot/settings/heading-bug-deg", currTrack_trueCourse - getprop("/environment/magnetic-variation-deg"))};
};

#############################################################################
# Use tha nasal timer to call the initialization function once the sim is
# up and running
#############################################################################

init_gps = func {
#setting gps
    wp_id = getprop("/autopilot/route-manager/route/wp/id");
    if(wp_id == nil or size(wp_id) == 0) {#if no wp or route
     print("init_gps() - no wp, exiting...");
     return;
    };
    print("Initializing GPS: wp_id=", wp_id);

    setprop("/instrumentation/gps/wp/wp[1]/waypoint-type", "nav");
    setprop("/instrumentation/gps/wp/wp[1]/name", "waypoint");

    setprop("/instrumentation/gps/wp/wp[1]/ID", wp_id);

    wp_altitude_ft = getprop("/autopilot/route-manager/route/wp/altitude-ft");
    setprop("/instrumentation/gps/wp/wp[1]/altitude-ft", wp_altitude_ft);
#    print("wp_alt_ft=", wp_altitude_ft);

    wp_latitude_deg = getprop("/autopilot/route-manager/route/wp/latitude-deg");
    setprop("/instrumentation/gps/wp/wp[1]/latitude-deg", wp_latitude_deg);
#    print("wp_lat_deg=", wp_latitude_deg);

    wp_longitude_deg = getprop("/autopilot/route-manager/route/wp/longitude-deg");
    setprop("/instrumentation/gps/wp/wp[1]/longitude-deg", wp_longitude_deg);
#    print("wp_lon_deg=", wp_longitude_deg);
    calc_initial_track_params();
    calc_orthodromic_params();
    GPS_GO=1;
};

gps_next_leg = func {
    wp_id = getprop("/autopilot/route-manager/route/wp/id");
    if(wp_id == nil or size(wp_id)==0) {
     print("It was last waypoint - nothing to do!");
     return;
    }
   if(complete_turn == 1) {
    setprop("/instrumentation/gps/wp/wp/waypoint-type", getprop("/instrumentation/gps/wp/wp[1]/waypoint-type"));
    setprop("/instrumentation/gps/wp/wp/name", getprop("/instrumentation/gps/wp/wp[1]/name"));
    setprop("/instrumentation/gps/wp/wp/ID", getprop("/instrumentation/gps/wp/wp[1]/ID"));
    setprop("/instrumentation/gps/wp/wp/altitude-ft", getprop("/instrumentation/gps/wp/wp[1]/altitude-ft"));
    setprop("/instrumentation/gps/wp/wp/latitude-deg", getprop("/instrumentation/gps/wp/wp[1]/latitude-deg"));
    setprop("/instrumentation/gps/wp/wp/longitude-deg", getprop("/instrumentation/gps/wp/wp[1]/longitude-deg"));
    complete_turn = 0;
    currTrack_trueCourse = NextWP_trueCourse;
    setprop("/instrumentation/flightdirector/currTrack_trueCourse-deg", currTrack_trueCourse);
    print("currTrack_trueCourse=", currTrack_trueCourse," deg");
    if(trk_on == 1) {setprop("autopilot/settings/heading-bug-deg", currTrack_trueCourse - getprop("/environment/magnetic-variation-deg"))};
   }
   else {calc_initial_track_params();};
   
    print("new track: ", getprop("/instrumentation/gps/wp/wp/ID"), " -> ", wp_id);
    setprop("/instrumentation/gps/wp/wp[1]/ID", wp_id);

    wp_altitude_ft = getprop("/autopilot/route-manager/route/wp/altitude-ft");
    setprop("/instrumentation/gps/wp/wp[1]/altitude-ft", wp_altitude_ft);

    wp_latitude_deg = getprop("/autopilot/route-manager/route/wp/latitude-deg");
    setprop("/instrumentation/gps/wp/wp[1]/latitude-deg", wp_latitude_deg);

    wp_longitude_deg = getprop("/autopilot/route-manager/route/wp/longitude-deg");
    setprop("/instrumentation/gps/wp/wp[1]/longitude-deg", wp_longitude_deg);

#change current track true heading    
    calc_orthodromic_params();
}

setlistener("/autopilot/route-manager/input", func {
 ap_rm_cmd = getprop("/autopilot/route-manager/input");
 if(ap_rm_cmd == nil) {return;}
# print("route-manager command=", ap_rm_cmd);
 cmd = substr(ap_rm_cmd,0,4);
 if(streq(cmd, "@pop") or streq(cmd, "@del")) {
# print("recognized command=", ap_rm_cmd);
  gps_next_leg(); 
  if(trk_on==1) {
     in_turn = 1;
     intrcpt_hdg_deg = currTrack_trueCourse;
     setprop("/instrumentation/flightdirector/curr-in-turn", in_turn);
  } 
  return;
 };
 if(streq(cmd, "@ins")) {
#  print("recognized command=", ap_rm_cmd);
  GPS_GO = 0;
  settimer(init_gps,1);
  return;
 };
});

setlistener("/sim/signals/fdm-initialized", func {
    # default values
    print("Initializing Flight Director");
    setprop("/instrumentation/flightdirector/lnav", 0.0);
    setprop("/instrumentation/flightdirector/vnav", 0.0);
    setprop("/instrumentation/flightdirector/vbar-pitch", 0.0);
    setprop("/instrumentation/flightdirector/vbar-roll", 0.0);
    setprop("/instrumentation/flightdirector/alt-offset", 0.0);
    setprop("/instrumentation/flightdirector/autopilot-on",0.0);
    setprop("/instrumentation/flightdirector/fd-on",0.0);
    setprop("/instrumentation/flightdirector/at-on",0);
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
    setprop("autopilot/settings/vertical-speed-fpm",0);
    setprop("autopilot/settings/target-speed-kt",250);
    setprop("/instrumentation/nav/slaved-to-gps",slaved);
    current_alt = getprop("/instrumentation/altimer/indicated-altitude-ft");
    alt_select = getprop("/autopilot/settings/target-altitude-ft");
    setprop("/instrumentation/flightdirector/to-flag",getprop("/instrumentation/nav/to-flag"));
#route settings
#    setprop("/instrumentation/flightdirector/IAS-kt", 0.0);
#    setprop("/instrumentation/flightdirector/TAS-kt", 0.0);
    setprop("/instrumentation/flightdirector/ground-speed-kt", 0.0);
    setprop("/instrumentation/flightdirector/NextWP_trueCourse-deg", 0.0);
    setprop("/instrumentation/flightdirector/currTrack_trueCourse-deg", 0.0);
    setprop("/instrumentation/flightdirector/turn-distance-m", 0.0);
    setprop("/instrumentation/flightdirector/turn-distance-nm", 0.0);
    setprop("/instrumentation/flightdirector/wind-speed-kt", 0.0);
    setprop("/instrumentation/flightdirector/wind-angle-deg", 0.0);
    setprop("/instrumentation/flightdirector/next-heading-diff-deg", 0.0);
    setprop("/instrumentation/flightdirector/curr-z-nm", 0.0);
    setprop("/instrumentation/flightdirector/curr-trk-lck-mode", 0.0);
    setprop("/instrumentation/flightdirector/currTrueHdg-deg", 0.0);
    setprop("/instrumentation/flightdirector/currUS-deg", 0.0);
    setprop("/instrumentation/flightdirector/track-mode-on", 0);
    setprop("/instrumentation/flightdirector/fpa-mode-on", 0);
    setprop("/instrumentation/flightdirector/fpa-deg", 0.0);
    setprop("/instrumentation/flightdirector/fpa-val-deg", 0.0);
    setprop("/instrumentation/flightdirector/currWP_trueCourse-deg",0.0);
    setprop("/instrumentation/flightdirector/curr-intrcpt-hdg-err", 0.0);
    setprop("/instrumentation/flightdirector/curr-trk-corr-err", 0.0);
    setprop("/instrumentation/flightdirector/curr-turn-radius-nm", 0.0);
    setprop("/instrumentation/flightdirector/curr-in-turn", 0);
 
    if(getprop("/autopilot/route-manager/route/wp/id")!=nil) {#if waypoint present
     if(getprop("/autopilot/route-manager/route/wp/altitude-ft") < 40000) {#setting target altitude
      setprop("/autopilot/settings/target-altitude-ft", getprop("/autopilot/route-manager/route/wp/altitude-ft"));
     }
   }
 settimer(init_gps,3);
});

#############################################################################
# handle KC 290 Mode Controller inputs, and compute correct mode/settings
#############################################################################
setlistener("/instrumentation/flightdirector/autopilot-on", func {
    ap_on = getprop("/instrumentation/flightdirector/autopilot-on");
if(ap_on == 1) {
 if(lnav == 0 or lnav ==nil){setprop("autopilot/locks/heading","wing-leveler");}
}
else {
 setprop("autopilot/locks/heading","");
 setprop("autopilot/locks/altitude","");
 setprop("autopilot/locks/speed","");
}
});

setlistener("/instrumentation/flightdirector/at-on", func {
    at_on = getprop("/instrumentation/flightdirector/at-on");
if(ap_on == 1) {
 if(at_on) {setprop("autopilot/locks/speed","speed-with-throttle");}
 else {setprop("autopilot/locks/speed","");}
}
});

setlistener("/instrumentation/flightdirector/fd-on", func {
    fd_on = getprop("/instrumentation/flightdirector/fd-on");
 if(fd_on == 1) {setprop("autopilot/locks/passive-mode",1);}
 else {setprop("autopilot/locks/passive-mode",0);}
});

setlistener("/instrumentation/flightdirector/lnav", func {
    lnav = getprop("/instrumentation/flightdirector/lnav");
if(ap_on==1){
if(lnav == 0 or lnav ==nil){setprop("autopilot/locks/heading","wing-leveler");}
if(lnav == 1){setprop("autopilot/locks/heading","dg-heading-hold");}
if(lnav == 2){setprop("autopilot/locks/heading","nav1-hold");}
#if(lnav == 3){setprop("autopilot/locks/heading","appr-hold");}
if(lnav == 3){setprop("autopilot/locks/heading","nav1-hold");}
if(lnav == 4){setprop("autopilot/locks/heading","bc-hold");}
if(lnav == 5){setprop("autopilot/locks/heading","true-heading-hold");}
}
});

setlistener("/instrumentation/flightdirector/vnav", func {
    vnav = getprop("/instrumentation/flightdirector/vnav");
if(ap_on==1){
if(vnav == 0 or vnav == nil){setprop("autopilot/locks/altitude","pitch-hold");}
if(vnav == 1){if(getprop("/instrumentation/nav/has-gs")!=0){
setprop("autopilot/locks/altitude","gs1-hold");}}
if(vnav == 2){setprop("autopilot/locks/altitude","altitude-hold");}
if(vnav == 3){setprop("autopilot/locks/altitude","vertical-speed-hold");}
if(vnav == 4){setprop("autopilot/locks/speed","speed-with-throttle");}
if(vnav == 5){setprop("autopilot/locks/speed","dcs-hold");}
if(vnav == 6){setprop("autopilot/locks/speed","climb-hold");}
}
});

setlistener("/instrumentation/flightdirector/track-mode-on", func {
    trk_on = getprop("/instrumentation/flightdirector/track-mode-on");
 if(ap_on == 1 and lnav == 5) {
   trk_lck_mode = 0; 
   if(trk_on == 1) {
    setprop("autopilot/locks/heading","track-hold");
    setprop("autopilot/settings/heading-bug-deg", currTrack_trueCourse - getprop("/environment/magnetic-variation-deg"));
   }
   else {setprop("autopilot/locks/heading","true-heading-hold");};
 }
});

setlistener("/instrumentation/flightdirector/fpa-mode-on", func {
    fpa_on = getprop("/instrumentation/flightdirector/fpa-mode-on");
   if(fpa_on == 1) {
    fpa_vs = W_kt*(math.sin(fpa_deg*DEG2RAD)/math.cos(fpa_deg*DEG2RAD));#in knots
    fpa_vs *= KT2MSEC;#in m/sec
    fpa_vs *= MSEC2FPM;# in fpm
    setprop("/autopilot/settings/vertical-speed-fpm", fpa_vs);
   };
});

setlistener("/instrumentation/flightdirector/fpa-deg", func {
    fpa_deg = getprop("/instrumentation/flightdirector/fpa-deg");
    if(fpa_deg < 0) {setprop("/instrumentation/flightdirector/fpa-val-deg", (-1*fpa_deg));}
    else {setprop("/instrumentation/flightdirector/fpa-val-deg", fpa_deg);}
   if(fpa_on == 1) {
    fpa_vs = W_kt*(math.sin(fpa_deg*DEG2RAD)/math.cos(fpa_deg*DEG2RAD));#in knots
    fpa_vs *= KT2MSEC;#in m/sec
    fpa_vs *= MSEC2FPM;# in fpm
    setprop("/autopilot/settings/vertical-speed-fpm", fpa_vs);
   };
});

handle_inputs = func {
# Autopilot  activate

if(ap_on==1){
maxroll = getprop("/orientation/roll-deg");
if(maxroll > 45 or maxroll < -45){
 ap_on = 0;
 setprop("/instrumentation/flightdirector/autopilot-on",ap_on);
}
maxpitch = getprop("/orientation/pitch-deg");
if(maxpitch > 45 or maxpitch < -45){
 ap_on = 0;
 setprop("/instrumentation/flightdirector/autopilot-on",ap_on);
}
#if(getprop("/position/altitude-agl-ft") < 50){ap_on = 0;}
}
#controlling vertical speed for FPA mode
   if(fpa_on == 1) {
    fpa_vs = W_kt*(math.sin(fpa_deg*DEG2RAD)/math.cos(fpa_deg*DEG2RAD));#in knots
    fpa_vs *= KT2MSEC;#in m/sec
    fpa_vs *= MSEC2FPM;# in fpm
    setprop("/autopilot/settings/vertical-speed-fpm", fpa_vs);
   };

}


#############################################################################
# track and update mode
#############################################################################

update_mode = func {
 #   lnav = getprop("/instrumentation/flightdirector/lnav");

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

#setlistener("/sim/signals/fdm-initialized", func {
#    setprop("/instrumentation/flightdirector/lnav", lnav);
#});
}

#############################################################################
#get pitch from autopilot altitude setting
#############################################################################

get_altpitch = func(){
#vnav = getprop("/instrumentation/flightdirector/vnav");
current_pitch = getprop("/orientation/pitch-deg");
if(vnav == 0){return(current_pitch);}
alt_offset = 0.0;

if(vnav == 6){return(5.0);}
if(vnav == 5){return(0.0);}
if(vnav == 4){return(0.0);}
if(vnav == 3){return(getprop("/autopilot/settings/vertical-speed-fpm")-getprop("/instrumentation/vertical-speed-indicator/indicated-speed-fpm"));}
#if(vnav == 1){alt_select = getprop("/autopilot/settings/target-alt-hold");}
if(vnav == 1){#calculation of glideslope altitude for current glideslope distance from NAV1
 alt_select=((getprop("/instrumentation/nav/gs-distance")*FD_TAN3DEG) + getprop("/environment/ground-elevation-m"))*3.281;#meters to feet
# alt_selest = alt_select + getprop("/environment/ground-elevation-m")*3.281;
}
if(vnav == 2){alt_select = getprop("/autopilot/settings/target-altitude-ft");}
#if(lnav == 3){test=getprop("/instrumentation/flightdirector/gs-pitch");
#if(vnav == 1){test=(getprop("/instrumentation/nav/gs-rate-of-climb")-getprop("/instrumentation/vertical-speed-indicator/indicated-speed-fpm"))*0.01;
#if(test == nil) {test = 0.0;}
#return(test);
#}

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
#setlistener("/sim/signals/fdm-initialized", func {
# setprop("/instrumentation/flightdirector/to-flag",getprop("/instrumentation/nav/to-flag"));
#});
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

#calculating true ground speed from navigation speed-triangle
#FIX ME! Here must TAS, not IAS!!!
#calculating TAS with linear interpolation of airspeed indicator error-model
  H760_m = getprop("/position/altitude-ft")*0.305;
  if(H760_m <= 6000) {
    tas_koeff = 0.05*(H760_m/1000);
  }
  if(H760_m > 6000) {
    tas_koeff = 0.3 + 0.1*((H760_m-6000)/1000);
  }
#  W_kt = getprop("/instrumentation/airspeed-indicator/indicated-speed-kt");
#  setprop("/instrumentation/flightdirector/IAS-kt", W_kt);
#  W_kt += (W_kt*tas_koeff);
#  setprop("/instrumentation/flightdirector/TAS-kt", W_kt);
#  if(W_kt == nil) {W_kt=0;}
  wind_angle = (getprop("/environment/wind-from-heading-deg") + 180 - getprop("/orientation/heading-deg"));#heading wind-angle
  if(wind_angle > 180) {wind_angle -= 360;};
  if(wind_angle < -180) {wind_angle += 360;};
#wind_angle = (getprop("/environment/wind-from-heading-deg") + 180 - currTrack_trueCourse);#wind-angle
  setprop("/instrumentation/flightdirector/wind-angle-deg", wind_angle);
  wind_angle *= DEG2RAD;
  wind_speed_kt = getprop("/environment/wind-speed-kt");
  if(wind_speed_kt == nil) {wind_speed_kt = 0;}
  setprop("/instrumentation/flightdirector/wind-speed-kt", wind_speed_kt);
#  W_kt += (wind_speed_kt*math.cos(wind_angle));#true ground speed from navigation speed-triangle

  W_kt = getprop("/velocities/groundspeed-kt");
  setprop("/instrumentation/flightdirector/ground-speed-kt", W_kt);

  We = getprop("/velocities/speed-east-fps");
  Wn = getprop("/velocities/speed-north-fps");
  Ue = getprop("/environment/wind-from-east-fps");
  Un = getprop("/environment/wind-from-north-fps");
  Ve = We + Ue;
  Vn = Wn + Un;
  if(Ve == nil) {Ve=0.0;};
  if(Vn == nil) {Vn=0.0;};
  V = math.sqrt(Ve*Ve+Vn*Vn);#ft/sec
  V *= 60;#ft/min
  V *= FPM2MSEC;#msec
  V *= MSEC2KT;#kt
  setprop("/instrumentation/flightdirector/TAS-kt", V);
#  V = 90 - math.atan2(Vn,Ve)*RAD2DEG;
#  if(V < 0) {V += 360;};
  if(We == nil) {We=0.0;};
  if(Wn == nil) {Wn=0.0;};
  FPU = 90 - math.atan2(Wn,We)*RAD2DEG;
  if(FPU < 0) {FPU += 360;};
  US = FPU - getprop("/orientation/heading-deg");
  if(US > 180) {US -= 360;};
  if(US < -180) {US += 360;};
#  kUS = FPU - V;
#  if(kUS > 180) {kUS -= 360;};
#  if(kUS < -180) {kUS += 360;};
  modUS = math.sqrt(US*US);
  setprop("/instrumentation/flightdirector/currTrueHdg-deg", FPU);
  setprop("/instrumentation/flightdirector/currUS-deg", US);
  setprop("/instrumentation/flightdirector/abs-US-deg", modUS);
#  setprop("/instrumentation/flightdirector/currkUS-deg", kUS);

if(lnav == 5) {#in LNAV mode only
 if(GPS_GO == 1) {
  calc_wp_heading();
  turn_radius_m = W_kt*KT2MSEC;
  turn_radius_m = (turn_radius_m*turn_radius_m)/lur_koeff1;
  setprop("/instrumentation/flightdirector/curr-turn-radius-nm", turn_radius_m/1852);
#  diff = NextWP_trueCourse - getprop("/orientation/heading-deg");
  diff = NextWP_trueCourse - FPU;
  if(diff<0) {diff += 360;}
  if(diff>180) {diff -= 360;}
  if(diff<0) {diff *= -1;}
  if(diff == 180) {diff = 179.9;}
  setprop("/instrumentation/flightdirector/next-heading-diff-deg", diff);
  diff *= DEG2RAD; 
  diff /= 2;
  lur_m = turn_radius_m*(math.sin(diff)/math.cos(diff))+TRK_M;#in meters
#  lur_m = turn_radius_m*(math.sin(diff)/math.cos(diff));#in meters
  lur_nm = lur_m/1852;
  setprop("/instrumentation/flightdirector/turn-distance-m", lur_m);
  setprop("/instrumentation/flightdirector/turn-distance-nm", lur_nm);
  
  turn_dist = getprop("/autopilot/route-manager/wp/dist");
  if(turn_dist == nil) {turn_dist = 0; return;}
#calculate side error from track 
  diff = currWP_trueCourse - currTrack_trueCourse;
  diff *= DEG2RAD;
  z_nm = turn_dist*math.sin(diff);
  setprop("/instrumentation/flightdirector/curr-z-nm", z_nm);
#======================= TRACK HOLD MODE ===============================================================================
   abs_z_nm = math.sqrt(z_nm*z_nm);
   trk_range1 = 2*(turn_radius_m/1852);
   trk_range2 = turn_radius_m/1852;
   trk_lock_range = TRK_M/1852;#lock range = +-200 meters
   trk_range3 = (turn_radius_m*FD_TAN15DEG)/1852;#lur 30-deg turn
   trk_range4 = (turn_radius_m*FD_TAN2_5DEG)/1852;#lur 5-deg turn
   if(in_turn == 1) {#make turn
    diff = currTrack_trueCourse - FPU;
    if(diff >= -2 and diff <= 2 ) {#+-2 degrees - turn finished
     in_turn = 0;
     setprop("/instrumentation/flightdirector/curr-in-turn", in_turn);
    }
   }
   else {#on track
   if(abs_z_nm > trk_range1 and trk_lck_mode==0) {#make 2 90-degree turns
    if(z_nm > 0) {#make right turn
      intrcpt_hdg_deg = currTrack_trueCourse + 90;
    };
    if(z_nm < 0) {#make left turn
      intrcpt_hdg_deg = currTrack_trueCourse - 90;
    };
    if(trk_on == 1) {trk_lck_mode = 1;}
   }
   elsif(abs_z_nm < trk_range1 and abs_z_nm > trk_range2 and trk_lck_mode == 0) {#entering with 30-degree heading diff
    if(z_nm > 0) {#make right turn
      intrcpt_hdg_deg = currTrack_trueCourse + 30;
    };
    if(z_nm < 0) {#make left turn
      intrcpt_hdg_deg = currTrack_trueCourse - 30;
    };
    if(trk_on == 1) {trk_lck_mode = 2;}
   }
   elsif(abs_z_nm > (2*trk_lock_range) and abs_z_nm < trk_range2  and trk_lck_mode == 0) {#entering with 5-degree heading diff
    if(z_nm > 0) {#make right turn
      intrcpt_hdg_deg = currTrack_trueCourse + 5;
    };
    if(z_nm < 0) {#make left turn
      intrcpt_hdg_deg = currTrack_trueCourse - 5;
    };
    if(trk_on == 1) {trk_lck_mode = 3;}
   }
   elsif(abs_z_nm > trk_lock_range and abs_z_nm < (2*trk_lock_range)  and trk_lck_mode == 0) {#entering with 1-degree heading diff
    if(z_nm > 0) {#make right turn
      intrcpt_hdg_deg = currTrack_trueCourse + 2;
    };
    if(z_nm < 0) {#make left turn
      intrcpt_hdg_deg = currTrack_trueCourse - 2;
    };
    if(trk_on == 1) {trk_lck_mode = 4;}
   }
   elsif(abs_z_nm <= (trk_lock_range/2)) {#locking track
    intrcpt_hdg_deg = currTrack_trueCourse;
    if(trk_on == 1) {trk_lck_mode = 5;}
   };
  };
  if(trk_lck_mode == 1) {
    if(abs_z_nm <= trk_range2) intrcpt_hdg_deg = currTrack_trueCourse;
    diff = currTrack_trueCourse - FPU;
    if((diff >= -2 and diff <= 2) and abs_z_nm > (2*trk_lock_range)) {#+-2 degrees - turn finished, but track not locked
     trk_lck_mode = 0;#reset
    }
  }
  elsif(trk_lck_mode == 2) {
    if((abs_z_nm/FD_SIN30DEG) <= trk_range3) {intrcpt_hdg_deg = currTrack_trueCourse;}
    diff = currTrack_trueCourse - FPU;
    if((diff >= -2 and diff <= 2) and abs_z_nm > (2*trk_lock_range)) {#+-2 degrees - turn finished, but track not locked
     trk_lck_mode = 0;#reset
    }
    if(abs_z_nm > trk_range1) {trk_lck_mode=0;}
  }
  elsif(trk_lck_mode == 3) {
    if((abs_z_nm/FD_SIN5DEG) <= trk_range4) {intrcpt_hdg_deg = currTrack_trueCourse;}
    diff = currTrack_trueCourse - FPU;
    if((diff >= -1 and diff <= 1) and abs_z_nm > trk_lock_range) {#+-2 degrees - turn finished, but track not locked
     trk_lck_mode = 0;#reset
    }
   if(abs_z_nm > trk_range2) {trk_lck_mode=0;}
  }
  elsif(trk_lck_mode == 4) {
    if(abs_z_nm <= (trk_lock_range/2)) {intrcpt_hdg_deg = currTrack_trueCourse;}
    if( abs_z_nm > (2*trk_lock_range)) trk_lck_mode = 0;
  }
  elsif(trk_lck_mode == 5 and abs_z_nm > trk_lock_range) {#loss track - restart
    trk_lck_mode = 0;
  }

    if(intrcpt_hdg_deg >= 360) {intrcpt_hdg_deg -= 360};
    if(intrcpt_hdg_deg < 0) {intrcpt_hdg_deg += 360};
    setprop("/instrumentation/flightdirector/curr-trk-lck-mode", trk_lck_mode);
    setprop("/instrumentation/flightdirector/curr-intrcpt-hdg-deg", intrcpt_hdg_deg);
    trk_intrcpt_hdg_err =  intrcpt_hdg_deg - FPU;
#    if(trk_lck_mode > 1) {#in lock mode
#     trk_intrcpt_hdg_err =  intrcpt_hdg_deg - FPU + 20*z_nm;
#    };
    if(trk_lck_mode == 5) {#in lock mode
     trk_intrcpt_hdg_err =  intrcpt_hdg_deg - FPU + 10*z_nm;
     trk_corr_err = 0;#-1*z_nm;
    }
    else {
     trk_corr_err = 0.0;
     if(ap_on==1) setprop("/controls/flight/rudder", 0.0);
    };
    if(trk_intrcpt_hdg_err > 180) {trk_intrcpt_hdg_err -= 360;};
    if(trk_intrcpt_hdg_err < -180) {trk_intrcpt_hdg_err += 360;};
    setprop("/instrumentation/flightdirector/curr-intrcpt-hdg-err", trk_intrcpt_hdg_err);
    setprop("/instrumentation/flightdirector/curr-trk-corr-err", trk_corr_err);
#=============================================================================================================================
  if(trk_on==1) {#in this mode standart FG autopilot not working - we pop waypoint always
   if(lur_m < 50000) {
    if( (turn_dist != 0) and (turn_dist <= lur_nm) ) {#turn distance - begin turn
#     print("Begin turn -> lur_m=", lur_m, " lur_nm =", lur_nm, " dist=", getprop("/autopilot/route-manager/wp/dist"));
     complete_turn = 1;
     setprop("/autopilot/route-manager/input", "@pop");
     trk_lck_mode=0;#next leg - reset track mode
    }
   }
  }
  else {
   if(lur_m > 200 and lur_m < 50000) {#blocking standart FG autopilot - it pop waypoint if distance to waypoint <= 200 meters
    if( (turn_dist != 0) and (turn_dist <= lur_nm) ) {#turn distance - begin turn
#     print("Begin turn -> lur_m=", lur_m, " lur_nm =", lur_nm, " dist=", getprop("/autopilot/route-manager/wp/dist"));
     complete_turn = 1;
     setprop("/autopilot/route-manager/input", "@pop");
    }
   }
  }
 }
}

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
        #Heading bug 
        # FIXME: at what angle off of the hdg bug do we start the rollout?
        # bank to track heading bug

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
#NAV and APR
#tgtrad = getprop("/instrumentation/flightdirector/nav-hdg") * 3;
tgtrad = (getprop("/instrumentation/nav/radials/target-auto-hdg-deg") - getprop("/orientation/heading-deg"))*3;

if(tgtrad > 30){tgtrad = 30;} 
if(tgtrad < -30){tgtrad = -30;} 

vbar_roll = tgtrad;
    } elsif (lnav == 5) {
if(trk_on == 1) {
  tgtrad = getprop("/instrumentation/flightdirector/curr-intrcpt-hdg-err");
  if(tgtrad == nil) {tgtrad=0;};
  tgtrad *= 3;
}
else {
  tgtrad = getprop("/autopilot/internal/true-heading-error-deg") * 3;
}        
if(tgtrad > 30){tgtrad = 30;} 
if(tgtrad < -30){tgtrad = -30;} 

vbar_roll = tgtrad;
   }
   else {
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


#setlistener("/sim/signals/fdm-initialized", func {
 setprop("/instrumentation/flightdirector/vbar-pitch",vbar_pitch_prop);
 setprop("/instrumentation/flightdirector/vbar-roll", vbar_roll_prop);
# setprop("/instrumentation/flightdirector/alt-offset", alt_offset);
# setprop("/instrumentation/flightdirector/current-alt", current_alt);
#});
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
