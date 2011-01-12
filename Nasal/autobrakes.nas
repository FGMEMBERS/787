# 787/CRJ-200 Autobrakes
# By Nick I
#

AB=0;

resetabrakes = func {
	setprop("/controls/gear/autobrakes",0);
	}

autobrake = func {

	NGEAR = getprop("/gear/gear/wow");
	ABRAKES = getprop("/controls/gear/autobrakes");
	ABRAKE = getprop("/controls/gear/autobrake");
	THROTTLE = getprop("/controls/engines/engine/throttle");
	GSPEED = getprop("/velocities/groundspeed-kt");

	if (ABRAKES > 0) {
	 AB=1;
	}

	if (ABRAKES == 0) {
	 AB=0;
	}

	if (THROTTLE < 0.03 and NGEAR == 1) {

	 if (ABRAKES < 1.2 and AB == 1 and GSPEED > 20) {
	  if (getprop("/controls/gear/autobrake") == 0) {
	   interpolate("/controls/gear/autobrake",ABRAKES,ABRAKES);
	  }
	  AB=1;
	 }

	 if ((ABRAKE == 1 or ABRAKE == 0.8 or (ABRAKE < 0.61 and ABRAKE > 0.59) or ABRAKE == 0.4 or ABRAKE == 0.2) and GSPEED < 20) {
	   interpolate("/controls/gear/autobrake",0,ABRAKE);
	  AB=0;
	 }

	 if (ABRAKES > 1 and GSPEED > 80) {
	  if (getprop("/controls/gear/autobrake") == 0) {
	   setprop("/controls/gear/autobrake",1);
	  }
	  if (GSPEED > 100) {
	   setprop("/controls/flight/spoiler", 1);
	  }
	 }

	 if (ABRAKES > 1 and GSPEED < 10 and ABRAKE == 1) {
	   setprop("/controls/gear/autobrake",0);
	 }

	 if (ABRAKES > 1 and NGEAR == 0 and getprop("/gear/gear[1]/wow") == 0 and getprop("/gear/gear[2]/wow") == 0) {
	  setprop("/controls/gear/autobrakes", 0);
	 }

	}

	if (NGEAR == 1) {
	 if ((ABRAKE == 1 or ABRAKE == 0.8 or (ABRAKE < 0.61 and ABRAKE > 0.59) or ABRAKE == 0.4 or ABRAKE == 0.2) and GSPEED < 20) {
	   setprop("/controls/gear/autobrake",0);
	  AB=0;
	 }
	}

}

setlistener("/gear/gear[0]/wow", func {
settimer(autobrake,0);
});