#Syd Adams
# Jet Engine electrical system.


battery = nil;
alternator = nil;

last_time = 0.0;
pwr_src = 0.0;

bat_bus_volts = 0.0;
emerg_bus_volts = 0.0;
Lmain_bus_volts = 0.0;
Rmain_bus_volts = 0.0;
AC_bus_amps = 0.0;
ammeter_ave = 0.0;

external_volts = 0.0;
load = 0.0;

init_electrical = func {
    print("Initializing Nasal Electrical System");
    battery = BatteryClass.new();
    alternator = AlternatorClass.new();
    setprop("/controls/switches/master-avionics", 1);
    setprop("/controls/switches/inverter", 1);
    
    if(getprop("/sim/model/start-idling")) {
     setprop("/controls/electric/battery-switch", 0);
     setprop("/controls/electric/pri-external-power", 0);
     setprop("/controls/electric/sec-external-power", 0);
     setprop("/controls/electric/engine[0]/bus-tie", 1);
     setprop("/controls/electric/engine[1]/bus-tie", 1);
     setprop("/controls/electric/engine[0]/generator", 1);
     setprop("/controls/electric/engine[1]/generator", 1);
     setprop("/controls/electric/engine[0]/drive", 1);
     setprop("/controls/electric/engine[1]/drive", 1);
     setprop("/controls/electric/engine[0]/bkp-generator", 1);
     setprop("/controls/electric/engine[1]/bkp-generator", 1);
     setprop("/controls/electric/engine[2]/bus-tie", 1);
     setprop("/controls/electric/engine[2]/generator", 0);
     setprop("/systems/electrical/outputs/eec-Lbus", 30);
     setprop("/systems/electrical/outputs/eec-Rbus", 30);
    }
    else {
     setprop("/controls/electric/battery-switch", 1.0);
     setprop("/controls/electric/pri-external-power", 0);
     setprop("/controls/electric/sec-external-power", 0);
     setprop("/controls/electric/engine[0]/bus-tie", 1);
     setprop("/controls/electric/engine[1]/bus-tie", 1);
     setprop("/controls/electric/engine[0]/generator", 0);
     setprop("/controls/electric/engine[1]/generator", 0);
     setprop("/controls/electric/engine[0]/drive", 1);
     setprop("/controls/electric/engine[1]/drive", 1);
     setprop("/controls/electric/engine[0]/bkp-generator", 0);
     setprop("/controls/electric/engine[1]/bkp-generator", 0);
     setprop("/controls/electric/engine[2]/bus-tie", 1);
     setprop("/controls/electric/engine[2]/generator", 0);
     setprop("/systems/electrical/outputs/eec-Lbus", 0);
     setprop("/systems/electrical/outputs/eec-Rbus", 0);
    };
    settimer(update_electrical, 0);
}

BatteryClass = {};

BatteryClass.new = func {
    obj = { parents : [BatteryClass],
            ideal_volts : 24.0,
            ideal_amps : 30.0,
            amp_hours : 12.75,
            charge_percent : 1.0,
            charge_amps : 7.0 };
    return obj;
}


BatteryClass.apply_load = func( amps, dt ) {
    amphrs_used = amps * dt / 3600.0;
    percent_used = amphrs_used / me.amp_hours;
    me.charge_percent -= percent_used;
    if ( me.charge_percent < 0.0 ) {
        me.charge_percent = 0.0;
    } elsif ( me.charge_percent > 1.0 ) {
        me.charge_percent = 1.0;
    }
    return me.amp_hours * me.charge_percent;
}


BatteryClass.get_output_volts = func {
    x = 1.0 - me.charge_percent;
    tmp = -(3.0 * x - 1.0);
    factor = (tmp*tmp*tmp*tmp*tmp + 32) / 32;
    return me.ideal_volts * factor;
}

BatteryClass.get_output_amps = func {
    x = 1.0 - me.charge_percent;
    tmp = -(3.0 * x - 1.0);
    factor = (tmp*tmp*tmp*tmp*tmp + 32) / 32;
    return me.ideal_amps * factor;
}

AlternatorClass = {};

AlternatorClass.new = func {
    obj = { parents : [AlternatorClass],
            rpm_source : "/sim/model/Boeing-787-8/n2",
            rpm_threshold : 40.0,
            ideal_volts : 28.0,
            ideal_amps : 60.0 };
    setprop( obj.rpm_source, 0.0 );
    return obj;
}

##
# Computes available amps and returns remaining amps after load is applied
#

AlternatorClass.apply_load = func( amps, dt, src ) {
    rpm = getprop(src);
    factor = rpm / me.rpm_threshold;
    if ( factor > 1.0 ) {
        factor = 1.0;
    }
    available_amps = me.ideal_amps * factor;
    return available_amps - amps;
}


AlternatorClass.get_output_volts = func( src ) {
    rpm = getprop(src );
    if (rpm == 0) {
        factor = 0;
    } else {
        factor = math.ln(rpm)/4;
    }
    return me.ideal_volts * factor;
}


AlternatorClass.get_output_amps = func(src ){
    rpm = getprop( src );
    if (rpm == 0) {
        factor = 0;
    } else {
        factor = math.ln(rpm)/4;
    }
    return me.ideal_amps * factor;
}


update_electrical = func {
    time = getprop("/sim/time/elapsed-sec");
    dt = time - last_time;
    last_time = time;
    update_virtual_bus( dt );
    settimer(update_electrical, 0);
}

update_virtual_bus = func( dt ) {
#Battery
    battery_volts = battery.get_output_volts();
#Main generators
    if(getprop("/controls/electric/engine/drive") and getprop("/controls/electric/engine/generator")) {
      alternator_volts = alternator.get_output_volts("/sim/model/Boeing-787-8/n2");
    }
    else {alternator_volts = 0;};
    if(getprop("/controls/electric/engine[1]/drive") and getprop("/controls/electric/engine[1]/generator")) {
      alternator1_volts = alternator.get_output_volts("/sim/model/Boeing-787-8/n2[1]");
    }
    else {alternator1_volts = 0;};
#Backup generators
    if(getprop("/controls/electric/engine/bkp-generator")) {
      bkpalt_volts = alternator.get_output_volts("/sim/model/Boeing-787-8/n2");
    }
    else {bkpalt_volts = 0;};
    if(getprop("/controls/electric/engine[1]/bkp-generator")) {
      bkpalt1_volts = alternator.get_output_volts("/sim/model/Boeing-787-8/n2[1]");
    }
    else {bkpalt1_volts = 0;};
#APU generator
    if(getprop("/controls/electric/engine[2]/generator")) {
      apu_volts = alternator.get_output_volts("/sim/model/Boeing-787-8/apu-n2");
    }
    else {apu_volts = 0;}; 
#external sources
    priext = getprop("/controls/electric/pri-external-power");
    secext = getprop("/controls/electric/sec-external-power");
    if(priext or secext) {
      external_volts = 27.0;
    }
    else {external_volts = 0;}; 

    load = 0.0;

    master_bat = getprop("/controls/electric/battery-switch");
    master_alt = getprop("/controls/electric/engine[0]/generator");
    master_alt1 = getprop("/controls/electric/engine[1]/generator");
    master_bkpalt = getprop("/controls/electric/engine[0]/bkp-generator");
    master_bkpalt1 = getprop("/controls/electric/engine[1]/bkp-generator");
    master_apu = getprop("/controls/electric/engine[2]/generator");
    lbus_tie = getprop("/controls/electric/engine[0]/bus-tie");
    rbus_tie = getprop("/controls/electric/engine[1]/bus-tie");

    # determine power source
    bat_bus_volts = 0.0;
    Lmain_bus_volts = 0.0;
    Rmain_bus_volts = 0.0;
    power_source = nil;

    if ( master_bat == 1.0 ) {
        bat_bus_volts = battery_volts;
        if(lbus_tie) Lmain_bus_volts = bat_bus_volts;
        if(rbus_tie) Rmain_bus_volts = bat_bus_volts;
        emerg_bus_volts = bat_bus_volts;
        power_source = "battery";
    };
#    else{
#     if ( master_bat == 2.0 ) {
#        emerg_bus_volts = battery_volts;
#        power_source = "battery";
#     }
#   }

    if (master_alt) {Lmain_bus_volts = alternator_volts;};
    if (master_alt1) {Rmain_bus_volts = alternator1_volts;};
    if (master_bkpalt and (bkpalt_volts > alternator_volts)) {Lmain_bus_volts = bkpalt_volts;};
    if (master_bkpalt1 and (bkpalt1_volts > alternator1_volts)) {Rmain_bus_volts = bkpalt1_volts;};

    if ( master_alt and (alternator_volts > bat_bus_volts) ) {
        bat_bus_volts = alternator_volts;
        emerg_bus_volts = alternator_volts;
        power_source = "alternator";
    }
    if ( master_alt1 and (alternator1_volts > bat_bus_volts) ) {
        bat_bus_volts = alternator1_volts;
        emerg_bus_volts = alternator1_volts;
        power_source = "alternator";
    }
    if (master_bkpalt and (bkpalt_volts > bat_bus_volts) ) {
        bat_bus_volts = bkpalt_volts;
        emerg_bus_volts = bkpalt_volts;
        power_source = "alternator";
    }
    if (master_bkpalt1 and (bkpalt1_volts > bat_bus_volts) ) {
        bat_bus_volts = bkpalt1_volts;
        emerg_bus_volts = bkpalt1_volts;
        power_source = "alternator";
    }
    if ( master_apu and (apu_volts > bat_bus_volts) ) {
        if(rbus_tie) {Rmain_bus_volts = apu_volts;};
        if(lbus_tie) {Lmain_bus_volts = apu_volts;};
        bat_bus_volts = apu_volts;
        emerg_bus_volts = apu_volts;
        power_source = "alternator";
    }
    if ( external_volts > bat_bus_volts ) {
        bat_bus_volts = external_volts;
        if(priext or (secext and rbus_tie)) {Rmain_bus_volts = external_volts;};
        if((secext and lbus_tie) or (priext and lbus_tie and rbus_tie)) {Lmain_bus_volts = external_volts;}
        emerg_bus_volts = external_volts;
        power_source = "external";
    }

#    starter_switch = getprop("/controls/APU/off-start-run");
#    starter_volts = 0.0;
#    if ( starter_switch == 1) {#start APU
#        starter_volts = bat_bus_volts;
#    }
#    setprop("/systems/electrical/outputs/starter", starter_volts);
     starter_volts = bat_bus_volts;

    load += emergency_bus();
    if(lbus_tie and rbus_tie) {load += cross_feed_bus();};
    load += Left_Main_bus();
    load += Right_Main_bus();
    load += AC_bus();

    setprop("/systems/electrical/outputs/starter", starter_volts);
    setprop("/systems/electrical/outputs/eec-Lbus", Lmain_bus_volts);
    setprop("/systems/electrical/outputs/eec-Rbus", Rmain_bus_volts);
    setprop("/systems/electrical/outputs/apu-bus", emerg_bus_volts);

    ammeter = 0.0;
    if ( bat_bus_volts > 1.0 ) {
        # normal load
        load += 15.0;

        # ammeter gauge
        if ( power_source == "battery" ) {
            ammeter = -load;
        } else {
            ammeter = battery.charge_amps;
        }
    }

    # charge/discharge the battery
    if ( power_source == "battery" ) {
        battery.apply_load( load, dt );
    } elsif ( bat_bus_volts > battery_volts ) {
        battery.apply_load( -battery.charge_amps, dt );
    }

    # filter ammeter needle pos
    ammeter_ave = 0.8 * ammeter_ave + 0.2 * ammeter;

    # outputs
    setprop("/systems/electrical/amps", ammeter_ave);
    setprop("/systems/electrical/volts", bat_bus_volts);
    setprop("/systems/electrical/ac_amps", AC_bus_amps);
    return load;
}

emergency_bus = func() {
    load = 0.0;
    setprop("/systems/electrical/outputs/nav[1]", emerg_bus_volts);
    setprop("/systems/electrical/outputs/com[0]", emerg_bus_volts);

    if ( getprop("/controls/switches/cabin-lights") ) {
        setprop("/systems/electrical/outputs/cabin-lights", emerg_bus_volts);
} else {
        setprop("/systems/electrical/outputs/cabin-lights", 0.0);
    }
    if ( getprop("/controls/switches/pitot-heat" ) ) {
        setprop("/systems/electrical/outputs/pitot-heat", emerg_bus_volts);
    } else {
        setprop("/systems/electrical/outputs/pitot-heat", 0.0);
    }
    return load;
}

cross_feed_bus = func() {
    if (Lmain_bus_volts >= Rmain_bus_volts ) {
        Rmain_bus_volts = Lmain_bus_volts;
    } else {
     Lmain_bus_volts = Rmain_bus_volts;
    }
    load = 0.0;
    return load;
}

#Hydraulic pumps driven by N2 ,but relays controlled by 28 volt DC bus
#- so no DC power - no hydraulics

Left_Main_bus = func() {
load = 0.0;
setprop("/controls/hydraulic/system/engine-pump","false");
if(Lmain_bus_volts > 0.2){
setprop("/controls/hydraulic/system/engine-pump","true");
}

setprop("/systems/electrical/outputs/instr-ignition-switch", Lmain_bus_volts);

    if ( getprop("/controls/engines/engine[0]/fuel-pump") ) {
        setprop("/systems/electrical/outputs/fuel-pump", Lmain_bus_volts);
    } else {
        setprop("/systems/electrical/outputs/fuel-pump", 0.0);
    }

    if ( getprop("/controls/switches/landing-light-c") ) {
        setprop("/systems/electrical/outputs/landing-light-c",Lmain_bus_volts);
    } else {
        setprop("/systems/electrical/outputs/landing-light-c", 0.0 );
    }
    if ( getprop("/controls/switches/landing-light-l") ) {
        setprop("/systems/electrical/outputs/landing-light-l",Lmain_bus_volts);
    } else {
        setprop("/systems/electrical/outputs/landing-light-l", 0.0 );
    }
    if ( getprop("/controls/switches/landing-light-r") ) {
        setprop("/systems/electrical/outputs/landing-light-r",Lmain_bus_volts);
    } else {
        setprop("/systems/electrical/outputs/landing-light-r", 0.0 );
    }

    if ( getprop("/controls/switches/beacon" ) ) {
        setprop("/systems/electrical/outputs/beacon", Lmain_bus_volts);
        if ( bat_bus_volts > 1.0 ) { load += 7.5; }
    } else {
        setprop("/systems/electrical/outputs/beacon", 0.0);
    }
    setprop("/systems/electrical/outputs/flaps",Lmain_bus_volts);
    setprop("/systems/electrical/outputs/turn-coordinator", Lmain_bus_volts);

    if ( getprop("/controls/switches/nav-lights" ) ) {
        setprop("/systems/electrical/outputs/nav-lights", Lmain_bus_volts);
        if ( bat_bus_volts > 1.0 ) { load += 7.0; }
    } else {
        setprop("/systems/electrical/outputs/nav-lights", 0.0);
    }
    setprop("/systems/electrical/outputs/instrument-lights", Lmain_bus_volts);
    if ( getprop("/controls/switches/strobe" ) ) {
        setprop("/systems/electrical/outputs/strobe-lights", Lmain_bus_volts);
    } else {
        setprop("/systems/electrical/outputs/strobe-lights", 0.0);
    }
    if ( getprop("/controls/switches/taxi-lights" ) ) {
        setprop("/systems/electrical/outputs/taxi-lights", Lmain_bus_volts);
    } else {
        setprop("/systems/electrical/outputs/taxi-lights", 0.0);
    }
    if ( getprop("/controls/switches/logo-lights" ) ) {
        setprop("/systems/electrical/outputs/logo-lights", Lmain_bus_volts);
    } else {
        setprop("/systems/electrical/outputs/logo-lights", 0.0);
    }	
	    if ( getprop("/controls/switches/map-lights" ) ) {
        setprop("/systems/electrical/outputs/map-lights", Lmain_bus_volts);
    } else {
        setprop("/systems/electrical/outputs/map-lights", 0.0);
    }	
    return load;
}

Right_Main_bus = func() {
setprop("/controls/hydraulic/system[1]/engine-pump","false");
if(Rmain_bus_volts > 0.2){setprop("/controls/hydraulic/system[1]/engine-pump","true")};
    master_av = getprop("/controls/switches/master-avionics");
    load = 0.0;
if(master_av){
    setprop("/systems/electrical/outputs/avionics-fan",Rmain_bus_volts);
    setprop("/systems/electrical/outputs/gps", Rmain_bus_volts);
    setprop("/systems/electrical/outputs/hsi", Rmain_bus_volts);
    setprop("/systems/electrical/outputs/nav[0]", Rmain_bus_volts);
    setprop("/systems/electrical/outputs/dme", Rmain_bus_volts);
    setprop("/systems/electrical/outputs/audio-panel[0]",Rmain_bus_volts);
    setprop("/systems/electrical/outputs/annunciators", Rmain_bus_volts);
    setprop("/systems/electrical/outputs/audio-panel[1]", Rmain_bus_volts);
    setprop("/systems/electrical/outputs/transponder", Rmain_bus_volts);
    setprop("/systems/electrical/outputs/autopilot", Rmain_bus_volts);
    setprop("/systems/electrical/outputs/adf", Rmain_bus_volts);
    setprop("/systems/electrical/outputs/mk-viii",Rmain_bus_volts);
}
    return load;
}

AC_bus = func() {
AC_bus_amps = 0.0;
if(getprop("/controls/switches/inverter") > 0.0){
if(Rmain_bus_volts > 0.2){AC_bus_amps = 225};
if(Lmain_bus_volts > 0.2){AC_bus_amps = 225};
}else{
setprop("/instrumentation/annunciator/master-caution",1.0);
}
    load = 0.0;
    return load;
}


settimer(init_electrical, 0);
