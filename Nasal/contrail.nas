var CONTRAIL_MAX_DISSIPATION_TIME = 300;
var CONTRAIL_SPREAD_FACTOR = 15;
var CONTRAIL_START_SIZE_FACTOR = 1;
var CONTRAIL_MAXIMUM_SIZE_FACTOR = 120;
var CONTRAIL_PARTICLES_PER_SECOND = 30;

var contrail_update_loop = func {
  var current_RH = getprop("/environment/relative-humidity");
  var temperature = getprop("/environment/temperature-degc");
  var real_RH = current_RH;

  if (0 > temperature) {
    var factor = (100 + 0.815 * temperature) / 100.0;
    if (factor == 0) {
      real_RH = 200;
    } else {
      real_RH = current_RH / factor;
    }
  }

  if (real_RH > 100) real_RH = 100;

  var dissipation_factor = (100 - real_RH) / 100.0;
  var dissipation_time = 0;

  if(dissipation_factor == 0) {
    dissipation_time = CONTRAIL_MAX_DISSIPATION_TIME;
  } else {
    dissipation_time = 10 / dissipation_factor;
  }

  var temp_factor = -temperature - 36.0;

  var density = 0.56 * temp_factor * temp_factor;

  if(temp_factor < 0.0) density = -density;

  density = density + real_RH * 0.312 - 13.8;
  density = density / 100.0;

  if(0.0 > density) density = 0.0;
  if(density > 1.0) density = 1.0;

  var alpha = density * 5.0;
  if(alpha > 1.0) alpha = 1.0;

  setprop("/sim/model/contrail", "dissipation-time", dissipation_time);
  setprop("/sim/model/contrail", "density", alpha);
  setprop("/sim/model/contrail", "start-size", alpha * CONTRAIL_START_SIZE_FACTOR);
  setprop("/sim/model/contrail", "maximum-size", density * CONTRAIL_MAXIMUM_SIZE_FACTOR);
  setprop("/sim/model/contrail", "spread", density * CONTRAIL_SPREAD_FACTOR);

  settimer(contrail_update_loop, 1);
}

setprop("/sim/model/contrail", "particles-per-second", CONTRAIL_PARTICLES_PER_SECOND);
setprop("/sim/model/contrail", "dissipation-time", 0);
setprop("/sim/model/contrail", "density", 0);
setprop("/sim/model/contrail", "start-size", 0);
setprop("/sim/model/contrail", "maximum-size", 0);
setprop("/sim/model/contrail", "spread", 0);
setlistener("/sim/signals/fdm-initialized", contrail_update_loop);
