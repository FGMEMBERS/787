togglereverser = func {
  throttle1 = "/controls/engines/engine/throttle";
  control1 = "/controls/engines/engine[0]/reverser"; 
  control2 = "/controls/engines/engine[1]/reverser"; 

  # The reverse can only be actuated while the engine is idling
  if (getprop(throttle1) < 0.03) {
    val = getprop("/surface-positions/reverser-pos-norm");
    if (val == 0 or val == nil) {
      setprop(control1,1);
      setprop(control2,1);
    } else {
      if (val == 1.0){
        setprop(control1,0);
        setprop(control2,0);
      }
    }
  }
}
