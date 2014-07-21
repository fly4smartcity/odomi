//http://sebmatton.github.io/flightindicators/
// Dynamic examples
var attitude = $.flightIndicator('#attitude', 'attitude', {roll:50, pitch:-20, size:200, showBox : true});
var heading = $.flightIndicator('#heading', 'heading', {heading:150, showBox:true});
var variometer = $.flightIndicator('#variometer', 'variometer', {vario:-5, showBox:true});
var airspeed = $.flightIndicator('#airspeed', 'airspeed', {showBox: false});
var altimeter = $.flightIndicator('#altimeter', 'altimeter');

// Update at 20Hz
var increment = 0;
setInterval(function() {
    // Attitude update
    attitude.setRoll(30*Math.sin(increment/10));
    attitude.setPitch(50*Math.sin(increment/20));
    
    // Heading update
    heading.setHeading(increment);
    
    // Vario update
    variometer.setVario(2*Math.sin(increment/10));
    
    // Airspeed update
    airspeed.setAirSpeed(80+80*Math.sin(increment/10));
    
    // Altimeter update
    altimeter.setAltitude(10*increment);
    altimeter.setPressure(1000+3*Math.sin(increment/50));
    increment++;
}, 50);

