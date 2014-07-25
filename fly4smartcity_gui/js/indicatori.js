/* reference
http://sebmatton.github.io/flightindicators/
*/

// Dynamic examples
var attitude = $.flightIndicator('#attitude', 'attitude',      {size:225,
																																roll:0,
																																pitch:0,
																																showBox : true,
																																img_directory : 'jQuery-Flight-Indicators-master/img/'
																															       });
var heading = $.flightIndicator('#heading', 'heading', 	       {size:225,
																																heading:0,
																																showBox:true,
																																img_directory : 'jQuery-Flight-Indicators-master/img/'
																															       });
var variometer = $.flightIndicator('#variometer', 'variometer',{size:225, 
																																vario:0,
																																showBox:true,
																																img_directory : 'jQuery-Flight-Indicators-master/img/'
																															       });//da 5 a -5
var airspeed = $.flightIndicator('#airspeed', 'airspeed',      {size:225,
																																showBox: true,
																																img_directory : 'jQuery-Flight-Indicators-master/img/'
																															       });
var altimeter = $.flightIndicator('#altimeter', 'altimeter',   {size:225,
																																img_directory : 'jQuery-Flight-Indicators-master/img/'
																															       });



/* opzioni definibili
	var settings = $.extend({
				size : 200,
				roll : 0,
				pitch : 0,
				heading: 0,
				vario: 0,
				airspeed: 0,
				altitude: 0,
				pressure: 1000,
				showBox : true,
				img_directory : 'img/'
			}, options );
*/
/*
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
*/
