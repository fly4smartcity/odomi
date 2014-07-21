/* reference
la documentazione e' qui' http://www.rgraph.net/docs/vprogress.html  
*/
var batteryLevel;
$(document).ready(function ()
        {
            batteryLevel = new RGraph.VProgress('cvs-battery',0,100,0) 
		//barrette a lato              
		.set('tickmarks', 100)//se le batterre sono disegnate : 100!=0 quindi true
                .set('numticks', 30)//numero di barrette a lato
		//padding
                .set('gutter.right', 40)//larghezza barra centrale
                .set('margin', 4)//margine barra/contenitore px
		//.set('tickmarks.color','red')
		.set('numticks.inner',0)// ~~~~~~~
		.set('arrows','true')//mette indicatori di dove si trova
		.set('colors', ['Gradient(red:orange:lime:lime:lime)'])
		//titolo a lato
		.set('title.side', 'Battery Level  [%]')	
		.set('title.side.size',11)

//.set('tooltips',['a','b','c','d'])
		//.set('tooltips.override','55')~~~~~~
		//ombreggiatura barra
		.set('shadow', 'true')
		.set('shadow.offsetx',-1)
		.set('shadow.offsety',1)
		.set('shadow.blur',4)
		//labels
		.set('text.size',9)
		.set('units.post','%')
		//.set('labels.position', 'right')
                .draw();
/*
            batteryLevel.canvas.onclick = function (e)
            {
                var value = vprogress.getValue(e);
                batteryLevel.value = value;
		//vprogress=set('title',value+'%');
		//alert(vprogress.title);
                batteryLevel.grow();
            }*/
        })
