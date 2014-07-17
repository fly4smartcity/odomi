//no message from subscriber is received so every buttons is disabled
	var n=0;
	var height='';
	var width='100%';
	var cam1="<img src='http://localhost:8080/stream?topic=/camera/image_raw' height='"+height+"' width='"+width+"'></img>";
	var cam1="<img src='http://localhost:8080/stream?topic=/camera/image_raw2' height='"+height+"' width='"+width+"'  style = 'position:relative;'></img>";
	
	var print1="<embed id='cam_embed'  type='application/x-vlc-plugin'  width=640 height=480  name='player'  target='udp://10.82.98.54:2122'" + "/" + ">";

	var print2="<embed id='cam_embed'  type='application/x-vlc-plugin'  width=640 height=480  name='player'  target='udp://10.82.98.54:2121'" + "/" + ">";


	//document.getElementById('cam').innerHTML = "<body> suca </body>";

	function change(){
			if(n%2==0){
			document.getElementById('cam').innerHTML = print1;
			alert("Cam 1");
				}
				else
				{
			document.getElementById('cam').innerHTML = print2;
			alert("Cam 2");
				}
						n++;
				}

function f_btnInviaPercorso()
{
	publ_waypoint(path);
	//alert("Invio al MP" + p);
	$('#btnInviaPercorso').prop('disabled', true);

}
function f_btnGoMission()
{	
	$('#btnGoMission').prop('disabled', true);
	
	//service with ID
	takeoff_activation();

}

function f_btnCreaPercorsot()
{
	erasePath();
	creapercorso=true;
	alert("Scegli più target sulla mappa");
}

function f_teleopHome(){
	
	publ_home();
	//alert(" home");
}


function f_btnCancellaPercorso(){
	
	erasePath();
	$('#btnInviaPercorso').prop('disabled', true);
	$('#btnCancellaPercorso').prop('disabled', true);

}
function f_teleopCircle(){
	
	alert("cirlce");
	publ_circle();
	
};
