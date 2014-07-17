var n=0;
var height='';
var width='100%';
var cam1="<img src='http://localhost:8080/stream?topic=/camera/image_raw' height='"+height+"' width='"+width+"'></img>";
var cam2="<img src='http://localhost:8080/stream?topic=/camera/image_raw2' height='"+height+"' width='"+width+"'></img>";
/*var print1="<spam id='cam1'></spam><spam id='cam2'></spam>";
var print2="<spam id='cam2'></spam><spam id='cam1'></spam>";
document.getElementById('cam').innerHTML = print1;*/

/*function change(){
if(n%2==0){
	document.getElementById('cam').innerHTML = print1;
}
else
{
	document.getElementById('cam').innerHTML = print2;
}
n++;
}*/

document.getElementById('camera1').innerHTML = cam1;
document.getElementById('camera2').innerHTML = cam2;
function xchange(){
if(n%2==0){
	//alert("cambia");
	/*document.getElementById('camera1').id = 'camera2';
	document.getElementById('camera2').id = 'camera1';*/
//document.getElementById('pluto').innerHTML = "<spam id='camera2'></spam>";
//document.getElementById('pippo').innerHTML = "<spam id='camera1'></spam>";
	//	alert("dopo cambia "+document.getElementById('pippo').innerHTML);
document.getElementById('camera1').innerHTML = cam2;
document.getElementById('camera2').innerHTML = cam1;
}
else
{
	//document.getElementById('pluto').innerHTML = "<spam id='camera1'></spam>";
	//document.getElementById('pippo').innerHTML = "<spam id='camera2'></spam>";
	document.getElementById('camera1').innerHTML = cam1;
	document.getElementById('camera2').innerHTML = cam2;
}
n++;
}
function xchange2(){
if(n%2==0){
	//alert("cambia");
	/*document.getElementById('camera1').id = 'camera2';
	document.getElementById('camera2').id = 'camera1';*/

	//	alert("dopo cambia "+document.getElementById('pippo').innerHTML);
var pippo;
alert(map.zoom);
document.getElementById('camera1').innerHTML = mappa;
document.getElementById('mappa').innerHTML = cam1;
var map = L.map('map').setView([45.06, 7.65], 13);
}
else
{
	document.getElementById('camera1').innerHTML = cam1;
document.getElementById('mappa').innerHTML = mappa;
var map = L.map('map').setView([45.06, 7.65], 13);
}
n++;
}
