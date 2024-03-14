let scene, camera, rendered, cube;
var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
window.addEventListener('load', onload);



function onload(event) {
  initWebSocket();
}

function initWebSocket() {
  console.log('Trying to open a WebSocket connection…');
  websocket = new WebSocket(gateway);
  websocket.onopen = onOpen;
  websocket.onclose = onClose;
  websocket.onmessage = onMessage;
}

function onOpen(event) {
  console.log('Connection opened');
  getValues();
}

function getValues(){
  websocket.send("getValues");
}

function onClose(event) {
  console.log('Connection closed');
  setTimeout(initWebSocket, 2000);
}

function updateSliderPWM(element) {
  var sliderNumber = element.id.charAt(element.id.length-1);
  var sliderValue = document.getElementById(element.id).value;
  document.getElementById("sliderValue"+sliderNumber).innerHTML = sliderValue;

  console.log(sliderValue);
  websocket.send(sliderNumber+"s"+sliderValue.toString());
}


function showSensors() {
  document.getElementById("modal").style.display = "block";
  websocket.send("SENSORS");
}

function closeSensors() {
  document.getElementById("modal").style.display = "none";
  websocket.send("SENSORS");
}


function FWD(element) {
  websocket.send("FWD");
}
function BWD(element) {
  websocket.send("BWD");
}
function RB(element) {
  websocket.send("RB");
}
function LB(element) {
  websocket.send("LB");
}
function stop(element) {
  websocket.send("stop");
}

function calibrate(element) {
  websocket.send("CALIB");
}

function Control(element) {
  websocket.send("CONTROL");
}

window.onclick = function(event) {
  if (event.target == modal) {
    modal.style.display = "none";
  }
}

function onMessage(event) {

  console.log(event.data);
  var myObj = JSON.parse(event.data);
  var keys = Object.keys(myObj);
  var key = keys[0];

  for (var i = 0; i < keys.length; i++){
      var key = keys[i];
      if (key.includes("s")) {
        document.getElementById(key).innerHTML = myObj[key];
        document.getElementById("sliderValue1".toString()).value = myObj.sliderValue1;
        document.getElementById("sliderValue2".toString()).value = myObj.sliderValue2;
        document.getElementById("sliderValue3".toString()).value = myObj.sliderValue3;
        document.getElementById("heading").innerHTML = myObj.heading;
        document.getElementById("pitch").innerHTML = myObj.pitch;
        document.getElementById("roll").innerHTML = myObj.roll;
        document.getElementById("accelX").innerHTML = myObj.ax;
        document.getElementById("accelY").innerHTML = myObj.ay;
        document.getElementById("accelZ").innerHTML = myObj.az;
      }
  }
}






