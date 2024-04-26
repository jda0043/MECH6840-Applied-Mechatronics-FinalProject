
var gateway = `ws://192.168.4.1/ws`;
var websocket;



function connectWS(event) {
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



function enCont(element) {
  websocket.send("ENCON");
}
function disCont(element) {
  websocket.send("DISCON");
}
function enMot(element) {
  websocket.send("ENMOT");
}
function disMot(element) {
  websocket.send("DISMOT");
}

function underDamp(element) {
  websocket.send("UD");
}
function critDamp(element) {
  websocket.send("CD");
}
function overDamp(element) {
  websocket.send("OD");
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
        document.getElementById("sliderValue4".toString()).value = myObj.sliderValue4;
        document.getElementById("sliderValue5".toString()).value = myObj.sliderValue5;
        document.getElementById("sliderValue6".toString()).value = myObj.sliderValue6;
        document.getElementById("sliderValue7".toString()).value = myObj.sliderValue7;
        document.getElementById("sliderValue8".toString()).value = myObj.sliderValue8;
        document.getElementById("sliderValue9".toString()).value = myObj.sliderValue9;
        document.getElementById("sliderValue0".toString()).value = myObj.sliderValue0;
        document.getElementById("sliderValue!".toString()).value = myObj.sliderValue11;
        document.getElementById("sliderValue@".toString()).value = myObj.sliderValue12;
        document.getElementById("sliderValue#".toString()).value = myObj.sliderValue13;
        document.getElementById("sliderValue$".toString()).value = myObj.sliderValue14;
        document.getElementById("heading").innerHTML = myObj.heading;
        document.getElementById("pitch").innerHTML = myObj.pitch;
        document.getElementById("roll").innerHTML = myObj.roll;
        document.getElementById("gZ").innerHTML = myObj.gZ;
        document.getElementById("aX").innerHTML = myObj.aX;
        document.getElementById("LMSpeed".toString()).innerHTML = myObj.LMSpeed;
        document.getElementById("RMSpeed".toString()).innerHTML = myObj.RMSpeed;
        document.getElementById("psp".toString()).innerHTML = myObj.SetpointP;
        document.getElementById("ssp".toString()).innerHTML = myObj.SetpointS;

        updateButtons(myObj.ConEN, myObj.MotEN);
        
      }
  }
  
  
}


function updateButtons(ConEN, MotEN, stepCount) {
  switch(ConEN) {
    case "1":
      document.getElementById("enableController").style.backgroundColor = 'rgb(220, 252, 196)';
      document.getElementById("disableController").style.backgroundColor = 'rgb(255,255,255)';
      break;
    case "0":
      document.getElementById("enableController").style.backgroundColor = 'rgb(255,255,255)';
      document.getElementById("disableController").style.backgroundColor = 'rgb(243, 113, 113)';
      break;

  }
  switch(MotEN) {
    case "1":
      document.getElementById("enableMotors").style.backgroundColor = 'rgb(220, 252, 196)';
      document.getElementById("disableMotors").style.backgroundColor = 'rgb(255,255,255)';
      break;
    case "0":
      document.getElementById("enableMotors").style.backgroundColor = 'rgb(255,255,255)';
      document.getElementById("disableMotors").style.backgroundColor = 'rgb(243, 113, 113)';
      break;

  }
  
}

