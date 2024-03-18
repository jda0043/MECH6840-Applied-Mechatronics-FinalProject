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

function sendSetpointMessage(x_relative, y_relative) {
  websocket.send("SP"+y_relative.toString()+","+x_relative.toString()+",");
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
function WS(element) {
  websocket.send("WS");
}
function HS(element) {
  websocket.send("HS");
}
function QS(element) {
  websocket.send("QS");
}
function ES(element) {
  websocket.send("ES");
}
function SS(element) {
  websocket.send("SS");
}
function calibIMU(element) {
  websocket.send("CIMU");
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
        document.getElementById("sliderValue#".toString()).value = myObj.sliderValue12;
        document.getElementById("heading").innerHTML = myObj.heading;
        document.getElementById("pitch").innerHTML = myObj.pitch;
        document.getElementById("roll").innerHTML = myObj.roll;
        document.getElementById("gX").innerHTML = myObj.gx;
        document.getElementById("gY").innerHTML = myObj.gy;
        document.getElementById("gZ").innerHTML = myObj.gz;
        document.getElementById("accelX").innerHTML = myObj.ax;
        document.getElementById("accelY").innerHTML = myObj.ay;
        document.getElementById("accelZ").innerHTML = myObj.az;
        document.getElementById("LMSpeed".toString()).innerHTML = myObj.LMSpeed;
        document.getElementById("RMSpeed".toString()).innerHTML = myObj.RMSpeed;
      }
  }
}




var canvas, ctx;

        window.addEventListener('load', () => {

            canvas = document.getElementById('canvas');
            ctx = canvas.getContext('2d');          
            resize(); 

            canvas.addEventListener('mousedown', startDrawing);
            canvas.addEventListener('mouseup', stopDrawing);
            canvas.addEventListener('mousemove', Draw);

            canvas.addEventListener('touchstart', startDrawing);
            canvas.addEventListener('touchend', stopDrawing);
            canvas.addEventListener('touchcancel', stopDrawing);
            canvas.addEventListener('touchmove', Draw);
            window.addEventListener('resize', resize);

            document.getElementById("x_coordinate").innerText = 0;
            document.getElementById("y_coordinate").innerText = 0;
            document.getElementById("speed").innerText = 0;
            document.getElementById("angle").innerText = 0;
        });

      


        var width, height, radius, x_orig, y_orig;
        function resize() {
            width = 450;
            radius = 50;
            height = 450;
            ctx.canvas.width = width;
            ctx.canvas.height = height;
            background();
            joystick(width / 2, height / 3);
        }

        function background() {
            x_orig = width / 2;
            y_orig = height / 3;

            ctx.beginPath();
            ctx.arc(x_orig, y_orig, radius + 20, 0, Math.PI * 2, true);
            ctx.fillStyle = '#ECE5E5';
            ctx.fill();
        }

        function joystick(width, height) {
            ctx.beginPath();
            ctx.arc(width, height, radius, 0, Math.PI * 2, true);
            ctx.fillStyle = 'rgb(0,0,0)';
            ctx.fill();
            ctx.strokeStyle = 'rgb(0,0,0)';
            ctx.lineWidth = 8;
            ctx.stroke();
        }

        let coord = { x: 0, y: 0 };
        let paint = false;

        function getPosition(event) {
            var mouse_x = event.clientX || event.touches[0].clientX;
            var mouse_y = event.clientY || event.touches[0].clientY;
            coord.x = mouse_x - canvas.offsetLeft;
            coord.y = mouse_y - canvas.offsetTop;
        }

        function is_it_in_the_circle() {
            var current_radius = Math.sqrt(Math.pow(coord.x - x_orig, 2) + Math.pow(coord.y - y_orig, 2));
            if (radius >= current_radius) return true
            else return false
        }


        function startDrawing(event) {
            paint = true;
            getPosition(event);
            if (is_it_in_the_circle()) {
                ctx.clearRect(0, 0, canvas.width, canvas.height);
                background();
                joystick(coord.x, coord.y);
                Draw();
            }
        }


        function stopDrawing() {
            paint = false;
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            background();
            joystick(width / 2, height / 3);
            document.getElementById("x_coordinate").innerText = 0;
            document.getElementById("y_coordinate").innerText = 0;
            document.getElementById("speed").innerText = 0;
            document.getElementById("angle").innerText = 0;
            sendSetpointMessage(x_relative, y_relative);

        }

        function Draw(event) {

            if (paint) {
                ctx.clearRect(0, 0, canvas.width, canvas.height);
                background();
                var angle_in_degrees,x, y, speed;
                var angle = Math.atan2((coord.y - y_orig), (coord.x - x_orig));

                if (Math.sign(angle) == -1) {
                    angle_in_degrees = Math.round(-angle * 180 / Math.PI);
                }
                else {
                    angle_in_degrees =Math.round( 360 - angle * 180 / Math.PI);
                }


                if (is_it_in_the_circle()) {
                    joystick(coord.x, coord.y);
                    x = coord.x;
                    y = coord.y;
                }
                else {
                    x = radius * Math.cos(angle) + x_orig;
                    y = radius * Math.sin(angle) + y_orig;
                    joystick(x, y);
                }

            
                getPosition(event);

                var speed =  Math.round(100 * Math.sqrt(Math.pow(x - x_orig, 2) + Math.pow(y - y_orig, 2)) / radius);

                var x_relative = Math.round(x - x_orig);
                var y_relative = Math.round(y - y_orig);
                

                document.getElementById("x_coordinate").innerText =  x_relative;
                document.getElementById("y_coordinate").innerText =y_relative ;
                document.getElementById("speed").innerText = speed;
                document.getElementById("angle").innerText = angle_in_degrees;

                send( x_relative,y_relative,speed,angle_in_degrees);
                sendSetpointMessage(x_relative, y_relative);
            }
        } 

