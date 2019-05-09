#include <pgmspace.h>
char index_html[] PROGMEM = R"=====(
<!DOCTYPE html>
	<head>
	<meta charset="utf-8"> <title>Tiger Tank (V1.1)</title>
	<meta name="viewport" content="width=device-width, user-scalable=no">

	<style>

	.switch, .voltage {
		position: relative;
		display: block;
		margin-left: auto;
		margin-right: auto;
		width: 34px;
		height: 34px;
	}

	.voltage {
		position: relative;
		display: block;
		margin-left: auto;
		margin-right: auto;
		width: 54px;
		height: 34px;
	}

	.switch input {display:none;}

	.slider {
		position: absolute;
		cursor: pointer;
		border-radius: 34px;
		top: 0;
		left: 0;
		right: 0;
		bottom: 0;
		background-color: #F00;
	}

	input:checked + .slider {
		background-color: #0F0;
	}

	</style>
	</head>
	<body onload="start()">
	 <div class="voltage" id="vbat_min_div" style="display: none">VMin: <input id="vbat_min" type="number" min="0.1" max="5.0" step="0.05" value="4.0" style="width: 50px"/>
	 </div>
	<br>
	<label class="switch">
		<input type="checkbox" id="Button0" onchange="Button1Change(this)">
		<span class="slider"></span>
	</label>
	<br>
	<label class="switch">
		<input type="checkbox" id="Button1" onchange="Button2Change(this)">
		<span class="slider"></span>
	</label>
	<br>
	<label class="switch">
		<input type="checkbox" id="Button2" onchange="Button3Change(this)">
		<span class="slider"></span>
	</label>
	<br>
	<label class="switch">
		<input type="checkbox" id="Button3" onchange="Button4Change(this)">
		<span class="slider"></span>
	</label>
	
	<br>
	<label class="switch">
		<input type="checkbox" id="Button4" onchange="Button5Change(this)">
		<span class="slider"></span>
	</label>

	
	<br>
	<label class="switch">
		<input type="checkbox" id="Button5" onchange="Button6Change(this)">
		<span class="slider"></span>
	</label>

	
	<br>
	<label class="switch">
		<input type="checkbox" id="Button6" onchange="Button7Change(this)">
		<span class="slider"></span>
	</label>


		<div class="voltage" id="vbat" style="display: none">0 V</div>


	<div id=textdiv style="position:absolute;top:0px;left:0px;"></div>
	<canvas id="Canvas_left" style="border:1px solid #d3d3d3;position:absolute; top:10%; left:5%; z-index:0">
	Your browser does not support the HTML5 canvas tag.</canvas>
	<canvas id="Canvas_right" style="border:1px solid #d3d3d3;position:absolute; top:10%; left:55%; z-index:0">
	Your browser does not support the HTML5 canvas tag.</canvas>
	<canvas id="Canvas_stickl" style="position:absolute; top:10%; left:5%; z-index:1">
	Your browser does not support the HTML5 canvas tag.</canvas>
	<canvas id="Canvas_stickr" style="position:absolute; top:10%; left:55%; z-index:1">
	Your browser does not support the HTML5 canvas tag.</canvas>

	<script>
	var touches = [];
	var w = 0;
	var wsconnect = 0;
	var h = 0;
	var end=0;
	var ctx_left;
	var ctx_right;
	var ctx_stickl;
	var ctx_stickr;
	var buttons=[0,0,0,0];
	var ppm=[0,0,0,0,0,1100,1100,1100];
	var oldppm=[0,0,0,0,0,0,0,0];
	var pwmtargets = [0,0,0,0,0];
	var getInterval;
	var aliveInterval;
	var visible=0;
	var decaymode = 0;
	
	//var connection = new WebSocket('ws://' + window.location.host + ':81', ['arduino']);
	var connection = new WebSocket('ws://' +'192.168.4.1' + ':81', ['arduino']);

	connection.onopen = function () {		 //open
		console.log("Websocket Open");
		wsconnect=1;
		window.requestAnimationFrame(update);
		draw_stick(ctx_stickl,ctx_stickl.canvas.width/2,ctx_stickl.canvas.height/2,0,1);
		draw_stick(ctx_stickr,ctx_stickr.canvas.width/2,ctx_stickr.canvas.height/2,2,3);
		aliveInterval=setInterval(sendalive,150);
	};

	connection.onerror = function (error) { //error
		console.log('WebSocket Error ' + error);
		wsconnect=0;
		draw_stick(ctx_stickl,ctx_stickl.canvas.width/2,ctx_stickl.canvas.height/2,0,1);
		draw_stick(ctx_stickr,ctx_stickr.canvas.width/2,ctx_stickr.canvas.height/2,2,3);
		clearInterval(getInterval);
	};

	connection.onmessage = function (e) {	 //Daten empfangen
		 console.log("indata: " + e.data);
	}

	connection.onclose = function (e)
	{
		console.log("Websocket close");
		wsconnect=0;
		draw_stick(ctx_stickl,ctx_stickl.canvas.width/2,ctx_stickl.canvas.height/2,0,1);
		draw_stick(ctx_stickr,ctx_stickr.canvas.width/2,ctx_stickr.canvas.height/2,2,3);
		clearInterval(getInterval);
	}
	function getCursorPosition(canvas, event) {
		var rect = canvas.getBoundingClientRect();
		var x =(2.0*(event.clientX - rect.left)/canvas.width) -1.0;
		var y =(2.0*(event.clientY - rect.top )/canvas.height)-1.0;
		console.log("x: " + x + " y: " + y);
		return {mx:x, my:y}
	}
	function start()
	{ 
		var c_left = document.getElementById("Canvas_left");
		ctx_left = c_left.getContext("2d");

		var c_right = document.getElementById("Canvas_right");
		ctx_right = c_right.getContext("2d");

		var c_stickl = document.getElementById("Canvas_stickl");
		ctx_stickl = c_stickl.getContext("2d");

		var c_stickr = document.getElementById("Canvas_stickr");
		ctx_stickr = c_stickr.getContext("2d");

		update();

		draw_background(ctx_left);
		draw_background(ctx_right);
		draw_stick(ctx_stickl,c_stickl.width/2,c_stickl.height/2,0,1);
		draw_stick(ctx_stickr,c_stickr.width/2,c_stickr.height/2,2,3);

		window.addEventListener("optimizedResize", function() {
			resize();
		});
		window.addEventListener("orientationchange", function() {
			window.setTimeout(resize, 300)
		});	
		c_stickl.addEventListener('touchend', function() {
			console.log( "endl");
			draw_stick(ctx_stickl,c_stickl.width/2,c_stickl.height/2,0,1);
		});
		c_stickl.addEventListener('touchmove', function(event) {
			event.preventDefault();
			touches = event.touches;
			window.requestAnimationFrame(update);
		});

		
		
		c_stickl.addEventListener('click', function(event) {
			console.log('stickl_clicked_at');
			console.log(event);
			var coords = getCursorPosition(this,event);
			ppm[1] = 0;
			ppm[2] = coords.my;
			sendtankdata();
		});
		
		
		c_stickr.addEventListener('click', function(event) {
			console.log('stickr_clicked_at');
			console.log(event);
			var coords = getCursorPosition(this,event);
			ppm[1] = coords.mx;
			ppm[2] = 0;
			sendtankdata();
		});
		
		

		c_stickl.addEventListener('touchstart', function(event) {
			console.log('startl');
		});
		c_stickr.addEventListener('touchend', function() {
			console.log("endr");
			end=1;
			draw_stick(ctx_stickr,c_stickr.width/2,c_stickr.height/2,2,3);
		});
		c_stickr.addEventListener('touchmove', function(event) {
			event.preventDefault();
			touches = event.touches;
			window.requestAnimationFrame(update);
		});
		c_stickr.addEventListener('touchstart', function(event) {
			console.log('startr');
			end=0;
		});
	};

	function getVBat(){
		if(wsconnect){
		console.log("get");
		connection.send("get");
		}
	}

	function Button1Change(checkbox)
	{
		if(checkbox.checked==true) ppm[4] = ppm[4] | 0x01;
				
						 
	  
				
		else ppm[4] = ppm[4] & (~0x01);
		window.requestAnimationFrame(update);
		console.log("Button1: " + ppm[4]);
	}
	function Button2Change(checkbox)
	{
		if(checkbox.checked==true) ppm[4] = ppm[4] | 0x02;
						 
	  
		else ppm[4] = ppm[4] & (~0x02);
		window.requestAnimationFrame(update);
		console.log("Button2: " + ppm[4]);
	}
	function Button3Change(checkbox)
	{
		if(checkbox.checked==true) ppm[4] = ppm[4] | 0x04;
						 
	  
		else ppm[4] = ppm[4] & (~0x04);
		window.requestAnimationFrame(update);
		console.log("Button3: " + ppm[4]);
	}
	
	function Button4Change(checkbox)
	{
		if(checkbox.checked==true) ppm[4] = ppm[4] | 0x08;
						 
	  
		else ppm[4] = ppm[4] & (~0x08);
		window.requestAnimationFrame(update);
		console.log("Button4: " + ppm[4]);
	}
	
	function Button5Change(checkbox)
	{
		if(checkbox.checked==true) ppm[4] = ppm[4] | 0x10;
						 
	  
		else ppm[4] = ppm[4] & (~0x10);
		window.requestAnimationFrame(update);
		console.log("Button5: " + ppm[4]);
	}
	
	function Button6Change(checkbox)
	{
		if(checkbox.checked==true) ppm[4] = ppm[4] | 0x20;
						 
	  
		else ppm[4] = ppm[4] & (~0x20);
		window.requestAnimationFrame(update);
		console.log("Button6: " + ppm[4]);
	}
	 function Button7Change(checkbox)
	{
		if(checkbox.checked==true){
			ppm[4] = ppm[4] | 0x40;
			decaymode = 1;
		}else{
			ppm[4] = ppm[4] & (~0x40);
			decaymode = 0;
		}
	  
							 
				
   
		window.requestAnimationFrame(update);
		console.log("Button7: " + ppm[4]);
	}
	

	function resize()
	{
		ctx_left.canvas.height=window.innerHeight-(window.innerHeight/10*2);
		ctx_left.canvas.width=(window.innerWidth-(window.innerWidth/10*2))/2;

		ctx_right.canvas.height=window.innerHeight-(window.innerHeight/10*2);
		ctx_right.canvas.width=(window.innerWidth-(window.innerWidth/10*2))/2;

		ctx_stickl.canvas.height=ctx_left.canvas.height;
		ctx_stickl.canvas.width=ctx_left.canvas.width;

		ctx_stickr.canvas.height=ctx_right.canvas.height;
		ctx_stickr.canvas.width=ctx_right.canvas.width;

		draw_background(ctx_left);
		draw_background(ctx_right);
		draw_stick(ctx_stickl,ctx_stickl.canvas.width/2,ctx_stickl.canvas.height/2);
		draw_stick(ctx_stickr,ctx_stickr.canvas.width/2,ctx_stickr.canvas.height/2);

	}

	function draw_stick(context,x,y,ppm0,ppm1)
	{
		context.clearRect(0, 0, context.canvas.width, context.canvas.height);

			context.beginPath();
			context.arc(x,y,window.innerWidth/100*2,0,2*Math.PI);
			if(wsconnect)
				context.fillStyle = 'green';
			else
				context.fillStyle = 'red';
			context.fill();
			context.lineWidth = 5;
			context.strokeStyle = '#003300';
			context.stroke();
		ppm[ppm0] = parseFloat((2.0*x/context.canvas.width)-1.0);
		ppm[ppm1] = parseFloat((2.0*y/context.canvas.height)-1.0);
	}


	function draw_background(ctx)
	{
			 ctx.beginPath();
			 for(var i=0;i<ctx.canvas.width/2;i+=ctx.canvas.width/20)
			 {
					 ctx.moveTo(ctx.canvas.width/2+i,ctx.canvas.height/2);
					 ctx.arc(ctx.canvas.width/2,ctx.canvas.height/2,i,0,2*Math.PI);
			 }
			 ctx.moveTo(0,ctx.canvas.height/2);
			 ctx.lineTo(ctx.canvas.width,ctx.canvas.height/2);
			 ctx.moveTo(ctx.canvas.width/2,0);
			 ctx.lineTo(ctx.canvas.width/2,ctx.canvas.height);
			 ctx.stroke();
	};


	function update() {
		var nw = window.innerWidth;
		var nh = window.innerHeight;
		if ((w != nw) || (h != nh)) {
		w = nw;
		h = nh;
		resize();
		}

		var i, len = touches.length;
		var left=0;
		var right=0;
		for (i=0; i<len; i++) {
			var touch = touches[i];
			var px = touch.pageX-touch.target.offsetLeft;
			var py = touch.pageY-touch.target.offsetTop;
			console.log(touch.target.id);
			if(touch.target.id=="Canvas_stickl" && !left)
			{
			if(px>ctx_stickl.canvas.width)
				px=ctx_stickl.canvas.width;
			if(py<0)
				py=0;
			if(px<0)
				px=0;
			if(py>ctx_stickl.canvas.height)
				py=ctx_stickl.canvas.height;
			draw_stick(ctx_stickl,px,py,0,1);
			left=1;
			}
			if(touch.target.id=="Canvas_stickr" && !right && !end)
			{
			if(px>ctx_stickr.canvas.width)
				px=ctx_stickr.canvas.width;
			if(py<0)
				py=0;
			if(px<0)
				px=0;
			if(py>ctx_stickr.canvas.height)
				py=ctx_stickr.canvas.height;
			draw_stick(ctx_stickr,px,py,2,3);
			right=1;
			}
		}
		
		sendtankdata();
		
		
	}
	function sendalive(){
		console.log("sendalive");
		sendtankdata();
		return;
	}
	
	function sendtankdata(){
		//So we know the ppm order: LX, LY, RX, RY
		//the ppm range needs to be transformed to (it already is :) )[-1,1]
		var LY = ppm[2];
		var RX = ppm[1];
		var axis_fwrev	 = LY*Math.abs(LY)*1.0+ LY*Math.abs(LY)*0.5*Math.abs(RX);
		var axis_leftright = Math.abs(RX) * RX;
		var mul = 255;
		var left_reverse = Math.round(mul * Math.max(0, Math.min(1.0,axis_leftright - axis_fwrev)) );
		var left_forward = Math.round(mul * Math.max(0, Math.min(1.0,-(axis_leftright - axis_fwrev)))); 
		var right_reverse= Math.round(mul * Math.max(0, Math.min(1.0,axis_leftright + axis_fwrev)));
		var right_forward= Math.round(mul * Math.max(0, Math.min(1.0,-(axis_leftright + axis_fwrev))));
		var l_pwm = Math.max(left_forward,left_reverse);
        var r_pwm = Math.max(right_forward,right_reverse);
		if (decaymode == 1){
			if (left_reverse >0){
				left_forward = 255 - left_reverse;
				left_reverse = 255;
			}else if(left_forward>0){
				left_reverse = 255 - left_forward;
				left_forward = 255;
			}
			if (right_reverse >0){
				right_forward = 255 - right_reverse;
				right_reverse = 255;
			}else if(right_forward>0){
				right_reverse = 255 - right_forward;
				right_forward = 255;
			}
		}
		
		console.log('ppm '+ppm);
		console.log('LY '+LY);
		console.log('RX '+RX);
		console.log('axis_fwrev '+axis_fwrev);
		console.log('axis_leftright '+axis_leftright);
		console.log('left_reverse '+left_reverse);
		console.log('left_forward '+left_forward);
		console.log('right_reverse '+right_reverse);
		console.log('right_forward '+right_forward);
		console.log('buttons '+ppm[4]);
		var sendframe = new Uint8Array(5);
		sendframe[0] = left_reverse;
		sendframe[1] = left_forward;
		sendframe[2] = right_reverse;
		sendframe[3] = right_forward;
		sendframe[4] = ppm[4];
		if (wsconnect) connection.send(sendframe);
		let newtext = "LF="+Math.floor(left_forward)+" LR="+Math.floor(left_reverse)+" RF="+Math.floor(right_forward)+" RR="+Math.floor(right_reverse) + " 0b"+ppm[4].toString(2);
		document.getElementById("textdiv").innerHTML = newtext;
	}

	</script>


	</body>
	</html>
	)=====";
