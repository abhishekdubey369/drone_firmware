var joystick_array=[];
var game= false;

function handleMessage(message_event) {
   console.log('data from gamepad' + message_event.data);	


joystick_array=[message_event.data[2],message_event.data[3],message_event.data[1],message_event.data[0],message_event.data[15],message_event.data[16],message_event.data[17],message_event.data[18]];

var game=true;

//console.log('gamepad Values:'+ joystick_array);	

//checkGamepadConnect();

}	
/*
var checkGamepadConnect =function {
	
if(game){
console.log('if--' + game);	
var	gamepadText= 'Gamepad connected';
}
else
{
console.log('else--' + game);
	var gamepadText = 'connect gamepad';
}
	 var gamepadStatus = document.getElementById('gamepadStatus');
	 if (gamepadStatus) {
	 console.log(gamepadStatus);
      gamepadStatus.innerHTML = gamepadText;
}

}
*/