const express = require('express');
const socketIo = require('socket.io');
const http = require('http');

const app = express();
const server = http.createServer(app);
const io = socketIo.listen(server);

io.on('connection', function(socket){
    console.log("Socket connected");
});


app.get( '/', function(req, res, next) {
    res.sendFile(__dirname + '/index.html');
});


const SerialPort  = require('serialport');
const Readline =  SerialPort.parsers.Readline;
const parser = new Readline();

const mySerial = new SerialPort('COM12' , {
    baudRate : 9600

});
 mySerial.pipe(parser);


mySerial.on('open', function(){
  console.log ('Opened Serial Port');
});

parser.on ('data',function(data){
    console.log(data.toString());
    io.emit('arduino:data', {
        value: data.toString()
    });
});
mySerial.on('err',function(err){
    console.log(err.message);
});

server.listen(2000,function(){
    console.log('Listening on port 2000');
});
