const WebSocket = require('ws');
const express = require('express');
const path = require("path");
const switchboard = require("./switchboard");

const app = express();
const source_socket = new WebSocket.Server({ port: 8080 });
const client_socket = new WebSocket.Server({ port: 8081 });
const switch_op = switchboard();

source_socket.on('connection', (ws) => {
  let auth = true;
  if (auth) {
    switch_op.initSource(ws);
  }
});

client_socket.on('connection', (ws) => {
  let auth = true;
  if (auth) {
    switch_op.initClient(ws);
  }
});


app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname+'/public/index.html')); 
});

app.get('/kinect', (req, res) => {
  res.sendFile(path.join(__dirname+'/public/kinect.html')); 
});

app.listen(3000, () => {
  console.log("Listening on port 3000");
});
