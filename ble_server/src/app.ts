// /<reference path="../ts/node.d.ts" />
// /<reference path='../ts/ws.d.ts' />

/**
 * @module app.ts
 * @author Benedict R. Gaster
 * @copyright Benedict R. Gaster 2016
 *
 * Websocket server that transmits (to any number of listeners) button press
 * information from Reveal.js remote.
 *
 * I'm not happy with having to use a Websocket as it means we have to not
 * only have the Reveal.js plugin but additionally we have to run this server
 * to gather button presses from the BLE remote. A much nicer approach would
 * be to use the WebBluetooth but at time of writing Firefox has not support
 * in the browser and Chrome OS X support is very limited (51, still no read
 * functionality). I really hope this will improve overtime as it would be
 * really great to be able to directly access BLE devices from the browser.
 *
 * @license See LICENSE
 */

import WebSocket = require('ws');

// import our BLE module for the controller
import Remote from "./BLERemote";

// instance of BLE remote device
let bleRemote = new Remote()

// @fixme:br-gaster, allow port to be defined on command line
const port_ = 3000

let port: number = process.env.PORT || port_
let WebSocketServer = WebSocket.Server
let server = new WebSocketServer({ port: port })

server.on('connection', ws => {

  console.log("ws conntection made")

  // add listener for BLE remote button presses
  bleRemote.addListener(bleRemote.receivedEvent, msg => {
    // broadcast msg (button press) to all listening sockets
    broadcast(msg)
  })
})

/**
 * Send a given message to all listening sockets
 * @param {string} data to broadcast to open sockets
 */
function broadcast(data: string): void {
	server.clients.forEach(client => {
		client.send(data)
	})
}

console.log('Reveal.JS Controller Server running on port ' + port_);
