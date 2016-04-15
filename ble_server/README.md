Reveal.js Remote Websocket Server
=================================

This components implements a simple Websocket server that scans for a Reveal.js
Remote BLE peripherals, connects if one exists, and sends button press messages
to any connected sockets.

The server is run on:

    http://localhost:3000

and sends the messages:

| Message | Comment           |
| --------------------------- |
| 'left'  | Move slide left   |
| 'right' | Move slide right  |
| 'up'    | Move slide up     |
| 'down'  | Move slide down   |

See the component *ble_remote* for an example of a Reveal.js plugin that
communicates with this component and directs Reveal.js to move within a given
presentation.

The component *ble_peripheral* provides an example implementation of a
Reveal.js BLE peripheral, using the Nordic SDK for Redbear's Nano BLE board.

Installation
============

The server is implemented with [Node.js](https://nodejs.org/) and [Typescript](https://www.typescriptlang.org/). As such you need to install
Node.js, following the instructions on for your particular platform:

&nbsp;&nbsp;&nbsp; [https://nodejs.org/en/](https://nodejs.org/)

Once installed you need to install Typescript's typings, using the command:

    npm install -g typing

Next you need to install the Node.js and *typing* dependencies with the
commands:

    npm install
    typings install

Finally, you need to compile the Typescript source files to Javascript,
using the following command:

    npm run build

Running
=======

Assuming you have completed the steps in the Section Installation, then to
start the server simple run the command:

    node start

which will display the message:

    Reveal.JS Controller Server running on port 3000

License
=======

See LICENSE file.
