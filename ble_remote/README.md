Reveal.js Remote Plugin
=======================

This component contains a simple plugin for Reveal.js that uses WebSocket
to communicate with the Reveal.js BLE remote. Component *ble_server*
implements the server that communicates with the BLE remote peripheral.

Installation
============

The simplest way to install the plugin is to copy the directory *ble_remote*
to the directory *plugin* in your Reveal.js install. Alternatively on
Windows or OS X creating a symbolic link to the directory *ble_remote* in
this repo works equally as well.

Once the plugin is installed within your Reveal.js you need to update
your presentations configuration to correctly load the plugin.

Example configuration:

    Reveal.initialize({
       // other options...

       // Don't forget to add the dependencies
       dependencies: [
          { src: 'plugin/ble_remote/remote.js', async: true }

        // other dependencies...
       ]
