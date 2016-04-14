/**
 * @module remote.js
 * @author Benedict R. Gaster
 * @copyright Benedict R. Gaster 2016
 *
 * BLE-based remote controller for your presentation. Checkout
 *
 *
 * @license: See LICENCE
 */
var start = new Date();

(function(window) {
    /**
     * Check that browser supports Web socket API, so far tested on
     *   Chrome and Firefox
     */
    if ("WebSocket" in window) {
	     console.log("Initializing ARemote...")

	      // open up remote server
        //var ws = new WebSocket("ws://localhost:9998/remote")
        var ws = new WebSocket('ws://localhost:3000');

	      ws.onopen = function() {
            // Web Socket is connected, send data using send()
            // currently nothing to send
        }

        /**
         * Handle an incomming button press
         * @param  {[type]} evt data payload is 'left', 'right', 'down', or
         * 'up', mapping to slide movement
         */
        ws.onmessage = function (evt) {
            // cheap debouncing (hack alert...)
            var time = new Date() - start

            // only handle events 1sec apart, in truth this mostly works fine
            // but does mean you can't rush through the slides
            // @fixme:bgaster add debouncing to BLE app not here :-)
            if (time > 1000) {
              switch(evt.data) {
                case 'left':
                  Reveal.left()
                  break
                case 'right':
                  Reveal.right()
                  break
                case 'up':
                  Reveal.up()
                  break
                case 'down':
                  Reveal.down()
                  break
              }

              // get a new time instance
              start = new Date();
            }
        }

        ws.onclose = function() {
            // websocket is closed.
        }
    }
})(window);
