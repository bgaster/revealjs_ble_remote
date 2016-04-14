// / <reference path="../ts/noble.d.ts" />

/**
  * @module BLERemote.ts
  * @author Benedict R. Gaster
  * @copyright Benedict R. Gaster 2016
  *
  * BLE class to connect to Reveal.js controller.
  * A successful connection uses the controller characteristic to read
  * values which are emitted
  *
  * @license See LICENSE
 */

import noble = require("noble")   // Node BLE library
import events = require("events") // Node emitter library

const CONTROL_RATE_VALUE_FORMAT: number = 1

// Servive UUID for physical remote
const CONTROLLER_SERVICE_UUID: string = 'A943A80B083F44D3AED71CB57B082F33'
// characteristic UUID for physical remote
const CONTROLLER_CHARACTERISTIC_UUID: string = 'B4FBC6CE380F4EC1BE0AD163EFCF02C4'

/**
 * BLE message data received for left button pressed on remote
 * @type {number}
 */
const BUTTON_LEFT: number = 2
/**
 * BLE message data received for right button pressed on remote
 * @type {number}
 */
const BUTTON_RIGHT: number = 16
/**
 * BLE message data received for up button pressed on remote
 * @type {number}
 */
const BUTTON_UP: number = 8
/**
 * BLE message data received for down button pressed on remote
 * @type {number}
 */
const BUTTON_DOWN: number = 4

/**
 * Emitted message for left button press
 * @type {number}
 */
const EMIT_LEFT: string  = 'left'
/**
 * Emitted message for right button press
 * @type {number}
 */
const EMIT_RIGHT: string = 'right'
/**
 * Emitted message for up button press
 * @type {number}
 */
const EMIT_UP: string    = 'up'
/**
 * Emitted message for down button press
 * @type {number}
 */
const EMIT_DOWN:string   = 'down'

 /**
  * Represents a BLE connection with the physical Reveal.js remote.
  * Events received from the connected remote are "emitted" as received events
  * to any listerner.
  */
export default class Remote extends events.EventEmitter {
  // name of listerner for emitted button presses
  private receivedEvent_ : string

  // handle to matched BLE characteristics once a remote is discovered
  private controllerChar_: noble.Characteristic = void 0

  // handle to matched BLE peripheral once a remote is discovered
  private peripheral_ : noble.Peripheral

  /**
   * Create an instance of Remote.
   * <p>
   * @param  {string = 'received'}  receivedEvent is optional but if set it will
   * be used as the name when events received from
   * the remote is emitted.
   */
  public constructor(receivedEvent: string = 'received') {
    super()

    this.receivedEvent_ = receivedEvent

    noble.on('stateChange', function(state) {
      if (state == 'poweredOn')
        noble.startScanning([], false)
      else
        noble.stopScanning()
    })

    noble.on('discover', this.handlePeripheral)
  }

  /**
   * Getter that is the name of the event to listed to for BLE button presses
   * @return {string} listener for emitted BLE button events
   */
  public get receivedEvent(): string {
       //console.log("receivedEvent_ = " + this.receivedEvent_)
       return this.receivedEvent_;
  }

  /**
   * callback used when a BLE peripheral is discovered close by. Note
   * that at this point it could be any BLE peripheral and we first
   * connect and then discover its services.
   * @param peripheral is the found BLE peripheral for a Reveal.js remote.
   */
  private handlePeripheral = (peripheral : noble.Peripheral) => {
    //console.log("found peripheral with UUID: " + peripheral.uuid)

    this.peripheral_ = peripheral

    peripheral.connect((error: string) => {
      //console.log("connected")

    // now see if it actually supports teh services we are interested in,
    // i.e. Reveal.js remote.
    this.peripheral_.discoverServices(
      [CONTROLLER_SERVICE_UUID],
      (error: string, services: noble.Service[]) => {
            //console.log("connected to controller service")
            this.handleControllerService(services[0])
      })
    })

    if (peripheral.advertisement) {
        //console.log("Name: " + peripheral.advertisement.localName)
        //console.log("Service UUID: " + peripheral.advertisement.serviceUuids[0])
    }
  }

  /**
   * Discover remote characteristic for matching service and install handler for read
   * messages, in this case button presses on remote device
   * @param service represetents the remote service matched with BLE peripheral
   */
  private handleControllerService = (service: noble.Service) => {
    //console.log("found service: " + service.uuid)
    service.discoverCharacteristics(
        [CONTROLLER_CHARACTERISTIC_UUID],
        (error: String, characteristics: noble.Characteristic[]) => {
          //console.log("dicovered controller characteristic")
          console.log("BLE remote connected")

          // make sure we keep the characteristic around for on going use
          this.controllerChar_ = characteristics[0]

          // setup handler for read notifications, i.e. process button presses
          // from device and emit them as messages for any listeners
          this.controllerChar_.on(
            'read',
            (data: NodeBuffer, isNotification: boolean) => {
              var controlVal = this.readControl(data)
              switch (controlVal) {
                case BUTTON_LEFT:
                  this.emit(this.receivedEvent_, EMIT_LEFT)
                  break
                case BUTTON_RIGHT:
                  this.emit(this.receivedEvent_, EMIT_RIGHT)
                  break
                case BUTTON_UP:
                  this.emit(this.receivedEvent_, EMIT_UP)
                  break
                case BUTTON_DOWN:
                  this.emit(this.receivedEvent_, EMIT_DOWN)
                  break
                default:
                  // do nothing
                  break
              }
              //console.log("received = " + controlVal)
          })

          this.controllerChar_.notify(true)
      })
  }

  /**
   * Extract the button press from the BLE packet received from device
   * @param  {[type]} val BLE packet sent from device
   * @return {number}  the value (button press) passed in BLE packet
   */
  private readControl(val): number {
      var flags = val.readUInt8(0)
      if (!((flags & CONTROL_RATE_VALUE_FORMAT) != CONTROL_RATE_VALUE_FORMAT)) {
          return 0
      }
      return val.readUInt8(1)
  }

} // class Remote
