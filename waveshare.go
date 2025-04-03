// Copyright 2025 The Snow Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"encoding/json"
	"fmt"
	"time"

	"github.com/veandco/go-sdl2/sdl"
	"go.bug.st/serial"
)

var joysticks = make(map[int]*sdl.Joystick)

// Waveshare is a waveshare robot
type Waveshare struct {
	Action  TypeAction
	Port    serial.Port
	Running bool
}

// Init initializes the robot
func (w *Waveshare) Init() {
	options := &serial.Mode{
		BaudRate: 115200,
	}
	var err error
	w.Port, err = serial.Open("/dev/ttyAMA0", options)
	if err != nil {
		panic(err)
	}

	var event sdl.Event
	sdl.Init(sdl.INIT_JOYSTICK)
	defer sdl.Quit()
	sdl.JoystickEventState(sdl.ENABLE)
	w.Running = true
	var axis [5]int16
	joystickLeft := JoystickStateNone
	joystickRight := JoystickStateNone
	lightState := LightStateOff
	speed := 0.1
	var mode Mode

	go func() {
		message := map[string]interface{}{
			"T":      900,
			"main":   2,
			"module": 0,
		}
		data, err := json.Marshal(message)
		if err != nil {
			panic(err)
		}
		data = append(data, '\n')
		_, err = w.Port.Write(data)
		if err != nil {
			panic(err)
		}
		leftSpeed, rightSpeed := 0.0, 0.0
		for w.Running {
			time.Sleep(300 * time.Millisecond)
			if mode == ModeAuto {
				switch w.Action {
				case ActionForward:
					joystickLeft = JoystickStateUp
					joystickRight = JoystickStateUp
				case ActionBackward:
					joystickLeft = JoystickStateDown
					joystickRight = JoystickStateDown
				case ActionLeft:
					joystickLeft = JoystickStateDown
					joystickRight = JoystickStateUp
				case ActionRight:
					joystickLeft = JoystickStateUp
					joystickRight = JoystickStateDown
				case ActionLight:
					pwm := 0
					if lightState == LightStateOn {
						pwm, lightState = 0, LightStateOff
					} else if lightState == LightStateOff {
						pwm, lightState = 128, LightStateOn
					}
					message := map[string]interface{}{
						"T":   132,
						"IO4": pwm,
						"IO5": pwm,
					}
					data, err := json.Marshal(message)
					if err != nil {
						panic(err)
					}
					data = append(data, '\n')
					_, err = w.Port.Write(data)
					if err != nil {
						panic(err)
					}
				case ActionNone:
					joystickLeft = JoystickStateNone
					joystickRight = JoystickStateNone
				}
			}

			switch joystickLeft {
			case JoystickStateUp:
				leftSpeed = speed
			case JoystickStateDown:
				leftSpeed = -speed
			case JoystickStateNone:
				leftSpeed = 0.0
			}
			switch joystickRight {
			case JoystickStateUp:
				rightSpeed = speed
			case JoystickStateDown:
				rightSpeed = -speed
			case JoystickStateNone:
				rightSpeed = 0.0
			}

			message := map[string]interface{}{
				"T": 1,
				"L": leftSpeed,
				"R": rightSpeed,
			}
			data, err := json.Marshal(message)
			if err != nil {
				panic(err)
			}
			data = append(data, '\n')
			_, err = w.Port.Write(data)
			if err != nil {
				panic(err)
			}
		}
	}()

	go func() {
		for w.Running {
			for event = sdl.PollEvent(); event != nil; event = sdl.PollEvent() {
				switch t := event.(type) {
				case *sdl.QuitEvent:
					w.Running = false
				case *sdl.JoyAxisEvent:
					value := int16(t.Value)
					axis[t.Axis] = value
					if t.Axis == 3 || t.Axis == 4 {
						if mode == ModeManual {
							if axis[3] < 20000 && axis[3] > -20000 {
								if axis[4] < -32000 {
									joystickRight = JoystickStateUp
								} else if axis[4] > 32000 {
									joystickRight = JoystickStateDown
								} else {
									joystickRight = JoystickStateNone
								}
							} else {
								joystickRight = JoystickStateNone
							}
						}
						//fmt.Printf("right [%d ms] Which: %v \t%d %d\n",
						//              t.Timestamp, t.Which, axis[3], axis[4])
					} else if t.Axis == 0 || t.Axis == 1 {
						if mode == ModeManual {
							if axis[0] < 20000 && axis[0] > -20000 {
								if axis[1] < -32000 {
									joystickLeft = JoystickStateUp
								} else if axis[1] > 32000 {
									joystickLeft = JoystickStateDown
								} else {
									joystickLeft = JoystickStateNone
								}
							} else {
								joystickLeft = JoystickStateNone
							}
						}
						//fmt.Printf("left [%d ms] Which: %v \t%d %d\n",
						//t.Timestamp, t.Which, axis[0], axis[1])
					} else if t.Axis == 2 {
						//fmt.Printf("2 axis [%d ms] Which: %v \t%x\n",
						//      t.Timestamp, t.Which, value)
						//speed = axis[2]
						//pwm = int(100 * (float64(speed) + 32768) / 65535)
						//fmt.Printf("speed %d pwm %d\n", speed, pwm)
					}
				case *sdl.JoyBallEvent:
					fmt.Printf("[%d ms] Ball:%d\txrel:%d\tyrel:%d\n",
						t.Timestamp, t.Ball, t.XRel, t.YRel)
				case *sdl.JoyButtonEvent:
					fmt.Printf("[%d ms] Button:%d\tstate:%d\n",
						t.Timestamp, t.Button, t.State)
					if t.Button == 0 && t.State == 1 {
						switch mode {
						case ModeManual:
							mode = ModeAuto
						case ModeAuto:
							mode = ModeManual
							joystickLeft = JoystickStateNone
							joystickRight = JoystickStateNone
						}
					} else if t.Button == 1 && t.State == 1 {
						speed += .1
						if speed > .3 {
							speed = 0.1
						}
					} else if t.Button == 2 && t.State == 1 {
						pwm := 0
						if lightState == LightStateOn {
							pwm, lightState = 0, LightStateOff
						} else if lightState == LightStateOff {
							pwm, lightState = 128, LightStateOn
						}
						message := map[string]interface{}{
							"T":   132,
							"IO4": pwm,
							"IO5": pwm,
						}
						data, err := json.Marshal(message)
						if err != nil {
							panic(err)
						}
						data = append(data, '\n')
						_, err = w.Port.Write(data)
						if err != nil {
							panic(err)
						}
					}
				case *sdl.JoyHatEvent:
					fmt.Printf("[%d ms] Hat:%d\tvalue:%d\n",
						t.Timestamp, t.Hat, t.Value)
					if t.Value == 1 {
						// up
					} else if t.Value == 4 {
						// down
					} else if t.Value == 8 {
						// left
					} else if t.Value == 2 {
						// right
					}
				case *sdl.JoyDeviceAddedEvent:
					fmt.Println(t.Which)
					joysticks[int(t.Which)] = sdl.JoystickOpen(int(t.Which))
					if joysticks[int(t.Which)] != nil {
						fmt.Printf("Joystick %d connected\n", t.Which)
					}
				case *sdl.JoyDeviceRemovedEvent:
					if joystick := joysticks[int(t.Which)]; joystick != nil {
						joystick.Close()
					}
					fmt.Printf("Joystick %d disconnected\n", t.Which)
				default:
					fmt.Printf("Unknown event\n")
				}
			}

			sdl.Delay(16)
		}
	}()
}

// Do does an action
func (w *Waveshare) Do(action TypeAction) {
	w.Action = action
}

// Done the robot is done
func (w *Waveshare) Done() {
	err := w.Port.Close()
	if err != nil {
		panic(err)
	}
	w.Running = false
}
