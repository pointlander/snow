// Copyright 2025 The Snow Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"fmt"

	"github.com/veandco/go-sdl2/sdl"
)

// Joy is a joystick state
type Joy struct {
	Mode          Mode
	Speed         float64
	JoystickLeft  JoystickState
	JoystickRight JoystickState
	LightState    LightState
}

// Joystick is a joystick
type Joystick struct {
	Running bool
	Joy     chan Joy
}

// NewJoystick creates a new joystick
func NewJoystick() *Joystick {
	joystick := &Joystick{
		Running: true,
		Joy:     make(chan Joy, 8),
	}
	go func() {
		var joysticks = make(map[int]*sdl.Joystick)
		state := Joy{
			JoystickLeft:  JoystickStateNone,
			JoystickRight: JoystickStateNone,
			LightState:    LightStateOff,
			Speed:         0.1,
		}
		var axis [5]int16
		var event sdl.Event
		sdl.Init(sdl.INIT_JOYSTICK)
		defer sdl.Quit()
		sdl.JoystickEventState(sdl.ENABLE)
		for joystick.Running {
			for event = sdl.PollEvent(); event != nil; event = sdl.PollEvent() {
				switch t := event.(type) {
				case *sdl.QuitEvent:
					joystick.Running = false
				case *sdl.JoyAxisEvent:
					value := int16(t.Value)
					axis[t.Axis] = value
					if t.Axis == 3 || t.Axis == 4 {
						if state.Mode == ModeManual {
							if axis[3] < 20000 && axis[3] > -20000 {
								if axis[4] < -32000 {
									state.JoystickRight = JoystickStateUp
								} else if axis[4] > 32000 {
									state.JoystickRight = JoystickStateDown
								} else {
									state.JoystickRight = JoystickStateNone
								}
							} else {
								state.JoystickRight = JoystickStateNone
							}
						}
						//fmt.Printf("right [%d ms] Which: %v \t%d %d\n",
						//              t.Timestamp, t.Which, axis[3], axis[4])
					} else if t.Axis == 0 || t.Axis == 1 {
						if state.Mode == ModeManual {
							if axis[0] < 20000 && axis[0] > -20000 {
								if axis[1] < -32000 {
									state.JoystickLeft = JoystickStateUp
								} else if axis[1] > 32000 {
									state.JoystickLeft = JoystickStateDown
								} else {
									state.JoystickLeft = JoystickStateNone
								}
							} else {
								state.JoystickLeft = JoystickStateNone
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
						switch state.Mode {
						case ModeManual:
							state.Mode = ModeAuto
						case ModeAuto:
							state.Mode = ModeManual
							state.JoystickLeft = JoystickStateNone
							state.JoystickRight = JoystickStateNone
						}
					} else if t.Button == 1 && t.State == 1 {
						state.Speed += .1
						if state.Speed > .3 {
							state.Speed = 0.1
						}
					} else if t.Button == 2 && t.State == 1 {
						state.LightState = LightStateOn
					} else if t.Button == 3 && t.State == 1 {
						state.LightState = LightStateOff
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
				joystick.Joy <- state
			}

			sdl.Delay(16)
		}
	}()
	return joystick
}
