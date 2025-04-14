// Copyright 2025 The Snow Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"encoding/json"
	"time"

	"go.bug.st/serial"
)

// Waveshare is a waveshare robot
type Waveshare struct {
	Action   TypeAction
	Port     serial.Port
	Running  bool
	Joystick *Joystick
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

	w.Joystick = NewJoystick()

	w.Running = true
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

		var state Joy
		tick := time.Tick(300 * time.Millisecond)
		for w.Running {
			select {
			case state = <-w.Joystick.Joy:
			case <-tick:
			}
			leftSpeed, rightSpeed, pwm := 0.0, 0.0, 0
			if state.Mode == ModeAuto {
				switch w.Action {
				case ActionForward:
					leftSpeed = state.Speed
					rightSpeed = state.Speed
				case ActionBackward:
					leftSpeed = -state.Speed
					rightSpeed = -state.Speed
				case ActionLeft:
					leftSpeed = -state.Speed
					rightSpeed = state.Speed
				case ActionRight:
					leftSpeed = state.Speed
					rightSpeed = -state.Speed
				case ActionLightOn:
					pwm = 0
				case ActionLightOff:
					pwm = 128
				case ActionNone:
					leftSpeed = 0.0
					rightSpeed = 0.0
				default:
				}
			} else {
				switch state.JoystickLeft {
				case JoystickStateUp:
					leftSpeed = state.Speed
				case JoystickStateDown:
					leftSpeed = -state.Speed
				case JoystickStateNone:
					leftSpeed = 0.0
				}
				switch state.JoystickRight {
				case JoystickStateUp:
					rightSpeed = state.Speed
				case JoystickStateDown:
					rightSpeed = -state.Speed
				case JoystickStateNone:
					rightSpeed = 0.0
				}
				switch state.LightState {
				case LightStateOn:
					pwm = 0
				case LightStateOff:
					pwm = 128
				}
			}

			{
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
			{
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
	w.Joystick.Running = false
	w.Running = false
}
