// Copyright 2025 The Snow Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"math"
	"time"

	"go.bug.st/serial"
)

type (
	PacketControllerState uint
	PacketFunction        uint
	PacketReportKeyEvents uint
)

const (
	PacketControllerStateStartbyte1 PacketControllerState = iota
	PacketControllerStateStartbyte2
	PacketControllerStateLength
	PacketControllerStateFunction
	PacketControllerStateId
	PacketControllerStateData
	PacketControllerStateCheckout
)

const (
	PacketFuncSys PacketFunction = iota
	PacketFuncLED
	PacketFuncBuzzer
	PacketFuncMotor
	PacketFuncPWMServo
	PacketFuncBusServo
	PacketFuncKey
	PacketFuncIMU
	PacketFuncGamepad
	PacketFuncSBus
	PacketFuncOLED
	PacketFuncRGB
	PacketFuncNone
)

const (
	KeyEventPressed PacketReportKeyEvents = 1 << iota
	KeyEventLongpress
	KeyEventLongpressRepeat
	KeyEventReleaseFromLP
	KeyEventReleaseFromSP
	KeyEventClick
	KeyEventDoubleClick
	KeyEventTripleClick
)

var CRC8Table = []byte{
	0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
	157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
	35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
	190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
	70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
	219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
	101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
	248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
	140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
	17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
	175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
	50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
	202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
	87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
	233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
	116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53,
}

// ChecksumCRC8 computes the 8 bit crc checksum
func ChecksumCRC8(data []byte) byte {
	check := byte(0)
	for _, b := range data {
		check = CRC8Table[check^b]
	}
	return check
}

// GeneratePacket generates a packet
func GeneratePacket(function PacketFunction, data []byte) []byte {
	buf := []byte{0xAA, 0x55, byte(function), byte(len(data))}
	buf = append(buf, data...)
	return append(buf, ChecksumCRC8(buf[2:]))
}

// GeneratePacketMotorDuty generates a packet for the motor duty
func GeneratePacketMotorDuty(duty [4]int32) []byte {
	data := []byte{0x05, byte(len(duty))}
	for i, d := range duty {
		data = append(data, byte(i))
		dd := math.Float32bits(float32(d))
		for j := 0; j < 4; j++ {
			data = append(data, byte(dd>>(j*8)))
		}
	}
	return GeneratePacket(PacketFuncMotor, data)
}

// Turbopi is a turbopi robot
type Turbopi struct {
	Action   TypeAction
	Port     serial.Port
	Running  bool
	Joystick *Joystick
}

// Init initializes the robot
func (t *Turbopi) Init() {
	options := &serial.Mode{
		BaudRate: 1000000,
	}
	var err error
	t.Port, err = serial.Open("/dev/ttyAMA0", options)
	if err != nil {
		panic(err)
	}

	t.Joystick = NewJoystick()

	t.Running = true
	go func() {
		var state Joy
		tick := time.Tick(300 * time.Millisecond)
		for t.Running {
			select {
			case state = <-t.Joystick.Joy:
			case <-tick:
			}
			leftSpeed, rightSpeed := 0.0, 0.0
			if state.Mode == ModeAuto {
				switch t.Action {
				case ActionForward:
					leftSpeed = -50
					rightSpeed = -50
				case ActionBackward:
					leftSpeed = 50
					rightSpeed = 50
				case ActionLeft:
					leftSpeed = 50
					rightSpeed = -50
				case ActionRight:
					leftSpeed = -50
					rightSpeed = 50
				case ActionLightOn:
				case ActionLightOff:
				case ActionNone:
					leftSpeed = 0.0
					rightSpeed = 0.0
				}
			} else {
				switch state.JoystickLeft {
				case JoystickStateUp:
					leftSpeed = -50
				case JoystickStateDown:
					leftSpeed = 50
				case JoystickStateNone:
					leftSpeed = 0.0
				}
				switch state.JoystickRight {
				case JoystickStateUp:
					rightSpeed = 50
				case JoystickStateDown:
					rightSpeed = -50
				case JoystickStateNone:
					rightSpeed = 0.0
				}
			}

			{
				message := GeneratePacketMotorDuty([4]int32{
					int32(rightSpeed), int32(rightSpeed),
					int32(leftSpeed), int32(leftSpeed)})
				_, err = t.Port.Write(message)
				if err != nil {
					panic(err)
				}
			}
		}
	}()
}

// Do does an action
func (t *Turbopi) Do(action TypeAction) {
	t.Action = action
}

// Done the robot is done
func (t *Turbopi) Done() {
	err := t.Port.Close()
	if err != nil {
		panic(err)
	}
	t.Joystick.Running = false
	t.Running = false
}
