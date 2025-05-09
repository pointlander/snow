// Copyright 2025 The Snow Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"testing"
)

func TestMotorPacket(t *testing.T) {
	correct := []byte{170, 85, 3, 22, 5, 4, 0, 0, 0, 72, 194, 1, 0, 0, 72, 66, 2, 0, 0, 72, 66, 3, 0, 0, 72, 194, 232}
	packet := GeneratePacketMotorDuty([4]int32{-50, 50, 50, -50})
	for i, v := range correct {
		if packet[i] != v {
			t.Fatalf("%v != %v", correct, packet)
		}
	}
}

func testPWMPacket(t *testing.T) {
	correct := []byte{170, 85, 4, 10, 1, 232, 3, 2, 1, 108, 7, 2, 232, 3, 237}
	packet := GeneratePacketPWMServoSetPosition(1, [2]uint16{1900, 1000})
	for i, v := range correct {
		if packet[i] != v {
			t.Fatalf("%v != %v", correct, packet)
		}
	}

}
