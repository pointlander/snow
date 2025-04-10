// Copyright 2025 The Snow Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	htgotts "github.com/hegedustibor/htgo-tts"
	handlers "github.com/hegedustibor/htgo-tts/handlers"
	voices "github.com/hegedustibor/htgo-tts/voices"
)

// Urobot is a human robot
type Urobot struct {
	Action TypeAction
	Say    chan string
}

// Init initializes the robot
func (u *Urobot) Init() {
	speech := htgotts.Speech{Folder: "audio", Language: voices.English, Handler: &handlers.MPlayer{}}
	speech.Speak("Starting...")
	say := make(chan string, 8)
	go func() {
		for s := range say {
			speech.Speak(s)
		}
	}()
	u.Say = say
}

// Do does an action
func (u *Urobot) Do(action TypeAction) {
	switch action {
	case ActionForward:
		u.Say <- "forward"
	case ActionBackward:
		u.Say <- "backward"
	case ActionLeft:
		u.Say <- "left"
	case ActionRight:
		u.Say <- "right"
	case ActionLightOn:
		u.Say <- "light on"
	case ActionLightOff:
		u.Say <- "light off"
	case ActionNone:
		u.Say <- "none"
	}
}

// Done the robot is done
func (u *Urobot) Done() {

}
