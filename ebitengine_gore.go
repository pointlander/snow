package main

import (
	"image"
	"image/color"
	"math"
	"math/rand"
	"sync"

	"github.com/AndreRenaud/gore"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

const (
	screenWidth  = 640
	screenHeight = 480
)

const actions = 5

type Auto struct {
	Auto   *AutoEncoder
	Action TypeAction
}

type DoomGame struct {
	lastFrame *ebiten.Image

	events      []gore.DoomEvent
	lock        sync.Mutex
	terminating bool

	rng       *rand.Rand
	auto      [actions]Auto
	votes     [actions]uint
	last      TypeAction
	iteration int
}

func (g *DoomGame) Update() error {
	keys := map[ebiten.Key]uint8{
		ebiten.KeySpace:     gore.KEY_USE1,
		ebiten.KeyEscape:    gore.KEY_ESCAPE,
		ebiten.KeyUp:        gore.KEY_UPARROW1,
		ebiten.KeyDown:      gore.KEY_DOWNARROW1,
		ebiten.KeyLeft:      gore.KEY_LEFTARROW1,
		ebiten.KeyRight:     gore.KEY_RIGHTARROW1,
		ebiten.KeyEnter:     gore.KEY_ENTER,
		ebiten.KeyControl:   gore.KEY_FIRE1,
		ebiten.KeyShift:     0x80 + 0x36,
		ebiten.KeyBackspace: gore.KEY_BACKSPACE3,
		ebiten.KeyY:         'y',
		ebiten.KeyN:         'n',
		ebiten.KeyI:         'i',
		ebiten.KeyD:         'd',
		ebiten.KeyF:         'f',
		ebiten.KeyA:         'a',
		ebiten.KeyE:         'e',
		ebiten.KeyR:         'r',
		ebiten.KeyV:         'v',
		ebiten.KeyC:         'c',
		ebiten.KeyL:         'l',
		ebiten.KeyQ:         'q',
		ebiten.Key1:         '1',
		ebiten.Key2:         '2',
		ebiten.Key3:         '3',
		ebiten.Key4:         '4',
		ebiten.Key5:         '5',
		ebiten.Key6:         '6',
		ebiten.Key7:         '7',
		ebiten.Key8:         '8',
		ebiten.Key9:         '9',
		ebiten.Key0:         '0',
	}
	g.lock.Lock()
	defer g.lock.Unlock()
	for key, doomKey := range keys {
		if inpututil.IsKeyJustPressed(key) {
			var event gore.DoomEvent

			event.Type = gore.Ev_keydown
			event.Key = doomKey
			g.events = append(g.events, event)
		} else if inpututil.IsKeyJustReleased(key) {
			var event gore.DoomEvent
			event.Type = gore.Ev_keyup
			event.Key = doomKey
			g.events = append(g.events, event)
		}

		var mouseEvent gore.DoomEvent
		x, y := ebiten.CursorPosition()
		mouseEvent.Mouse.XPos = float64(x) / float64(screenWidth)
		mouseEvent.Mouse.YPos = float64(y) / float64(screenHeight)
		mouseEvent.Type = gore.Ev_mouse
		if ebiten.IsMouseButtonPressed(ebiten.MouseButtonLeft) {
			mouseEvent.Mouse.Button1 = true
		}
		if ebiten.IsMouseButtonPressed(ebiten.MouseButtonRight) {
			mouseEvent.Mouse.Button2 = true
		}
		g.events = append(g.events, mouseEvent)
	}
	if g.terminating {
		return ebiten.Termination
	}
	return nil
}

func (g *DoomGame) Draw(screen *ebiten.Image) {
	g.lock.Lock()
	defer g.lock.Unlock()

	if g.lastFrame == nil {
		return
	}
	op := &ebiten.DrawImageOptions{}
	rect := g.lastFrame.Bounds()
	yScale := float64(screenHeight) / float64(rect.Dy())
	xScale := float64(screenWidth) / float64(rect.Dx())
	op.GeoM.Scale(xScale, yScale)
	screen.DrawImage(g.lastFrame, op)
}

func (g *DoomGame) Layout(outsideWidth, outsideHeight int) (int, int) {
	return screenWidth, screenHeight
}

func (g *DoomGame) GetEvent(event *gore.DoomEvent) bool {
	g.lock.Lock()
	defer g.lock.Unlock()
	if len(g.events) > 0 {
		*event = g.events[0]
		g.events = g.events[1:]
		return true
	}
	return false
}

func (g *DoomGame) DrawFrame(frame *image.RGBA) {
	g.lock.Lock()
	defer g.lock.Unlock()

	width := frame.Bounds().Max.X
	height := frame.Bounds().Max.Y
	var l [actions]float32
	var p [128][]float32
	for i := range p {
		x := g.rng.Intn(width - 8)
		y := g.rng.Intn(height - 8)
		pixels := make([]float32, 8*8)
		for yy := 0; yy < 8; yy++ {
			for xx := 0; xx < 8; xx++ {
				pixel := color.GrayModel.Convert(frame.At(x+xx, y+yy)).(color.Gray)
				pixels[yy*8+xx] = float32(pixel.Y)
			}
		}
		p[i] = pixels
	}
	for i := range g.auto {
		l[i] = g.auto[i].Auto.Measure(&p)
	}
	min, max, index, index1 := float32(math.MaxFloat32), float32(0.0), 0, 0
	for ii, value := range l {
		if value > max {
			max, index = value, ii
		}
		if value < min {
			min, index1 = value, ii
		}
	}
	g.votes[index1]++
	if g.iteration%30 == 0 {
		max, index := uint(0), 0
		for ii, value := range g.votes {
			if value > max {
				max, index = value, ii
			}
			g.votes[ii] = 0
		}
		if g.last != ActionCount {
			var event gore.DoomEvent
			event.Type = gore.Ev_keyup
			switch g.last {
			case ActionLeft:
				event.Key = gore.KEY_LEFTARROW1
			case ActionRight:
				event.Key = gore.KEY_RIGHTARROW1
			case ActionForward:
				event.Key = gore.KEY_UPARROW1
			case ActionBackward:
				event.Key = gore.KEY_DOWNARROW1
			case ActionNone:
			}
			g.events = append(g.events, event)
		}
		{
			var event gore.DoomEvent
			event.Type = gore.Ev_keydown
			switch g.auto[index].Action {
			case ActionLeft:
				event.Key = gore.KEY_LEFTARROW1
			case ActionRight:
				event.Key = gore.KEY_RIGHTARROW1
			case ActionForward:
				event.Key = gore.KEY_UPARROW1
			case ActionBackward:
				event.Key = gore.KEY_DOWNARROW1
			case ActionNone:
			}
			g.events = append(g.events, event)
		}
		g.last = g.auto[index].Action
	}
	g.auto[index].Auto.Encode(&p)
	g.iteration++

	if g.lastFrame != nil {
		if g.lastFrame.Bounds().Dx() != frame.Bounds().Dx() || g.lastFrame.Bounds().Dy() != frame.Bounds().Dy() {
			g.lastFrame.Deallocate()
			g.lastFrame = nil
		}
	}
	if g.lastFrame == nil {
		g.lastFrame = ebiten.NewImage(frame.Bounds().Dx(), frame.Bounds().Dy())
	}
	g.lastFrame.WritePixels(frame.Pix)
}

func (g *DoomGame) SetTitle(title string) {
	ebiten.SetWindowTitle(title)
}
