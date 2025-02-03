// Copyright 2025 The Snow Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"fmt"
	"math"
	"math/rand"

	"github.com/alixaxel/pagerank"
)

func main() {
	rng := rand.New(rand.NewSource(1))
	u := NewMatrix(8, 8)
	for i := 0; i < u.Cols*u.Rows; i++ {
		u.Data = append(u.Data, rng.Float32())
	}

	for {
		graph := pagerank.NewGraph()
		for i := 0; i < u.Rows; i++ {
			for j := 0; j < u.Rows; j++ {
				p := CS(u.Data[i*u.Cols:(i+1)*u.Cols], u.Data[j*u.Cols:(j+1)*u.Cols])
				graph.Link(uint32(i), uint32(j), float64(p))
			}
		}
		ranks := make([]float64, u.Rows)
		graph.Rank(1.0, 1e-3, func(node uint32, rank float64) {
			ranks[node] = rank
		})
		min, index := math.MaxFloat64, 0
		for i, v := range ranks {
			if v < min {
				min, index = v, i
			}
		}
		for i, v := range ranks {
			u.Data[index*u.Cols+i] = float32(v)
		}
		for i := 0; i < u.Rows; i++ {
			for j := 0; j < u.Cols; j++ {
				fmt.Printf("%f ", u.Data[i*u.Cols+j])
			}
			fmt.Println()
		}
		fmt.Println()
	}
}
