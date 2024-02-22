package main

import (
	"sync"
)

const (
	R = 2
	G = 1
	B = 0
)

const (
	BarycentricAlgorithm = 0

	EdgeTestAlgorithm = 1

	SweepLineAlgorithm = 2
)

type Shader func(s, t, w float32) (float32, float32, float32)

type Tile struct {
	Frame []byte

	Triangles []ProcessedTriangle

	X, Y int
}

var AlgorithmUsed int = SweepLineAlgorithm

var Pitch int
var BytesPerPixel int
var WaitGroup sync.WaitGroup

func (tile *Tile) Barycentric(triangle *ProcessedTriangle) {
	for y := tile.Y; y < tile.Y+TileYSize-1; y++ {
		for x := tile.X; x < tile.X+TileXSize-1; x++ {

			if inside, s, t, w := triangle.Inside(x, y); inside {
				var r, g, b float32 = triangle.Triangle.Shader(s, t, w)
				tile.Set(x, y, byte(r*255), byte(g*255), byte(b*255))
			}
		}
	}
}

func (tile *Tile) Rasterize(shader Shader) {
	for index := range tile.Triangles {
		//fmt.Println(tile.Triangles[index].Triangle)

		tile.Barycentric(&tile.Triangles[index])
	}

	tile.Triangles = nil
}

func (tile *Tile) Add(triangle *ProcessedTriangle) {
	tile.Triangles = append(tile.Triangles, *triangle)
}

func (tile *Tile) Set(x, y int, r, g, b byte) {
	var position int = y*Pitch + x*BytesPerPixel

	tile.Frame[position+B] = b
	tile.Frame[position+G] = g
	tile.Frame[position+R] = r
}

func (tile *Tile) Clear(r, g, b byte) {
	for y := tile.Y; y < tile.Y+TileYSize; y++ {
		for x := tile.X; x < tile.X+TileXSize; x++ {
			tile.Set(x, y, r, g, b)
		}
	}
}
