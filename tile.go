package main

import (
	"math"
	"sync"
)

type Tile struct {
	Frame []byte
	Depth []float32

	Triangles []*ProcessedTriangle

	X, Y int
}

var WaitGroup sync.WaitGroup
var Mutex sync.Mutex

func (tile *Tile) Barycentric(triangle *ProcessedTriangle) {
	var xMin, yMin, xMax, yMax int = int(Clamp(triangle.Bounds[XMin], tile.X, tile.X+TileXSize)),
		int(Clamp(triangle.Bounds[YMin], tile.Y, tile.Y+TileYSize)),
		int(Clamp(triangle.Bounds[XMax], tile.X, tile.X+TileXSize)),
		int(Clamp(triangle.Bounds[YMax], tile.Y, tile.Y+TileYSize))

	for y := yMin; y < yMax; y++ {
		for x := xMin; x < xMax; x++ {
			if inside, w, s, t := triangle.Inside(x, y); inside {
				var depth float32 = w*triangle.Triangle.Vertices[0][Z] + s*triangle.Triangle.Vertices[1][Z] + t*triangle.Triangle.Vertices[2][Z]

				if position := tile.ConvertPosition(x, y); depth < tile.Depth[position] {
					var wt float32 = 1 / (w*triangle.Triangle.UV[0][Z] + s*triangle.Triangle.UV[1][Z] + t*triangle.Triangle.UV[2][Z])

					var uv Vertex = Vertex{
						(w*triangle.Triangle.UV[0][X] + s*triangle.Triangle.UV[1][X] + t*triangle.Triangle.UV[2][X]) * wt,
						(w*triangle.Triangle.UV[0][Y] + s*triangle.Triangle.UV[1][Y] + t*triangle.Triangle.UV[2][Y]) * wt,
					}

					var r, g, b float32 = triangle.Triangle.Interpolate(w, s, t)
					triangle.Triangle.Shader.Fragment(&r, &g, &b, &uv, &Brick)

					tile.Set(position, byte(r*255), byte(g*255), byte(b*255), depth)
				}
			}
		}
	}
}

func (tile *Tile) Rasterize() {
	for index := range tile.Triangles {
		tile.Barycentric(tile.Triangles[index])
	}

	tile.Reset()
}

func (tile *Tile) Reset() {
	tile.Triangles = tile.Triangles[:0]
}

func (tile *Tile) Add(triangle *ProcessedTriangle) {
	tile.Triangles = append(tile.Triangles, triangle)
}

func (tile *Tile) ConvertPosition(x, y int) int {
	return y*Width + x
}

func (tile *Tile) Set(position int, r, g, b byte, depth float32) {
	var colorPosition int = position * 4

	tile.Depth[position] = depth
	tile.Frame[colorPosition+R], tile.Frame[colorPosition+G], tile.Frame[colorPosition+B] = r, g, b
}

func (tile *Tile) Clear(r, g, b byte) {
	for y := tile.Y; y < tile.Y+TileYSize; y++ {
		for x := tile.X; x < tile.X+TileXSize; x++ {
			tile.Set(tile.ConvertPosition(x, y),
				r, g, b, math.MaxFloat32)
		}
	}
}
