package main

import (
	"math"
)

type ProcessedTriangle struct {
	Triangle *Triangle

	Bounds, VS1, VS2 Vertex

	Span, Split float32
}

func Clamp(value float32, min, max int) float32 {
	if value < float32(min) {
		return float32(min)
	} else if value > float32(max) {
		return float32(max)
	}

	return value
}

func Process(triangle *Triangle, tiles *([4][3]Tile)) ProcessedTriangle {
	triangle.Normalize()
	triangle.ScreenSpace()
	//copiedTriangle.Sort()

	var split float32 = triangle.Vertices[0][X] + ((triangle.Vertices[1][Y]-triangle.Vertices[0][Y])/
		(triangle.Vertices[2][Y]-triangle.Vertices[0][Y]))*
		(triangle.Vertices[2][X]-triangle.Vertices[0][X])

	var vs1, vs2 Vertex = triangle.Span()

	return ProcessedTriangle{
		triangle,
		triangle.Bounds(), vs1, vs2,
		1 / vs1.CrossProduct(&vs2), split,
	}
}

func (ts *ProcessedTriangle) TileBoundary(tiles *([4][3]Tile)) (int, int, int, int) {
	return int(Clamp(float32(math.Floor(float64(ts.Triangle.Bounds()[XMIN]/float32(TileXSize)))), 0, 4)),
		int(Clamp(float32(math.Floor(float64(ts.Triangle.Bounds()[YMIN]/float32(TileYSize)))), 0, 3)),
		int(Clamp(float32(math.Ceil(float64(ts.Triangle.Bounds()[XMAX]/float32(TileXSize)))), 0, 4)),
		int(Clamp(float32(math.Ceil(float64(ts.Triangle.Bounds()[YMAX]/float32(TileYSize)))), 0, 3))
}

func (ts *ProcessedTriangle) Barycentric(x, y int) (float32, float32, float32) {
	var q Vertex = Vertex{float32(x) - ts.Triangle.Vertices[0][X], float32(y) - ts.Triangle.Vertices[0][Y]}

	var s float32 = q.CrossProduct(&ts.VS2) * ts.Span
	var t float32 = ts.VS1.CrossProduct(&q) * ts.Span

	return s, t, 1 - s - t
}

func (ts *ProcessedTriangle) Inside(x, y int) (bool, float32, float32, float32) {
	var s, t, w float32 = ts.Barycentric(x, y)

	return (s >= 0 && t >= 0 && s+t <= 1),
		s, t, w
}
