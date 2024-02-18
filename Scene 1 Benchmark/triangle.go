package main

import "math"

type Triangle struct {
	UV [3]Vertex

	Vertices [3]Vertex

	Normals [3]Vertex
}

type TriangleSpan struct {
	VS1, VS2 Vertex

	Span float32
}

func (triangle *Triangle) Transform(m2 *Matrix) {
	for index := range triangle.Vertices {
		var matrix Matrix = triangle.Vertices[index].Matrix()
		matrix = matrix.Multiply(m2)

		triangle.Vertices[index] = matrix.Vertex()
	}
}

func (triangle *Triangle) ScreenSpace() {
	for index := range triangle.Vertices {
		triangle.Vertices[index].ScreenSpace()
	}
}

func (triangle *Triangle) Normalize() {
	for index := range triangle.Vertices {
		triangle.Vertices[index].Normalize()
	}
}

func (triangle *Triangle) Sort() {
	for i := range triangle.Vertices {
		for j := i + 1; j < len(triangle.Vertices); j++ {
			if triangle.Vertices[i][Y] > triangle.Vertices[j][Y] {

				triangle.Vertices[i].Swap(&triangle.Vertices[j])
				triangle.Normals[i].Swap(&triangle.Normals[j])
				triangle.UV[i].Swap(&triangle.UV[j])
			}
		}
	}
}

func (triangle *Triangle) Bounds() (float32, float32, float32, float32) {
	return float32(math.Max(float64(triangle.Vertices[0][X]),
			math.Max(float64(triangle.Vertices[1][X]), float64(triangle.Vertices[2][X])))),

		float32(math.Max(float64(triangle.Vertices[0][Y]),
			math.Max(float64(triangle.Vertices[1][Y]), float64(triangle.Vertices[2][Y])))),

		float32(math.Min(float64(triangle.Vertices[0][X]),
			math.Min(float64(triangle.Vertices[1][X]), float64(triangle.Vertices[2][X])))),

		float32(math.Min(float64(triangle.Vertices[0][Y]),
			math.Min(float64(triangle.Vertices[1][Y]), float64(triangle.Vertices[2][Y]))))
}

func (triangle *Triangle) EdgeSpan(x, y int) (float32, float32, float32) {
	return (triangle.Vertices[2][Y]-triangle.Vertices[1][Y])*(float32(x)-triangle.Vertices[1][X]) -
			(triangle.Vertices[2][X]-triangle.Vertices[1][X])*(float32(y)-triangle.Vertices[1][Y]),

		(triangle.Vertices[0][Y]-triangle.Vertices[2][Y])*(float32(x)-triangle.Vertices[2][X]) -
			(triangle.Vertices[0][X]-triangle.Vertices[2][X])*(float32(y)-triangle.Vertices[2][Y]),

		(triangle.Vertices[1][Y]-triangle.Vertices[0][Y])*(float32(x)-triangle.Vertices[0][X]) -
			(triangle.Vertices[1][X]-triangle.Vertices[0][X])*(float32(y)-triangle.Vertices[0][Y])
}

func (triangle *Triangle) Span() (Vertex, Vertex) {
	return Vertex{triangle.Vertices[1][X] - triangle.Vertices[0][X], triangle.Vertices[1][Y] - triangle.Vertices[0][Y]},
		Vertex{triangle.Vertices[2][X] - triangle.Vertices[0][X], triangle.Vertices[2][Y] - triangle.Vertices[0][Y]}
}

func (triangle *Triangle) Barycentric(span *TriangleSpan, x, y int) (float32, float32, float32) {
	var q Vertex = Vertex{float32(x) - triangle.Vertices[0][X], float32(y) - triangle.Vertices[0][Y]}

	var s float32 = q.CrossProduct(&span.VS2) * span.Span
	var t float32 = span.VS1.CrossProduct(&q) * span.Span

	return s, t, 1 - s - t
}
