package main

import (
	"math"
)

const (
	XMax = 0
	YMax = 1
	XMin = 2
	YMin = 3
)

type Triangle struct {
	UV [3]Vertex

	Vertices [3]Vertex

	Color [3]Vertex

	Normals [3]Vertex

	Texture *Texture

	Shader Shader
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

func (triangle *Triangle) Interpolate(w, s, t float32) (float32, float32, float32) {
	return w*triangle.Color[0][R] + s*triangle.Color[1][R] + t*triangle.Color[2][R],
		w*triangle.Color[0][G] + s*triangle.Color[1][G] + t*triangle.Color[2][G],
		w*triangle.Color[0][B] + s*triangle.Color[1][B] + t*triangle.Color[2][B]
}

func (triangle *Triangle) Bounds() Vertex {
	return Vertex{
		float32(math.Max(float64(triangle.Vertices[0][X]),
			math.Max(float64(triangle.Vertices[1][X]), float64(triangle.Vertices[2][X])))) + 1,

		float32(math.Max(float64(triangle.Vertices[0][Y]),
			math.Max(float64(triangle.Vertices[1][Y]), float64(triangle.Vertices[2][Y])))) + 1,

		float32(math.Min(float64(triangle.Vertices[0][X]),
			math.Min(float64(triangle.Vertices[1][X]), float64(triangle.Vertices[2][X])))) - 1,

		float32(math.Min(float64(triangle.Vertices[0][Y]),
			math.Min(float64(triangle.Vertices[1][Y]), float64(triangle.Vertices[2][Y])))) - 1,
	}
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

func (triangle *Triangle) Copy() (copiedTriangle Triangle) {
	for index := range triangle.Vertices {
		copiedTriangle.UV[index] = triangle.UV[index].Copy()
		copiedTriangle.Vertices[index] = triangle.Vertices[index].Copy()
		copiedTriangle.Color[index] = triangle.Color[index].Copy()
		copiedTriangle.Normals[index] = triangle.Normals[index].Copy()
		copiedTriangle.Texture = triangle.Texture
	}

	copiedTriangle.Shader = triangle.Shader

	return
}
