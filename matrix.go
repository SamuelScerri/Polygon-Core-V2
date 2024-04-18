package ghetty

import (
	"math"
)

type Matrix [][]float32

func ProjectionMatrix() Matrix {
	var tangent float32 = float32(math.Tan((float64(FOV) * (math.Pi / 180)) / 2))

	return Matrix{
		{1 / (tangent * Aspect), 0, 0, 0},
		{0, 1 / tangent, 0, 0},
		{0, 0, (Far + Near) / (Near - Far), (Near * Far * 2) / (Near - Far)},
		{0, 0, -1, 0},
	}
}

func TransformationMatrix(p, r Vertex) Matrix {
	return Matrix{
		{1 - 2*r[Y]*r[Y] - 2*r[Z]*r[Z], 2*r[X]*r[Y] + 2*r[Z]*r[W], 2*r[X]*r[Z] - 2*r[Y]*r[W], 0},
		{2*r[X]*r[Y] - 2*r[Z]*r[W], 1 - 2*r[X]*r[X] - 2*r[Z]*r[Z], 2*r[Y]*r[Z] + 2*r[W]*r[X], 0},
		{2*r[X]*r[Z] + 2*r[Y]*r[W], 2*r[Y]*r[Z] - 2*r[W]*r[X], 1 - 2*r[X]*r[X] - 2*r[Y]*r[Y], 0},
		{p[X], p[Y], p[Z], 1},
	}
}

func (m1 *Matrix) Multiply(m2 *Matrix) Matrix {
	var result Matrix = make(Matrix, len(*m1))

	for i := range *m1 {
		result[i] = make([]float32, len((*m2)[0]))

		for j := range (*m2)[0] {
			for k := range *m2 {
				result[i][j] += (*m1)[i][k] * (*m2)[k][j]
			}
		}
	}

	return result
}

func (m1 *Matrix) Vertex() Vertex {
	return (*m1)[0][:]
}
