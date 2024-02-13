package main

type Matrix [][]float32

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
