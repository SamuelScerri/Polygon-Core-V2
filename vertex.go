package main

const (
	X = 0
	Y = 1
	Z = 2
	W = 3
)

type Vertex []float32

func (v1 Vertex) Sum(v2 *Vertex) Vertex {
	for index := range v1 {
		v1[index] += (*v2)[index]
	}

	return v1
}

func (v1 Vertex) Multiply(v2 *Vertex) Vertex {
	for index := range v1 {
		v1[index] *= (*v2)[index]
	}

	return v1
}

func (v1 *Vertex) Dot(v2 *Vertex) (result float32) {
	for index := range *v1 {
		result += (*v1)[index] * (*v2)[index]
	}

	return
}

func (v1 *Vertex) Cross(v2 *Vertex) Vertex {
	return Vertex{
		(*v1)[Y]*(*v2)[Z] - (*v1)[Z]*(*v2)[Y],
		(*v1)[Z]*(*v2)[X] - (*v1)[X]*(*v2)[Z],
		(*v1)[X]*(*v2)[Y] - (*v1)[Y]*(*v2)[X],
	}
}

func (v1 Vertex) Interpolate(v2 *Vertex, factor float32) Vertex {
	for index := range v1 {
		v1[index] = v1[index]*(1-factor) + (*v2)[index]*factor
	}

	return v1
}

func (v1 Vertex) Normalize() Vertex {
	var homogeneous float32 = 1 / v1[W]

	v1[X] *= homogeneous
	v1[Y] *= homogeneous
	v1[Z] *= homogeneous

	return v1
}

func (v1 Vertex) Matrix() Matrix {
	return Matrix{v1[:]}
}
