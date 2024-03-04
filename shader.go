package main

type VertexShader func(vertex, uv, normal *Vertex, matrices ...*Matrix)
type FragmentShader func(r, g, b *float32)

type Shader struct {
	Vertex VertexShader

	Fragment FragmentShader
}