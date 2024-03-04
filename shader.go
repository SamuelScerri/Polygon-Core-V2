package main

type VertexShader func(vertex, uv, normal, color *Vertex, matrices ...*Matrix)
type FragmentShader func(r, g, b *float32, uv *Vertex, textures ...*Texture)

type Shader struct {
	Vertex VertexShader

	Fragment FragmentShader
}