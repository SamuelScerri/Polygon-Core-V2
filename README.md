# What Is Polygon Core?
Polygon Core is a Software Renderer built with speed in mind and is designed to run in real-time, it utilizes Goroutines to speed up performance significantly and to allow other things such as game logic to run in the background. Since it utilizes multiple cores to rasterize a triangle, remember that with something such as a Dual-Core CPU or a Quad-Core CPU, you might not get that much of a performance increase.

# Any Experimental Features?
The future iterations of Polygon Core will utilize a Super Resolution Model to increase performance by rendering everything with the CPU using a low resolution, and then upscaling the screen buffer with the GPU. This will of course be optional and it is unsure whether this would even increase performance or not, but that's why it's called experimental!

# Libraries Used:
- Ebiten: For Rendering The Screen Buffer & Keyboard Input
- Math32: For Fast Single Precision Math (Golang Used Double Precision Math Functions In The Default Library)

# Current Features:
- Barycentric Triangle Rasterization
- Perspective Correct Texutre Mapping
- Depth Buffer Support
- Triangle Clipping
- Multicore Rendering

# Features That Will Be Added:
- SDL2 Version
- Triange Culling
- Translation & Rotation Support
- Wavefront OBJ Support
- Super Resolution Upscaling

# Features That MIGHT Be Added:
- Occlusion Culling
- Basic Diffuse Lighting
- Basic Shadow Mapping
- Normal Mapping

# Why Go & Not Something Like Rust Or C++? (Beware, Opinionated Paragraph!)
The original version was written in Python, even after using libraries that do JIT compilations on functions, performance still wasn't the best it could be & so I had to rewrite it using another language. The most ideal choice seemed to be Go, as it is a sort of middle ground between a High-Level and a Low-Level language (for me) and has relatively simple syntax (took me half a week to learn the language) with built in concurrency. Rust seemed too complex and felt like it would take ages to learn, and C++... well C++ is just a mess.
