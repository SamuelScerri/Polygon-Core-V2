# What is Ghetty?
Ghetty is a Software Renderer designed to run in real-time with excellent performance, whilst also being quite efficient.
Multiple goroutines are utilised to process and rasterize triangles, allowing every core to be used in conjunction with one another, to deliver a better framerate!
Naturally since it is a Software Renderer, it is bound by certain limitations. It can't handle millions of triangles like a modern GPU, and will be less efficient than one.
Ensure that your PC has a proper cooling solution, since all of the CPU cores will be at 100% utilization.

# Current Features
- Triangle Rasterization
- Perspective Correct Texture Mapping
- Depth Buffer Support
- Triangle Clipping
- Multicore Rendering
- Backface Culling
- Matrix Multiplication
- Wavefront OBJ Support
- FSRCNN & LapSRN Upscaling Integration
