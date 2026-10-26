# Coordinate & Mathematical Conventions

All modules, math routines, shaders, and physics dynamics adhere strictly to the following conventions without exception:

## 1. Units of Measure
- **Length**: Meters ($m$)
- **Mass**: Kilograms ($kg$)
- **Time**: Seconds ($s$)
- **Force**: Newtons ($N$)
- **Torque**: Newton-meters ($N \cdot m$)
- **Angles**: Radians ($rad$)
- **Frequency**: Hertz ($Hz$)

Conversions to millimetres, degrees, or microseconds occur exclusively at UI presentation boundaries or wire-format packet serialization.

## 2. Coordinate System
- **Handedness**: Right-Handed System (RHS)
- **Axes**:
	- $+X$: Forward / docking axis
	- $+Y$: Up axis
	- $+Z$: Lateral / Right axis (satisfying $\hat{X} \times \hat{Y} = \hat{Z}$)
- **Winding Order**: Counter-Clockwise (CCW) for front-facing polygons.
- **Backface Culling**: `GL_BACK` enabled.

## 3. Transformations & Matrices
- **Library**: OpenGL Mathematics (GLM)
- **Matrix Storage**: Column-major order (OpenGL & GLM standard).
- **Multiplication Order**: Vector post-multiplication ($v' = M \cdot v = P \cdot V \cdot M \cdot v$).
- **Rotations & Orientations**:
	- Represented by unit quaternions ($w, x, y, z$).
	- Unit norm maintained via normalization after numerical integration.

## 4. Floating-Point Comparisons
- Direct equality checks on floating-point numbers (`==` or `!=`) are forbidden.
- Use `space_docking::utils::nearlyEqual(a, b, epsilon)` with documented epsilon values (typically `1e-6` for coordinates, `1e-9` for unit quaternions).
