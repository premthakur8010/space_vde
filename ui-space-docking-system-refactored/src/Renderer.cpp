// =============================================================================
// src/Renderer.cpp
// Implements the OpenGL 4.5 3D rendering classes: two-light GLSL 450 shader,
// Direct State Access (DSA) 4x MSAA Framebuffer, Wavefront OBJ Mesh loader,
// OrbitCamera math, immediate-mode Painter, RGB coordinate triads, VdeScene
// (hexapod + overhead gantry), and SpacecraftScene (orbital Chaser & Target).
// =============================================================================
#include "Renderer.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <system_error>
#include <unordered_map>

namespace vde::render {

namespace {
constexpr GLsizei kSamples          = 4;
constexpr float   kRotateDegPerPx   = 0.3f;
constexpr float   kMaxPitchDeg      = 89.0f;
constexpr float   kZoomPerWheelStep = 0.88f;
constexpr float   kMinDistanceM     = 0.5f;
constexpr float   kMaxDistanceM     = 400.0f;

// Resolves a Wavefront .obj asset path next to the executable first, then source/repo locations.
std::string resolveModelPath(const char* file) {
	namespace fs = std::filesystem;
	std::error_code ec;
	const std::string candidates[] = {
		vde::executableDir() + "/models/" + file,
		std::string("assets/models/") + file,
		std::string("ui-space-docking-system-refactored/assets/models/") + file,
#ifdef VDE_MODEL_DIR
		std::string(VDE_MODEL_DIR) + "/" + file,
#endif
	};
	for (const std::string& path : candidates) {
		if (fs::is_regular_file(path, ec)) return path;
	}
	return std::string("assets/models/") + file;
}

// Compiles a single GLSL shader stage and prints driver diagnostics on syntax/compile errors.
GLuint compileShader(GLenum type, const char* src) {
	GLuint s = glCreateShader(type);
	glShaderSource(s, 1, &src, nullptr);
	glCompileShader(s);
	GLint ok = 0;
	glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
	if (!ok) {
		char infoLog[1024];
		glGetShaderInfoLog(s, sizeof(infoLog), nullptr, infoLog);
		std::fprintf(stderr, "[SHADER] %s\n", infoLog);
	}
	return s;
}

// Appends one interleaved 9-float vertex (px, py, pz, nx, ny, nz, r, g, b) into a mesh buffer.
void pushVertex(std::vector<float>& v, glm::vec3 p, glm::vec3 n, glm::vec3 c) {
	v.insert(v.end(), {p.x, p.y, p.z, n.x, n.y, n.z, c.r, c.g, c.b});
}

// Vertex shader transforming local positions, normals, and material colors into world and clip space.
const char* kVertex = R"(#version 450 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec3 a_color;
uniform mat4 u_model;
uniform mat4 u_viewProjection;
out vec3 v_normal;
out vec3 v_world;
out vec3 v_color;
void main() {
	vec4 world = u_model * vec4(a_position, 1.0);
	v_world = world.xyz;
	v_normal = mat3(transpose(inverse(u_model))) * a_normal;
	v_color = a_color;
	gl_Position = u_viewProjection * world;
}
)";

// Fragment shader applying ambient + key + fill directional lighting with two-sided normal flipping.
const char* kFragment = R"(#version 450 core
in vec3 v_normal;
in vec3 v_world;
in vec3 v_color;
uniform vec3 u_color;
uniform vec3 u_cameraPos;
out vec4 fragColor;
void main() {
	vec3 n = normalize(v_normal);
	if (dot(n, u_cameraPos - v_world) < 0.0) n = -n;
	float key  = max(dot(n, normalize(vec3(0.4, -0.6, 0.8))), 0.0);
	float fill = max(dot(n, normalize(vec3(-0.6, 0.5, 0.3))), 0.0);
	vec3 c = (v_color * u_color) * (0.38 + 0.55 * key + 0.22 * fill);
	fragColor = vec4(min(c, vec3(1.0)), 1.0);
}
)";
} // namespace

// ============================================================================
// Shader, Framebuffer & Mesh
// ============================================================================

Shader::~Shader() { if (m_program) glDeleteProgram(m_program); }

bool Shader::build(const char* vertexSrc, const char* fragmentSrc) {
	GLuint vs = compileShader(GL_VERTEX_SHADER, vertexSrc);
	GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragmentSrc);
	m_program = glCreateProgram();
	glAttachShader(m_program, vs);
	glAttachShader(m_program, fs);
	glLinkProgram(m_program);
	glDeleteShader(vs);
	glDeleteShader(fs);
	GLint ok = 0;
	glGetProgramiv(m_program, GL_LINK_STATUS, &ok);
	return ok != 0;
}

void Shader::set(const char* name, const glm::mat4& m) const {
	glProgramUniformMatrix4fv(m_program, glGetUniformLocation(m_program, name), 1, GL_FALSE, glm::value_ptr(m));
}
void Shader::set(const char* name, const glm::vec3& v) const {
	glProgramUniform3fv(m_program, glGetUniformLocation(m_program, name), 1, glm::value_ptr(v));
}
void Shader::set(const char* name, float f) const {
	glProgramUniform1f(m_program, glGetUniformLocation(m_program, name), f);
}

Framebuffer::~Framebuffer() { release(); }

void Framebuffer::release() {
	if (m_msaaFbo) glDeleteFramebuffers(1, &m_msaaFbo);
	if (m_resolveFbo) glDeleteFramebuffers(1, &m_resolveFbo);
	if (m_msaaColor) glDeleteRenderbuffers(1, &m_msaaColor);
	if (m_msaaDepth) glDeleteRenderbuffers(1, &m_msaaDepth);
	if (m_texture) glDeleteTextures(1, &m_texture);
	m_msaaFbo = m_resolveFbo = m_msaaColor = m_msaaDepth = m_texture = 0;
}

// Allocates (or reallocates on viewport resize) 4x MSAA color/depth renderbuffers and a single-sample resolve texture.
void Framebuffer::resize(int width, int height) {
	width  = std::max(width, 1);
	height = std::max(height, 1);
	if (width == m_width && height == m_height && m_msaaFbo) return;
	release();
	m_width = width; m_height = height;

	glCreateRenderbuffers(1, &m_msaaColor);
	glNamedRenderbufferStorageMultisample(m_msaaColor, kSamples, GL_RGBA8, width, height);
	glCreateRenderbuffers(1, &m_msaaDepth);
	glNamedRenderbufferStorageMultisample(m_msaaDepth, kSamples, GL_DEPTH24_STENCIL8, width, height);

	glCreateFramebuffers(1, &m_msaaFbo);
	glNamedFramebufferRenderbuffer(m_msaaFbo, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, m_msaaColor);
	glNamedFramebufferRenderbuffer(m_msaaFbo, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, m_msaaDepth);

	glCreateTextures(GL_TEXTURE_2D, 1, &m_texture);
	glTextureStorage2D(m_texture, 1, GL_RGBA8, width, height);
	glTextureParameteri(m_texture, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTextureParameteri(m_texture, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTextureParameteri(m_texture, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTextureParameteri(m_texture, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glCreateFramebuffers(1, &m_resolveFbo);
	glNamedFramebufferTexture(m_resolveFbo, GL_COLOR_ATTACHMENT0, m_texture, 0);
}

void Framebuffer::bind() const {
	glBindFramebuffer(GL_FRAMEBUFFER, m_msaaFbo);
	glViewport(0, 0, m_width, m_height);
}

// Downsamples the 4x MSAA color attachment into `m_texture` so Dear ImGui can sample it.
void Framebuffer::resolve() const {
	glBlitNamedFramebuffer(m_msaaFbo, m_resolveFbo, 0, 0, m_width, m_height, 0, 0, m_width, m_height,
						   GL_COLOR_BUFFER_BIT, GL_NEAREST);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Framebuffer::readPixels(unsigned char* rgba) const {
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glGetTextureImage(m_texture, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_width * m_height * 4, rgba);
}

Mesh::~Mesh() {
	if (m_vbo) glDeleteBuffers(1, &m_vbo);
	if (m_vao) glDeleteVertexArrays(1, &m_vao);
}

// Configures DSA vertex attributes (location 0 = position, location 1 = normal, location 2 = color) and uploads triangle data.
void Mesh::upload(const std::vector<float>& data) {
	if (!m_vao) {
		glCreateVertexArrays(1, &m_vao);
		glCreateBuffers(1, &m_vbo);
		glVertexArrayVertexBuffer(m_vao, 0, m_vbo, 0, 9 * sizeof(float));
		glEnableVertexArrayAttrib(m_vao, 0);
		glVertexArrayAttribFormat(m_vao, 0, 3, GL_FLOAT, GL_FALSE, 0);
		glVertexArrayAttribBinding(m_vao, 0, 0);
		glEnableVertexArrayAttrib(m_vao, 1);
		glVertexArrayAttribFormat(m_vao, 1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float));
		glVertexArrayAttribBinding(m_vao, 1, 0);
		glEnableVertexArrayAttrib(m_vao, 2);
		glVertexArrayAttribFormat(m_vao, 2, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float));
		glVertexArrayAttribBinding(m_vao, 2, 0);
	}
	glNamedBufferData(m_vbo, (GLsizeiptr)(data.size() * sizeof(float)), data.data(), GL_STATIC_DRAW);
	m_count = (GLsizei)(data.size() / 9);
}

void Mesh::draw() const {
	if (!m_vao || m_count <= 0) return;
	glBindVertexArray(m_vao);
	glDrawArrays(GL_TRIANGLES, 0, m_count);
	glBindVertexArray(0);
}

namespace {
// Loads diffuse Kd colors keyed by material name from a Wavefront .mtl file.
void loadMtlFile(const std::filesystem::path& mtlPath,
                 std::unordered_map<std::string, glm::vec3>& materials) {
	std::ifstream file(mtlPath);
	if (!file.is_open()) return;
	std::string line, currentMtl;
	while (std::getline(file, line)) {
		const size_t first = line.find_first_not_of(" \t\r\n");
		if (first == std::string::npos || line[first] == '#') continue;
		const size_t last = line.find_last_not_of(" \t\r\n");
		const std::string trimmed = line.substr(first, last - first + 1);
		if (trimmed.rfind("newmtl ", 0) == 0 || trimmed.rfind("newmtl\t", 0) == 0) {
			const size_t nameStart = trimmed.find_first_not_of(" \t", 6);
			currentMtl = (nameStart != std::string::npos) ? trimmed.substr(nameStart) : "";
		} else if (!currentMtl.empty() && (trimmed.rfind("Kd ", 0) == 0 || trimmed.rfind("Kd\t", 0) == 0)) {
			const char* p    = trimmed.c_str() + 2;
			char*       next = nullptr;
			const float r    = std::strtof(p, &next); p = next;
			const float g    = std::strtof(p, &next); p = next;
			const float b    = std::strtof(p, &next);
			materials[currentMtl] = glm::vec3(r, g, b);
		}
	}
}
} // namespace

// Parses a Wavefront .obj/.mtl file (v, vn, vt, mtllib, usemtl, f) and uploads to GPU.
bool Mesh::loadObj(Mesh& m, const std::string& path) {
	std::ifstream file(path, std::ios::binary | std::ios::ate);
	if (!file.is_open()) {
		std::fprintf(stderr, "[OBJ] Failed to open model file: %s\n", path.c_str());
		return false;
	}
	const std::streamsize size = file.tellg();
	if (size <= 0) {
		std::fprintf(stderr, "[OBJ] Empty model file: %s\n", path.c_str());
		return false;
	}
	std::string content(static_cast<size_t>(size), '\0');
	file.seekg(0);
	if (!file.read(content.data(), size)) {
		std::fprintf(stderr, "[OBJ] Failed to read model file: %s\n", path.c_str());
		return false;
	}

	const std::filesystem::path objDir = std::filesystem::path(path).parent_path();
	std::unordered_map<std::string, glm::vec3> materials;
	glm::vec3 currentColor(1.0f, 1.0f, 1.0f);

	std::vector<glm::vec3> positions;
	std::vector<glm::vec3> normals;
	std::vector<float>     vertices;

	struct FaceCorner {
		int vIdx  = -1;
		int vnIdx = -1;
	};
	std::vector<FaceCorner> poly;

	auto resolveIndex = [](int raw, int count) -> int {
		if (raw > 0) return raw - 1;
		if (raw < 0) return count + raw;
		return -1;
	};

	const char* cur = content.data();
	const char* end = cur + content.size();

	while (cur < end) {
		while (cur < end && (*cur == ' ' || *cur == '\t' || *cur == '\r' || *cur == '\n')) ++cur;
		if (cur >= end) break;

		const char* lineEnd = cur;
		while (lineEnd < end && *lineEnd != '\n' && *lineEnd != '\r') ++lineEnd;

		const size_t lineLen = static_cast<size_t>(lineEnd - cur);
		if (lineLen > 7 && std::strncmp(cur, "mtllib", 6) == 0 && (cur[6] == ' ' || cur[6] == '\t')) {
			const char* p = cur + 7;
			while (p < lineEnd && (*p == ' ' || *p == '\t')) ++p;
			const char* q = lineEnd;
			while (q > p && (q[-1] == ' ' || q[-1] == '\t')) --q;
			if (q > p) loadMtlFile(objDir / std::string(p, q), materials);
		} else if (lineLen > 7 && std::strncmp(cur, "usemtl", 6) == 0 && (cur[6] == ' ' || cur[6] == '\t')) {
			const char* p = cur + 7;
			while (p < lineEnd && (*p == ' ' || *p == '\t')) ++p;
			const char* q = lineEnd;
			while (q > p && (q[-1] == ' ' || q[-1] == '\t')) --q;
			const auto it = materials.find(std::string(p, q));
			currentColor = (it != materials.end()) ? it->second : glm::vec3(1.0f);
		} else if (cur[0] == 'v' && (cur + 1 < lineEnd) && (cur[1] == ' ' || cur[1] == '\t')) {
			const char* p    = cur + 2;
			char*       next = nullptr;
			const float x    = std::strtof(p, &next); p = next;
			const float y    = std::strtof(p, &next); p = next;
			const float z    = std::strtof(p, &next);
			positions.emplace_back(x, y, z);
		} else if (cur[0] == 'v' && (cur + 2 < lineEnd) && cur[1] == 'n' && (cur[2] == ' ' || cur[2] == '\t')) {
			const char* p    = cur + 3;
			char*       next = nullptr;
			const float nx   = std::strtof(p, &next); p = next;
			const float ny   = std::strtof(p, &next); p = next;
			const float nz   = std::strtof(p, &next);
			normals.emplace_back(nx, ny, nz);
		} else if (cur[0] == 'f' && (cur + 1 < lineEnd) && (cur[1] == ' ' || cur[1] == '\t')) {
			poly.clear();
			const char* p = cur + 2;
			while (p < lineEnd) {
				while (p < lineEnd && (*p == ' ' || *p == '\t')) ++p;
				if (p >= lineEnd) break;

				char*      next = nullptr;
				const long vRaw = std::strtol(p, &next, 10);
				if (next == p) break;
				p = next;

				long vnRaw = 0;
				if (p < lineEnd && *p == '/') {
					++p;
					if (p < lineEnd && *p != '/') {
						std::strtol(p, &next, 10);
						p = next;
					}
					if (p < lineEnd && *p == '/') {
						++p;
						vnRaw = std::strtol(p, &next, 10);
						p = next;
					}
				}

				const int vIdx  = resolveIndex(static_cast<int>(vRaw), static_cast<int>(positions.size()));
				const int vnIdx = resolveIndex(static_cast<int>(vnRaw), static_cast<int>(normals.size()));
				if (vIdx >= 0 && vIdx < static_cast<int>(positions.size())) {
					poly.push_back({vIdx, vnIdx});
				}
				while (p < lineEnd && *p != ' ' && *p != '\t') ++p;
			}

			for (size_t i = 2; i < poly.size(); ++i) {
				const FaceCorner c[3] = {poly[0], poly[i - 1], poly[i]};
				const glm::vec3  p0   = positions[c[0].vIdx];
				const glm::vec3  p1   = positions[c[1].vIdx];
				const glm::vec3  p2   = positions[c[2].vIdx];

				glm::vec3 faceN(0.0f, 0.0f, 1.0f);
				const bool hasNormals =
					(c[0].vnIdx >= 0 && c[0].vnIdx < static_cast<int>(normals.size())) &&
					(c[1].vnIdx >= 0 && c[1].vnIdx < static_cast<int>(normals.size())) &&
					(c[2].vnIdx >= 0 && c[2].vnIdx < static_cast<int>(normals.size()));

				if (!hasNormals) {
					const glm::vec3 cr = glm::cross(p1 - p0, p2 - p0);
					const float     ln = glm::length(cr);
					if (ln > 1e-8f) faceN = cr / ln;
				}

				for (int k = 0; k < 3; ++k) {
					const glm::vec3 pos = positions[c[k].vIdx];
					const glm::vec3 nrm = hasNormals ? normals[c[k].vnIdx] : faceN;
					pushVertex(vertices, pos, nrm, currentColor);
				}
			}
		}

		cur = lineEnd;
	}

	if (vertices.empty()) {
		std::fprintf(stderr, "[OBJ] No geometry loaded from model file: %s\n", path.c_str());
		return false;
	}
	m.upload(vertices);
	return true;
}

// ============================================================================
// OrbitCamera, Painter & Axis Triad
// ============================================================================

glm::vec3 OrbitCamera::position() const {
	const float yaw = glm::radians(yawDeg), pitch = glm::radians(pitchDeg);
	return target + glm::vec3(std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)) * distance;
}
glm::mat4 OrbitCamera::view() const { return glm::lookAt(position(), target, glm::vec3(0, 0, 1)); }
glm::mat4 OrbitCamera::projection(float aspect) const {
	return glm::perspective(glm::radians(fovDeg), std::max(aspect, 0.01f), distance * 0.01f, distance * 100.0f + 500.0f);
}
void OrbitCamera::rotate(float dx, float dy) {
	yawDeg -= dx * kRotateDegPerPx;
	pitchDeg = std::clamp(pitchDeg + dy * kRotateDegPerPx, -kMaxPitchDeg, kMaxPitchDeg);
}
void OrbitCamera::pan(float dx, float dy, float viewportHeight) {
	const glm::mat4 v = view();
	const glm::vec3 right(v[0][0], v[1][0], v[2][0]), up(v[0][1], v[1][1], v[2][1]);
	const float scale = 2.0f * distance * std::tan(glm::radians(fovDeg) * 0.5f) / std::max(viewportHeight, 1.0f);
	target += (-dx * right + dy * up) * scale;
}
void OrbitCamera::zoom(float wheelSteps) {
	distance = std::clamp(distance * std::pow(kZoomPerWheelStep, wheelSteps), kMinDistanceM, kMaxDistanceM);
}

// Compiles the lighting shader and loads dedicated .obj object models once per SceneView.
Painter::Painter() {
	m_shader.build(kVertex, kFragment);
	Mesh::loadObj(m_chaser, resolveModelPath("chaser.obj"));
	Mesh::loadObj(m_target, resolveModelPath("target.obj"));
	Mesh::loadObj(m_earth, resolveModelPath("earth.obj"));
	Mesh::loadObj(m_vdeBase, resolveModelPath("vde_base.obj"));
	Mesh::loadObj(m_vdeGantry, resolveModelPath("vde_gantry.obj"));
	Mesh::loadObj(m_vdePlatform, resolveModelPath("vde_platform.obj"));
	Mesh::loadObj(m_legSleeve, resolveModelPath("leg_sleeve.obj"));
	Mesh::loadObj(m_legPiston, resolveModelPath("leg_piston.obj"));
	Mesh::loadObj(m_legJoint, resolveModelPath("leg_joint.obj"));
	Mesh::loadObj(m_axisTriad, resolveModelPath("axis_triad.obj"));
}

void Painter::begin(const glm::mat4& view, const glm::mat4& proj, const glm::vec3& cameraPos) {
	m_labels.clear();
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glDisable(GL_CULL_FACE);
	glDisable(GL_BLEND);
	glDisable(GL_SCISSOR_TEST);
	m_shader.use();
	m_shader.set("u_viewProjection", proj * view);
	m_shader.set("u_cameraPos", cameraPos);
}

// Clears the depth buffer so coordinate-frame triads drawn afterwards stay visible in front of solid geometry.
void Painter::overlayPass() { glClear(GL_DEPTH_BUFFER_BIT); }

const Mesh& Painter::meshFor(ModelId id) const {
	switch (id) {
	case ModelId::Chaser:      return m_chaser;
	case ModelId::Target:      return m_target;
	case ModelId::Earth:       return m_earth;
	case ModelId::VdeBase:     return m_vdeBase;
	case ModelId::VdeGantry:   return m_vdeGantry;
	case ModelId::VdePlatform: return m_vdePlatform;
	case ModelId::LegSleeve:   return m_legSleeve;
	case ModelId::LegPiston:   return m_legPiston;
	case ModelId::LegJoint:    return m_legJoint;
	case ModelId::AxisTriad:   return m_axisTriad;
	}
	return m_chaser;
}

void Painter::draw(const Mesh& mesh, const glm::mat4& model, const glm::vec3& tint) {
	m_shader.set("u_model", model);
	m_shader.set("u_color", tint);
	mesh.draw();
}

void Painter::drawModel(ModelId id, const glm::mat4& model, const glm::vec3& tint) {
	draw(meshFor(id), model, tint);
}

// Constructs an orthonormal transform mapping the unit [0, 1] Z axis onto segment a -> b with XY scaled by `radius`.
glm::mat4 Painter::alongSegment(const glm::vec3& a, const glm::vec3& b, float radius) {
	const glm::vec3 d = b - a;
	const float len = glm::length(d);
	const glm::vec3 z = len > 1e-6f ? d / len : glm::vec3(0, 0, 1);
	const glm::vec3 helper = std::abs(z.z) < 0.9f ? glm::vec3(0, 0, 1) : glm::vec3(1, 0, 0);
	const glm::vec3 x = glm::normalize(glm::cross(helper, z)), y = glm::cross(z, x);
	glm::mat4 m(1.0f);
	m[0] = glm::vec4(x * radius, 0); m[1] = glm::vec4(y * radius, 0);
	m[2] = glm::vec4(z * len, 0);    m[3] = glm::vec4(a, 1);
	return m;
}

void Painter::drawSegment(ModelId id, const glm::vec3& a, const glm::vec3& b, float radius, const glm::vec3& tint) {
	draw(meshFor(id), alongSegment(a, b, radius), tint);
}

void Painter::drawJoint(const glm::vec3& center, float radius) {
	draw(m_legJoint, glm::scale(glm::translate(glm::mat4(1.0f), center), glm::vec3(radius)), glm::vec3(1.0f));
}

void Painter::drawTriad(const glm::mat4& frame, float length, const char* name) {
	draw(m_axisTriad, glm::scale(frame, glm::vec3(length)), glm::vec3(1.0f));
	if (name) label(glm::vec3(frame[3]), name, LabelKind::Frame);
}

void Painter::label(const glm::vec3& world, std::string text, LabelKind kind, float side) {
	m_labels.push_back({world, std::move(text), kind, side});
}

// ============================================================================
// VdeScene & SpacecraftScene
// ============================================================================

namespace {
constexpr float kBaseTop = 0.12f, kPlatformNominal = 1.10f, kOverheadZ = 2.89f;
constexpr float kBaseAnchorR = 0.85f, kPlatAnchorR = 0.52f;

// Computes a circular actuator joint anchor point in cylindrical coordinates.
glm::vec3 anchor(float radius, float angleDeg, float z) {
	const float a = glm::radians(angleDeg);
	return {radius * std::cos(a), radius * std::sin(a), z};
}

// Builds a rigid-body 4x4 transform from position and ZYX Roll-Pitch-Yaw angles (radians).
glm::mat4 attitude(const glm::vec3& pos, const Rpy& rpy) {
	glm::mat4 m = glm::translate(glm::mat4(1.0f), pos);
	m = glm::rotate(m, (float)rpy.yaw, {0, 0, 1});
	m = glm::rotate(m, (float)rpy.pitch, {0, 1, 0});
	return glm::rotate(m, (float)rpy.roll, {1, 0, 0});
}
} // namespace

OrbitCamera VdeScene::defaultCamera() const { return {{0.0f, 0.0f, 1.55f}, -62.0f, 16.0f, 7.4f, 40.0f}; }

// Renders the VDE testbed using dedicated .obj models: vde_base, vde_gantry, vde_platform
// driven by `snap.vde`, 6 telescoping leg assemblies, labels, and axis triads.
void VdeScene::render(Painter& p, const SimSnapshot& snap) {
	const VdeState& s = snap.vde;
	const glm::mat4 bSp = glm::translate(glm::mat4(1.0f), {0, 0, kBaseTop});
	const glm::mat4 oSp = glm::translate(glm::mat4(1.0f), {0, 0, kOverheadZ});

	glm::mat4 pSp = glm::translate(bSp, glm::vec3(s.positionM.x, s.positionM.y, s.positionM.z) + glm::vec3(0, 0, kPlatformNominal));
	pSp = glm::rotate(pSp, (float)s.orientationRad.yaw, {0, 0, 1});
	pSp = glm::rotate(pSp, (float)s.orientationRad.pitch, {0, 1, 0});
	pSp = glm::rotate(pSp, (float)s.orientationRad.roll, {1, 0, 0});

	p.drawModel(ModelId::VdeBase);
	p.drawModel(ModelId::VdeGantry);
	p.drawModel(ModelId::VdePlatform, pSp);

	for (int i = 0; i < 6; ++i) {
		const int pair = i / 2;
		const float side = (i % 2 == 0) ? -1.0f : 1.0f;
		const glm::vec3 b = glm::vec3(bSp * glm::vec4(anchor(kBaseAnchorR, 120.0f * pair + side * 14.0f, 0.0f), 1));
		const glm::vec3 t = glm::vec3(pSp * glm::vec4(anchor(kPlatAnchorR, 120.0f * pair + side * 46.0f, -0.06f), 1));
		const glm::vec3 mid = b + (t - b) * 0.55f;
		p.drawSegment(ModelId::LegSleeve, b, mid, 0.050f);
		p.drawSegment(ModelId::LegPiston, mid, t, 0.032f);
		p.drawJoint(b, 0.07f);
		p.drawJoint(t, 0.06f);
	}

	p.label({-0.9f, 0.0f, kBaseTop * 0.5f}, "Base (B_sp)", LabelKind::Object, -1.0f);
	p.label(glm::vec3(pSp * glm::vec4(-0.6f, 0.0f, -0.04f, 1)), "Moving platform (P_sp)", LabelKind::Object, -1.0f);
	p.label({-0.8f, 0.0f, kOverheadZ + 0.08f}, "Overhead platform (O_sp)", LabelKind::Object, -1.0f);
	p.label(glm::vec3(pSp * glm::vec4(0.3f, 0, 0.25f, 1)), "Chaser docking mechanism", LabelKind::Object, 1.0f);
	p.label({0.3f, 0.0f, kOverheadZ - 0.25f}, "Target docking mechanism", LabelKind::Object, 1.0f);

	p.overlayPass();
	p.drawTriad(bSp, 0.6f, "B_sp");
	p.drawTriad(pSp, 0.6f, "P_sp");
	p.drawTriad(oSp, 0.6f, "O_sp");
}

OrbitCamera SpacecraftScene::defaultCamera() const { return {{0.3f, 0.0f, -1.2f}, -76.0f, 10.0f, 12.5f, 40.0f}; }

// Renders the Earth sphere, Chaser and Target spacecraft .obj models, labels, and body/docking triads.
void SpacecraftScene::render(Painter& p, const SimSnapshot& snap) {
	const SpacecraftBundle& sc = snap.spacecraft;

	p.drawModel(ModelId::Earth, glm::translate(glm::mat4(1.0f), {0, 0, -75.0f}));
	p.label({-6.0f, -6.0f, -5.3f}, "Earth", LabelKind::Object, 1.0f);

	// -------------------------------------------------------------------------
	// 1. Spacecraft Attitude: Direct Telemetry from Dynamics Engine
	// -------------------------------------------------------------------------
	const Rpy& chaserAtt = sc.chaser.eciOrientationRad;
	const Rpy& targetAtt = sc.target.eciOrientationRad;

	// -------------------------------------------------------------------------
	// 2. Relative Position & Closing Distance: From 14-Element State Vector Y
	// -------------------------------------------------------------------------
	const glm::vec3 targetSize(3.0f, 2.6f, 2.6f), chaserSize(2.0f, 1.7f, 1.7f);
	const float mech = 0.45f;

	float gap = 0.0f;
	float offsetY = 0.0f;
	float offsetZ = 0.0f;

	const double relX = snap.relative.pose.position.x;
	const double relY = snap.relative.pose.position.y;
	const double relZ = snap.relative.pose.position.z;

	if (std::abs(relX) > 1e-4 || std::abs(relY) > 1e-4 || std::abs(relZ) > 1e-4) {
		gap = (float)std::abs(relX);
		offsetY = (float)relY;
		offsetZ = (float)relZ;
	} else if (snap.timeS <= 0.0) {
		gap = 1.2f;
	}

	const glm::mat4 bT = attitude({targetSize.x * 0.5f + mech, 0.0f, 0.0f}, targetAtt);
	const glm::mat4 bC = attitude({-(chaserSize.x * 0.5f + mech + gap), offsetY, offsetZ}, chaserAtt);

	p.drawModel(ModelId::Target, bT);
	p.drawModel(ModelId::Chaser, bC);

	const glm::mat4 dC = glm::translate(bC, {chaserSize.x * 0.5f + mech, 0, 0});
	const glm::mat4 dT = glm::rotate(glm::translate(bT, {-(targetSize.x * 0.5f + mech), 0, 0}), glm::radians(180.0f), {0, 0, 1});

	p.label(glm::vec3(bC * glm::vec4(0, 0, -chaserSize.z * 0.5f - 0.35f, 1)), "CHASER", LabelKind::Object, -1.0f);
	p.label(glm::vec3(bT * glm::vec4(0, 0, -targetSize.z * 0.5f - 0.35f, 1)), "TARGET", LabelKind::Object, 1.0f);

	p.overlayPass();
	p.drawTriad(bC, 0.9f, "B_C");
	p.drawTriad(dC, 0.7f, "D_C");
	p.drawTriad(dT, 0.7f, "D_T");
	p.drawTriad(bT, 0.9f, "B_T");
}

} // namespace vde::render
