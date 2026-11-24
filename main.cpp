/**
 * =========================================================================================
 * BÀI TẬP LỚN ĐỒ HỌA MÁY TÍNH - C++ & OPENGL / GLUT
 * ĐỀ TÀI: MÔ PHỎNG ĐIỀU KHIỂN DRONE BAY QUANH ĐÔ THỊ HIỆN ĐẠI (PHONG CÁCH HÌNH 2)
 * =========================================================================================
 * 
 * PHONG CÁCH THIẾT KẾ ĐỒ HỌA (THEO HÌNH ẢNH MẪU 2):
 * - Bảng màu kiến trúc trang nhã, hiện đại, có chiều sâu:
 *   + Tòa chọc trời Trắng sứ cao vút với dải kính xanh dương thẫm (White Skyscraper).
 *   + Tòa nhà Vàng đất hoàng thổ / Ochre ấm áp với ô cửa sổ gạch nung (Yellow Brick Building).
 *   + Cao ốc Cam đất Terracotta với gờ mái xám sẫm và khung trắng.
 *   + Tháp trụ tròn viễn thông Trắng muốt với đài quan sát và cột kim thu lôi (Observation Needle).
 *   + Tòa tháp mái vòm xanh ngọc bích / Lam ngọc cổ điển pha hiện đại (Teal Dome Landmark).
 *   + Cao ốc kính Bạc xám thanh lịch với các tầng giật cấp (Silver-Grey High-rise).
 * - Cảnh quan đô thị sinh động:
 *   + Mặt đường nhựa xám tím sẫm (Slate Asphalt) với vạch kẻ đường trắng muốt tinh tế.
 *   + Thảm cỏ xanh mạ tươi tắn (Fresh Lime Green) bao bọc quanh các khối nhà và vỉa hè.
 *   + Hàng cây đô thị tán tròn xum xuê hai tầng lá xanh ngọc mướt mát.
 *   + Xe cộ kiểu dáng thanh lịch (taxi vàng, xe con trắng, xe thể thao đỏ, xanh dương).
 * - Bầu trời xanh ngọc ban ngày với nắng vàng rực rỡ và những đám mây trắng bồng bềnh;
 *   khi chuyển đêm: thành phố lên đèn ấm áp (đèn vàng ấm, đèn trắng phòng) tinh tế, sang trọng.
 * - Điều khiển Drone vật lý: W/S (tiến/lùi), A/D (bay ngang), T/G hoặc Space/Q (lên/xuống),
 *   rê chuột FPV lượn cua nghiêng thân mượt mà.
 * - 3 Chế độ Camera: [0] FPV Buồng lái, [1] Bám theo sau đuôi, [2] Quỹ đạo tự do.
 * - Radar Minimap thời gian thực góc trên bên phải thiết kế sáng sủa, dễ nhìn.
 * =========================================================================================
 */

#include <GL/glut.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace CityDroneSimulation {

// =========================================================================================
// 1. HẰNG SỐ VÀ CẤU TRÚC HÌNH HỌC 3D
// =========================================================================================
const float PI = 3.14159265358979323846f;
const float DEG2RAD = PI / 180.0f;
const float RAD2DEG = 180.0f / PI;
const float ROAD_HALF_WIDTH = 8.0f;
const float CITY_GRID_SPACING = 17.5f;
// Thành phố là một khu vực đóng, gồm 4 x 4 quận, mỗi quận 105 m.
const int CITY_MIN_ROAD_INDEX = -12;
const int CITY_MAX_ROAD_INDEX = 12;
const int CITY_MIN_DISTRICT = -2;
const int CITY_MAX_DISTRICT = 1;
const float CITY_EDGE = 210.0f;
const float DRONE_MIN_X = -200.0f;
const float DRONE_MAX_X = 680.0f; // Giới hạn phía Đông ở đảo ngoài biển.
const float DRONE_LIMIT_Z = 200.0f;

// 2 Tòa Siêu Cao Ốc Landmark mang Bãi Đáp Trực Thăng (Helipads)
const float HELIPAD_1_X = 83.7f;
const float HELIPAD_1_Z = 83.7f;
const float HELIPAD_1_Y = 74.0f;

const float HELIPAD_2_X = -126.3f;
const float HELIPAD_2_Z = -21.3f;
const float HELIPAD_2_Y = 74.0f;

struct Vec3 {
    float x, y, z;
    Vec3(float X = 0.0f, float Y = 0.0f, float Z = 0.0f) : x(X), y(Y), z(Z) {}
    Vec3 operator+(const Vec3& b) const { return Vec3(x + b.x, y + b.y, z + b.z); }
    Vec3 operator-(const Vec3& b) const { return Vec3(x - b.x, y - b.y, z - b.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(float s) const { return Vec3(x / s, y / s, z / s); }
};

inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
inline float length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalized(const Vec3& v) {
    float l = length(v);
    return l > 1e-6f ? v * (1.0f / l) : Vec3(0, 1, 0);
}
inline float clampf(float val, float low, float high) {
    return std::max(low, std::min(high, val));
}

struct Color4 {
    float r, g, b, a;
    Color4(float R = 0, float G = 0, float B = 0, float A = 1.0f) : r(R), g(G), b(B), a(A) {}
    static Color4 lerp(const Color4& c1, const Color4& c2, float t) {
        t = clampf(t, 0.0f, 1.0f);
        return Color4(
            c1.r + (c2.r - c1.r) * t,
            c1.g + (c2.g - c1.g) * t,
            c1.b + (c2.b - c1.b) * t,
            c1.a + (c2.a - c1.a) * t
        );
    }
};

struct Building {
    float x, z, w, d, h;
    int style;
};

// =========================================================================================
// 2. BIẾN TRẠNG THÁI DRONE, CAMERA VÀ THỜI GIAN
// =========================================================================================
bool keys[256] = {};
bool specialKeys[256] = {};

// Vật lý Drone do người chơi điều khiển
Vec3 drone(0.0f, 18.0f, 0.0f);
Vec3 droneVelocity(0.0f, 0.0f, 0.0f);
float droneRoll = 0.0f;
float dronePitch = 0.0f;
float rotorAngle = 0.0f;
float mouseRollImpulse = 0.0f;
float forwardSpeed = 16.0f;
float invulnerableTimer = 0.0f;
bool paused = false;
bool wireframeMode = false;

// Cải tiến vật lý, rung lắc và hỗ trợ hiển thị HUD
float cameraShakeTimer = 0.0f;
float collisionFlashTimer = 0.0f;
float proximityDist = 999.0f;
Vec3 smoothCameraEye(0.0f, 20.0f, 20.0f);
Vec3 smoothCameraLook(0.0f, 18.0f, 0.0f);
bool smoothCamInitialized = false;
bool isDroneLanded = false;
bool searchlightOn = true;

// Hệ quan sát Camera
int cameraMode = 1;               // 0: FPV, 1: Bám theo sau đuôi, 2: Quỹ đạo tự do
bool parallelProjection = false;
int parallelPlane = 0;            // 0: XOY, 1: YOZ, 2: ZOX
float theta = 0.0f;
float phi = 0.05f;
float cameraR = 22.0f;
float zoomD = 1.0f;
int windowWidth = 1280;
int windowHeight = 720;
int lastTick = 0;

// Chu kỳ Ngày - Đêm
float timeOfDay = 11.5f;          // Mặc định khởi đầu lúc 11h30 (Ban ngày nắng đẹp như Hình 2)
float timeSpeed = 0.08f;        // Giữ cảnh ban ngày đủ lâu để dạo phố trước khi chuyển tối.
bool timeAuto = true;
float nightFactor = 0.0f;         // 0.0: Nắng ban ngày -> 1.0: Đêm trăng
float animTime = 0.0f;

// Xe cộ chạy trên phố
struct CityCar {
    float x, z;
    float speed;
    float dir;
    Color4 color;
    bool runsAlongX;
    float currentSpeed;
    CityCar(float X = 0.0f, float Z = 0.0f, float S = 18.0f, float D = 1.0f, Color4 C = Color4(), bool RAX = false, float CS = -1.0f)
        : x(X), z(Z), speed(S), dir(D), color(C), runsAlongX(RAX), currentSpeed(CS < 0.0f ? S : CS) {}
};
std::vector<CityCar> cityCars;

// =========================================================================================
// 3. TOÁN HỌC VECTOR & HASH PROCEDURAL
// =========================================================================================
Vec3 lookDirection() {
    float cp = std::cos(phi);
    return normalized(Vec3(std::sin(theta) * cp, std::sin(phi), -std::cos(theta) * cp));
}

Vec3 strafeDirection() {
    return Vec3(std::cos(theta), 0.0f, std::sin(theta));
}

unsigned int hashInt(int n, unsigned int salt = 0x9e3779b9u) {
    unsigned int x = (unsigned int)n ^ salt;
    x ^= x >> 16; x *= 0x7feb352du; x ^= x >> 15; x *= 0x846ca68bu; x ^= x >> 16;
    return x;
}

float hash01(int n, unsigned int salt = 0x9e3779b9u) {
    return (hashInt(n, salt) & 0x00ffffffu) / 16777215.0f;
}

float gridHash01(int x, int z, unsigned int salt = 0x9e3779b9u) {
    unsigned int packed = (unsigned int)x * 73856093u ^ (unsigned int)z * 19349663u;
    return (hashInt((int)packed, salt) & 0x00ffffffu) / 16777215.0f;
}

bool isStreetIndex(int index) {
    return (index % 6) == 0;
}

int floorDiv6(int value) {
    int q = value / 6, r = value % 6;
    return (r < 0) ? q - 1 : q;
}

enum AmenityType {
    AMENITY_CITY = 0, AMENITY_PARK, AMENITY_MALL, AMENITY_STADIUM,
    AMENITY_RESIDENTIAL, AMENITY_SCHOOL, AMENITY_HOSPITAL
};

int districtAmenity(int districtX, int districtZ) {
    // Bố cục Đô Thị Phân Khu (Zoning Masterplan theo Hình mẫu kiến trúc)
    if (districtX == 0 && districtZ == 0) return AMENITY_CITY; // Khu Đô Thị Trung Tâm theo hình mẫu
    if (districtX == -1 && districtZ == 0) return AMENITY_PARK;
    if (districtX == 0 && districtZ == -1) return AMENITY_STADIUM;
    if (districtX == 1 && districtZ == 0) return AMENITY_SCHOOL;
    if (districtX == -1 && districtZ == -1) return AMENITY_HOSPITAL;
    if (districtX == 0 && districtZ == 1) return AMENITY_MALL; // TTTM dời lên quận (0, 1)
    if ((districtX == 1 && districtZ == 1) ||
        (districtX == -2 && districtZ == 1) ||
        (districtX == 1 && districtZ == -1)) return AMENITY_RESIDENTIAL;
    return AMENITY_CITY;
}

// Mỗi ô phố lớn có khoảng trống cho quảng trường/tiện ích; nhà thường nằm lùi vào trong lô.
bool hasBuildingAt(int gridX, int gridZ) {
    if (gridX < CITY_MIN_ROAD_INDEX || gridX > CITY_MAX_ROAD_INDEX ||
        gridZ < CITY_MIN_ROAD_INDEX || gridZ > CITY_MAX_ROAD_INDEX) return false;
    if (isStreetIndex(gridX) || isStreetIndex(gridZ) || (gridX % 2 == 0) || (gridZ % 2 == 0)) return false;
    int district = districtAmenity(floorDiv6(gridX), floorDiv6(gridZ));
    return district == AMENITY_CITY || district == AMENITY_RESIDENTIAL;
}

// =========================================================================================
// 4. THIẾT LẬP VẬT LIỆU CHUẨN (PHONG CÁCH VECTOR 3D SÁNG RÕ)
// =========================================================================================
void setMaterial(float r, float g, float b, float shine = 25.0f, float spec = 0.35f) {
    GLfloat amb[]  = { r * 0.45f, g * 0.45f, b * 0.48f, 1.0f };
    GLfloat diff[] = { r, g, b, 1.0f };
    GLfloat sp[]   = { spec, spec, spec, 1.0f };
    GLfloat emit[] = { 0.0f, 0.0f, 0.0f, 1.0f };

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, sp);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shine);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emit);
    // Đồng bộ màu hiện hành với vật liệu khi GL_COLOR_MATERIAL đang bật.
    glColor4f(r, g, b, 1.0f);
}

void setWindowMaterial(float r, float g, float b, bool isLitNight = false) {
    if (isLitNight && nightFactor > 0.05f) {
        // Đèn phòng ấm áp về đêm
        float emit = nightFactor * 0.85f;
        GLfloat amb[]  = { 1.0f, 0.88f, 0.50f, 1.0f };
        GLfloat diff[] = { 1.0f, 0.90f, 0.60f, 1.0f };
        GLfloat sp[]   = { 0.5f, 0.5f, 0.5f, 1.0f };
        GLfloat em[]   = { 1.0f * emit, 0.88f * emit, 0.50f * emit, 1.0f };

        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, sp);
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 30.0f);
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, em);
    } else {
        // Kính ban ngày phẳng mịn phản chiếu màu trời
        setMaterial(r, g, b, 50.0f, 0.65f);
    }
}

// =========================================================================================
// 5. THUẬT TOÁN HÌNH HỌC PROCEDURAL
// =========================================================================================
void outwardTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& outward) {
    Vec3 n = cross(b - a, c - a), p1 = b, p2 = c;
    if (dot(n, outward) < 0) { p1 = c; p2 = b; n = n * (-1.0f); }
    n = normalized(n);
    glNormal3f(n.x, n.y, n.z);
    glVertex3f(a.x, a.y, a.z); glVertex3f(p1.x, p1.y, p1.z); glVertex3f(p2.x, p2.y, p2.z);
}

void drawCube(float x, float y, float z) {
    glPushMatrix();
    glScalef(x, y, z);
    glutSolidCube(1.0);
    glPopMatrix();
}

void drawCylinder(float radius, float height, int slices = 24) {
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < slices; i++) {
        float a0 = 2.0f * PI * i / slices, a1 = 2.0f * PI * (i + 1) / slices;
        float c0 = std::cos(a0), s0 = std::sin(a0);
        float c1 = std::cos(a1), s1 = std::sin(a1);

        Vec3 b0(radius * c0, 0.0f,   radius * s0);
        Vec3 b1(radius * c1, 0.0f,   radius * s1);
        Vec3 t0(radius * c0, height, radius * s0);
        Vec3 t1(radius * c1, height, radius * s1);

        glNormal3f(c0, 0.0f, s0); glVertex3f(b0.x, b0.y, b0.z);
        glNormal3f(c0, 0.0f, s0); glVertex3f(t0.x, t0.y, t0.z);
        glNormal3f(c1, 0.0f, s1); glVertex3f(t1.x, t1.y, t1.z);

        glNormal3f(c0, 0.0f, s0); glVertex3f(b0.x, b0.y, b0.z);
        glNormal3f(c1, 0.0f, s1); glVertex3f(t1.x, t1.y, t1.z);
        glNormal3f(c1, 0.0f, s1); glVertex3f(b1.x, b1.y, b1.z);

        glNormal3f(0.0f, 1.0f, 0.0f); glVertex3f(0.0f, height, 0.0f);
        glNormal3f(0.0f, 1.0f, 0.0f); glVertex3f(t1.x, t1.y, t1.z);
        glNormal3f(0.0f, 1.0f, 0.0f); glVertex3f(t0.x, t0.y, t0.z);

        glNormal3f(0.0f, -1.0f, 0.0f); glVertex3f(0.0f, 0.0f, 0.0f);
        glNormal3f(0.0f, -1.0f, 0.0f); glVertex3f(b0.x, b0.y, b0.z);
        glNormal3f(0.0f, -1.0f, 0.0f); glVertex3f(b1.x, b1.y, b1.z);
    }
    glEnd();
}

void drawCone(float radius, float height, int slices = 24) {
    Vec3 tip(0.0f, height, 0.0f);
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < slices; i++) {
        float a0 = 2.0f * PI * i / slices, a1 = 2.0f * PI * (i + 1) / slices;
        float c0 = std::cos(a0), s0 = std::sin(a0);
        float c1 = std::cos(a1), s1 = std::sin(a1);
        Vec3 b0(radius * c0, 0.0f, radius * s0);
        Vec3 b1(radius * c1, 0.0f, radius * s1);

        outwardTriangle(b0, tip, b1, Vec3(std::cos((a0 + a1) * 0.5f), radius / height, std::sin((a0 + a1) * 0.5f)));

        glNormal3f(0.0f, -1.0f, 0.0f); glVertex3f(0.0f, 0.0f, 0.0f);
        glNormal3f(0.0f, -1.0f, 0.0f); glVertex3f(b0.x, b0.y, b0.z);
        glNormal3f(0.0f, -1.0f, 0.0f); glVertex3f(b1.x, b1.y, b1.z);
    }
    glEnd();
}

void drawDuctRing(float innerR, float outerR, float height, int slices = 24) {
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= slices; ++i) {
        float a = 2.0f * PI * i / slices;
        float c = std::cos(a), s = std::sin(a);
        glNormal3f(-c, 0.0f, -s);
        glVertex3f(innerR * c, -height * 0.5f, innerR * s);
        glVertex3f(innerR * c,  height * 0.5f, innerR * s);
    }
    glEnd();
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= slices; ++i) {
        float a = 2.0f * PI * i / slices;
        float c = std::cos(a), s = std::sin(a);
        glNormal3f(c, 0.0f, s);
        glVertex3f(outerR * c,  height * 0.5f, outerR * s);
        glVertex3f(outerR * c, -height * 0.5f, outerR * s);
    }
    glEnd();
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= slices; ++i) {
        float a = 2.0f * PI * i / slices;
        float c = std::cos(a), s = std::sin(a);
        glNormal3f(0.0f, 1.0f, 0.0f);
        glVertex3f(innerR * c, height * 0.5f, innerR * s);
        glVertex3f(outerR * c, height * 0.5f, outerR * s);
    }
    glEnd();
}

void drawPropellerDisk(float radius, float alpha) {
    glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glBegin(GL_TRIANGLE_FAN);
    glColor4f(0.25f, 0.85f, 1.0f, alpha * 0.65f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    for (int i = 0; i <= 24; ++i) {
        float a = 2.0f * PI * i / 24.0f;
        glColor4f(0.15f, 0.70f, 0.95f, alpha * 0.22f);
        glVertex3f(radius * std::cos(a), 0.0f, radius * std::sin(a));
    }
    glEnd();
    glPopAttrib();
}

// Đường cong Bezier bậc 3 vẽ dây cáp điện
Vec3 bezierPoint(const Vec3 cp[4], float t) {
    float u = 1.0f - t;
    return cp[0] * (u * u * u) + cp[1] * (3.0f * u * u * t) + cp[2] * (3.0f * u * t * t) + cp[3] * (t * t * t);
}

void drawBezierCable(const Vec3 cp[4]) {
    glLineWidth(2.0f);
    setMaterial(0.25f, 0.28f, 0.32f, 20.0f);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= 28; i++) {
        float t = i / 28.0f;
        Vec3 p = bezierPoint(cp, t);
        glVertex3f(p.x, p.y, p.z);
    }
    glEnd();
    glLineWidth(1.0f);
}

// Mặt cong Bezier Bicubic vẽ mái vòm nhà ga Metro xanh ngọc lam (như hình 2)
float bernstein(int i, float t) {
    float u = 1.0f - t;
    if (i == 0) return u * u * u;
    if (i == 1) return 3.0f * u * u * t;
    if (i == 2) return 3.0f * u * t * t;
    return t * t * t;
}

Vec3 bezierPatchPoint(const Vec3 cp[4][4], float u, float v) {
    Vec3 p;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            p = p + cp[i][j] * (bernstein(i, u) * bernstein(j, v));
    return p;
}

void drawBezierDome(float cx, float baseY, float cz) {
    Vec3 cp[4][4];
    const float xs[4] = { -8.5f, -3.5f, 3.5f, 8.5f };
    const float zs[4] = { -5.5f, -1.8f, 1.8f, 5.5f };
    const float arch[4] = { 0.0f, 5.5f, 5.5f, 0.0f };

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            cp[i][j] = Vec3(cx + xs[i], baseY + arch[i] + ((j == 1 || j == 2) ? 1.0f : 0.0f), cz + zs[j]);
        }
    }

    const int n = 16;
    // Mái vòm xanh ngọc lam lam ngọc (Teal / Turquoise như tòa nhà mái vòm xanh trong hình 2)
    setMaterial(0.18f, 0.60f, 0.72f, 65.0f, 0.6f);
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            float u0 = (float)i / n, u1 = (float)(i + 1) / n;
            float v0 = (float)j / n, v1 = (float)(j + 1) / n;

            Vec3 p00 = bezierPatchPoint(cp, u0, v0), p10 = bezierPatchPoint(cp, u1, v0);
            Vec3 p01 = bezierPatchPoint(cp, u0, v1), p11 = bezierPatchPoint(cp, u1, v1);

            Vec3 du = bezierPatchPoint(cp, std::min(1.0f, u0 + 0.002f), v0) - bezierPatchPoint(cp, std::max(0.0f, u0 - 0.002f), v0);
            Vec3 dv = bezierPatchPoint(cp, u0, std::min(1.0f, v0 + 0.002f)) - bezierPatchPoint(cp, u0, std::max(0.0f, v0 - 0.002f));
            Vec3 normal = normalized(cross(dv, du));

            glNormal3f(normal.x, normal.y, normal.z); glVertex3f(p00.x, p00.y, p00.z);
            glNormal3f(normal.x, normal.y, normal.z); glVertex3f(p01.x, p01.y, p01.z);
            glNormal3f(normal.x, normal.y, normal.z); glVertex3f(p11.x, p11.y, p11.z);

            glNormal3f(normal.x, normal.y, normal.z); glVertex3f(p00.x, p00.y, p00.z);
            glNormal3f(normal.x, normal.y, normal.z); glVertex3f(p11.x, p11.y, p11.z);
            glNormal3f(normal.x, normal.y, normal.z); glVertex3f(p10.x, p10.y, p10.z);
        }
    }
    glEnd();

    // Khung viền màu trắng kem
    setMaterial(0.92f, 0.94f, 0.95f, 40.0f);
    glPushMatrix();
    glTranslatef(cx, baseY + 0.1f, cz);
    drawCube(17.2f, 0.25f, 11.2f);
    glPopMatrix();
}

// =========================================================================================
// 6. MÔ HÌNH DRONE NGƯỜI CHƠI (HIỆN ĐẠI, THỂ THAO, CÓ GIMBAL CAMERA VÀ VÒNG BẢO VỆ CÁNH)
// =========================================================================================
void drawDrone() {
    float hoverBob = (1.0f - std::min(1.0f, length(droneVelocity) * 0.12f)) * std::sin(animTime * 3.2f) * 0.05f;

    glPushMatrix();
    glTranslatef(drone.x, drone.y + hoverBob, drone.z);
    glRotatef(droneRoll, 0.0f, 0.0f, 1.0f);
    glRotatef(dronePitch, 1.0f, 0.0f, 0.0f);
    glRotatef(-theta * 180.0f / PI, 0.0f, 1.0f, 0.0f);

    // 1. THÂN CHÍNH KHÍ ĐỘNG HỌC (Aerodynamic Fuselage)
    // Khoang lưng trên (Top Shell - Sợi carbon xám than mờ sang trọng)
    setMaterial(0.18f, 0.21f, 0.26f, 85.0f, 0.85f);
    glPushMatrix();
    glTranslatef(0.0f, 0.02f, -0.02f);
    glScalef(0.36f, 0.12f, 0.52f);
    glutSolidSphere(1.0, 18, 14);
    glPopMatrix();

    // Mũi khí động học phía trước (Nose Cone)
    setMaterial(0.24f, 0.28f, 0.35f, 90.0f, 0.8f);
    glPushMatrix();
    glTranslatef(0.0f, 0.035f, -0.28f);
    glScalef(0.22f, 0.075f, 0.24f);
    glutSolidSphere(1.0, 16, 10);
    glPopMatrix();

    // Bụng máy bay bảo vệ pin (Underbody Battery Compartment)
    setMaterial(0.11f, 0.13f, 0.17f, 60.0f, 0.5f);
    glPushMatrix();
    glTranslatef(0.0f, -0.12f, 0.02f);
    glScalef(0.28f, 0.08f, 0.44f);
    glutSolidSphere(1.0, 16, 10);
    glPopMatrix();

    // Dải decal sọc thể thao màu cam neon (Neon Racing Decal)
    setMaterial(1.0f, 0.44f, 0.08f, 70.0f, 0.4f);
    glPushMatrix();
    glTranslatef(0.0f, 0.12f, 0.01f);
    glScalef(0.055f, 0.025f, 0.38f);
    glutSolidCube(1.0);
    glPopMatrix();

    // Cảm biến tránh va chạm kép phía trước (Obstacle Avoidance Dual Vision Sensors)
    setMaterial(0.04f, 0.06f, 0.09f, 95.0f, 0.95f);
    for (int s = -1; s <= 1; s += 2) {
        glPushMatrix();
        glTranslatef(s * 0.10f, 0.04f, -0.42f);
        glScalef(0.032f, 0.032f, 0.02f);
        glutSolidSphere(1.0, 12, 8);
        glPopMatrix();
    }

    // 2. CỤM GIMBAL CAMERA 3 TRỤC DƯỚI MŨI (3-Axis Gimbal & 4K Camera)
    glPushMatrix();
    glTranslatef(0.0f, -0.12f, -0.32f);
    // Khung treo Gimbal (Gimbal Yaw Bracket)
    setMaterial(0.35f, 0.38f, 0.42f, 80.0f, 0.8f);
    glPushMatrix();
    glScalef(0.09f, 0.03f, 0.09f);
    glutSolidCube(1.0);
    glPopMatrix();

    // Trục xoay gật camera theo góc ngắm (Camera Pitch)
    glRotatef(clampf(phi * RAD2DEG * 0.45f - 8.0f, -40.0f, 25.0f), 1.0f, 0.0f, 0.0f);

    // Thân Camera Pod
    glTranslatef(0.0f, -0.075f, -0.03f);
    setMaterial(0.12f, 0.14f, 0.18f, 90.0f, 0.85f);
    glPushMatrix();
    glScalef(0.11f, 0.095f, 0.14f);
    glutSolidSphere(1.0, 16, 12);
    glPopMatrix();

    // Vành kim loại vàng quanh ống kính (Gold Accent Ring)
    setMaterial(0.95f, 0.78f, 0.22f, 95.0f, 0.95f);
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, -0.10f);
    drawCylinder(0.065f, 0.025f, 16);
    glPopMatrix();

    // Thấu kính Camera kính xanh ngọc bích bóng bẩy (Reflective Blue/Cyan Lens)
    setMaterial(0.12f, 0.65f, 0.88f, 100.0f, 1.0f);
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, -0.115f);
    glScalef(0.055f, 0.055f, 0.018f);
    glutSolidSphere(1.0, 14, 8);
    glPopMatrix();
    glPopMatrix();

    // 3. BỐN CÀNG CARBON, ĐỘNG CƠ, VÒNG BẢO VỆ CÁNH (Propeller Ducts) VÀ CÁNH QUẠT 3 LÁ
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sz = -1; sz <= 1; sz += 2) {
            float rx = 0.86f * sx, rz = 0.66f * sz;

            // Càng đỡ carbon gia cường (Carbon Fiber Arm)
            setMaterial(0.14f, 0.17f, 0.22f, 75.0f, 0.7f);
            glPushMatrix();
            glTranslatef(rx * 0.50f, -0.02f, rz * 0.50f);
            glRotatef(std::atan2(-rz, rx) * RAD2DEG, 0.0f, 1.0f, 0.0f);
            glScalef(std::sqrt(rx * rx + rz * rz) * 0.58f, 0.048f, 0.07f);
            glutSolidCube(1.0);
            glPopMatrix();

            // Cụm động cơ (Brushless Motor Pod)
            glPushMatrix();
            glTranslatef(rx, -0.02f, rz);

            // Thân động cơ xám titan
            setMaterial(0.30f, 0.33f, 0.38f, 85.0f, 0.8f);
            drawCylinder(0.13f, 0.16f, 18);

            // Nắp chuông động cơ màu đồng ánh kim
            glTranslatef(0.0f, 0.16f, 0.0f);
            setMaterial(0.85f, 0.68f, 0.28f, 90.0f, 0.85f);
            drawCylinder(0.135f, 0.035f, 18);

            // VÒNG BẢO VỆ CÁNH (Propeller Duct / Bumper Ring)
            glPushMatrix();
            glTranslatef(0.0f, 0.04f, 0.0f);
            setMaterial(0.20f, 0.24f, 0.30f, 60.0f, 0.5f);
            drawDuctRing(0.39f, 0.43f, 0.065f, 24);

            // 3 Nan hoa nối từ động cơ ra vòng bảo vệ
            for (int sp = 0; sp < 3; ++sp) {
                glPushMatrix();
                glRotatef(sp * 120.0f, 0.0f, 1.0f, 0.0f);
                glTranslatef(0.22f, 0.0f, 0.0f);
                glScalef(0.18f, 0.015f, 0.024f);
                glutSolidCube(1.0);
                glPopMatrix();
            }
            glPopMatrix();

            // CÁNH QUẠT 3 LÁ (Tri-blade Rotor) + ĐĨA MOTION BLUR
            glTranslatef(0.0f, 0.055f, 0.0f);
            drawPropellerDisk(0.39f, 0.45f); // Đĩa quay mờ xé gió

            glRotatef(rotorAngle * (sx * sz), 0.0f, 1.0f, 0.0f);
            for (int b = 0; b < 3; ++b) {
                glPushMatrix();
                glRotatef(b * 120.0f, 0.0f, 1.0f, 0.0f);
                glTranslatef(0.16f, 0.0f, 0.0f);
                glScalef(0.18f, 0.012f, 0.038f);
                setMaterial(0.12f, 0.68f, 0.92f, 75.0f, 0.7f); // Cánh quạt xanh thể thao
                glutSolidSphere(1.0, 10, 6);
                glPopMatrix();
            }

            // ĐÈN LED ĐỊNH VỊ HÀNG KHÔNG (Aviation LEDs)
            // Phía trước: Đỏ bên trái (Port), Xanh lá bên phải (Starboard)
            // Phía sau: Xanh dương; chớp trắng định kỳ (Strobe flash)
            bool isStrobe = std::fmod(animTime, 0.55f) < 0.20f;
            if (sz < 0) {
                if (sx < 0) setMaterial(0.98f, 0.15f, 0.15f, 40.0f, 0.2f); // LED Đỏ
                else        setMaterial(0.15f, 0.95f, 0.25f, 40.0f, 0.2f); // LED Xanh lá
            } else {
                if (isStrobe) setMaterial(0.98f, 0.98f, 1.0f, 100.0f, 0.9f); // Chớp trắng
                else          setMaterial(0.20f, 0.50f, 0.98f, 40.0f, 0.2f); // Xanh dương
            }
            glPushMatrix();
            glTranslatef(0.0f, -0.20f, 0.0f);
            glScalef(0.065f, 0.045f, 0.065f);
            glutSolidSphere(1.0, 10, 8);
            glPopMatrix();

            glPopMatrix();
        }
    }

    // 4. HAI CÀNG TIẾP ĐẤT (Landing Skids)
    setMaterial(0.68f, 0.72f, 0.76f, 60.0f, 0.5f);
    for (int side = -1; side <= 1; side += 2) {
        // Chân chống đứng
        glPushMatrix();
        glTranslatef(side * 0.26f, -0.22f, 0.06f);
        glScalef(0.03f, 0.16f, 0.03f);
        glutSolidCube(1.0);
        glPopMatrix();

        // Thanh trượt ngang tiếp xúc mặt đất
        glPushMatrix();
        glTranslatef(side * 0.26f, -0.30f, 0.02f);
        glScalef(0.04f, 0.028f, 0.36f);
        glutSolidCube(1.0);
        glPopMatrix();
    }

    // 5. ĐÈN PHA RỌI TRƯỚC (Front LED Searchlight)
    setMaterial(1.0f, 0.98f, 0.75f, 90.0f, 0.9f);
    glPushMatrix();
    glTranslatef(0.0f, -0.04f, -0.48f);
    glScalef(0.08f, 0.07f, 0.10f);
    glutSolidSphere(1.0, 12, 8);
    glPopMatrix();



    glPopMatrix();
}

/**
 * Vẽ Hiệu ứng Luồng gió thổi bụi & Sóng khí động học dưới gầm Drone (Downwash Wind & Ground Dust)
 */
void drawDroneDownwash() {
    float groundY = -0.32f;
    float d1 = std::hypot(drone.x - HELIPAD_1_X, drone.z - HELIPAD_1_Z);
    float d2 = std::hypot(drone.x - HELIPAD_2_X, drone.z - HELIPAD_2_Z);
    if ((d1 < 5.2f || d2 < 5.2f) && drone.y > 70.0f) groundY = HELIPAD_1_Y;

    float distToFloor = drone.y - groundY;
    if (distToFloor > 0.1f && distToFloor < 5.5f) {
        float factor = 1.0f - distToFloor / 5.5f;

        glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_LIGHTING);
        glDepthMask(GL_FALSE);

        // 1. Vòng sóng khí động học giãn nở lan tỏa trên mặt sàn
        for (int ring = 0; ring < 2; ++ring) {
            float phase = std::fmod(animTime * 3.5f + ring * 0.5f, 1.0f);
            float ringR = 0.8f + phase * 3.4f;
            float ringAlpha = (1.0f - phase) * 0.40f * factor;

            glPushMatrix();
            glTranslatef(drone.x, groundY + 0.025f, drone.z);
            glLineWidth(2.2f);
            glColor4f(0.85f, 0.95f, 1.0f, ringAlpha);
            glBegin(GL_LINE_LOOP);
            for (int i = 0; i < 24; ++i) {
                float a = 2.0f * PI * i / 24.0f;
                glVertex3f(std::cos(a) * ringR, 0.0f, std::sin(a) * ringR);
            }
            glEnd();
            glPopMatrix();
        }

        // 2. Các hạt bụi/gió li ti xoáy tản tròn dưới 4 luồng cánh quạt
        for (int p = 0; p < 12; ++p) {
            float seed = (float)p * (2.0f * PI / 12.0f);
            float pDist = 1.2f + 2.5f * std::fmod(animTime * 4.0f + p * 0.18f, 1.0f);
            float pAngle = seed + animTime * 8.0f;
            float px = drone.x + std::cos(pAngle) * pDist;
            float pz = drone.z + std::sin(pAngle) * pDist;
            float pAlpha = (1.0f - (pDist - 1.2f) / 2.5f) * 0.45f * factor;

            glPushMatrix();
            glTranslatef(px, groundY + 0.03f, pz);
            glColor4f(0.92f, 0.90f, 0.80f, pAlpha);
            glScalef(0.12f, 0.04f, 0.12f);
            glutSolidSphere(1.0, 6, 4);
            glPopMatrix();
        }

        glLineWidth(1.0f);
        glPopAttrib();
    }
}

void drawDroneShadow() {
    float groundY = -0.32f;
    if (drone.y > groundY + 0.1f && drone.y < 75.0f) {
        float h = drone.y - groundY;
        float alpha = clampf((1.0f - h / 75.0f) * 0.42f, 0.04f, 0.45f);
        float radius = 0.85f + h * 0.05f;

        glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_LIGHTING);
        glDepthMask(GL_FALSE);

        // 1. Bóng đổ mềm mặt đất dưới Drone
        glPushMatrix();
        glTranslatef(drone.x, groundY + 0.015f, drone.z);
        glBegin(GL_TRIANGLE_FAN);
            glColor4f(0.03f, 0.05f, 0.08f, alpha);
            glVertex3f(0.0f, 0.0f, 0.0f);
            for (int i = 0; i <= 24; ++i) {
                float a = 2.0f * PI * i / 24.0f;
                glColor4f(0.03f, 0.05f, 0.08f, 0.0f);
                glVertex3f(std::cos(a) * radius * 1.35f, 0.0f, std::sin(a) * radius * 1.15f);
            }
        glEnd();
        glPopMatrix();



        glPopAttrib();
    }
}

// =========================================================================================
// 7. KIẾN TRÚC CAO ỐC PHONG CÁCH HÌNH 2 (ĐỒ HỌA SÁNG RÕ, THANH LỊCH)
// =========================================================================================
Building getBuilding(int gridX, int gridZ) {
    int district = districtAmenity(floorDiv6(gridX), floorDiv6(gridZ));
    bool residential = district == AMENITY_RESIDENTIAL;
    int localX = ((gridX % 6) + 6) % 6;
    int localZ = ((gridZ % 6) + 6) % 6;

    // Lùi vào trong lô để chừa vỉa hè rộng và sân quảng trường đi bộ
    float insetX = localX == 1 ? 3.8f : (localX == 5 ? -3.8f : 0.0f);
    float insetZ = localZ == 1 ? 4.5f : (localZ == 5 ? -3.8f : 0.0f);

    float x = gridX * CITY_GRID_SPACING + insetX;
    float z = gridZ * CITY_GRID_SPACING + insetZ;

    float width = 14.0f, depth = 13.5f, h = 30.0f;
    int style = 0;

    if (residential) {
        width = 12.0f + gridHash01(gridX, gridZ, 0x713u) * 2.0f;
        depth = 11.5f + gridHash01(gridX, gridZ, 0x714u) * 2.0f;
        h = 9.0f + gridHash01(gridX, gridZ, 0x552u) * 6.5f;
        float colorPick = gridHash01(gridX, gridZ, 0x331u);
        style = colorPick < 0.42f ? 1 : (colorPick < 0.76f ? 0 : 5);
    } else {
        // BỐ CỤC ĐÔ THỊ GIẬT CẤP HÌNH BẬC THANG / RẠP HÁT (STEPPED AMPHITHEATER SKYLINE THEO HÌNH MẪU)
        // localZ == 1: HÀNG TIỀN CẢNH (THẤP NHẤT - SHOPHOUSE & BOUTIQUE RETAIL)
        if (localZ == 1) {
            if (localX == 1) {
                // Tòa nhà boutique vàng cam ấm (amber), 4 tầng, có khối penthouse vòm và mái che
                width = 13.2f; depth = 12.0f; h = 18.0f; style = 1;
            } else if (localX == 3) {
                // Nhà phố shophouse tím lavender, 4 tầng, có hàng cột sảnh đón và ki-ốt góc
                width = 12.8f; depth = 12.2f; h = 15.0f; style = 2;
            } else {
                // Khối thương mại dịch vụ thấp tầng, 3 tầng, có mái đón kính
                width = 13.5f; depth = 11.5f; h = 11.0f; style = 5;
            }
        }
        // localZ == 3: HÀNG TRUNG CẢNH (CAO VỪA - CAO ỐC VĂN PHÒNG & THÁP LĂNG KÍNH PRISM)
        else if (localZ == 3) {
            if (localX == 1) {
                // Tòa nhà văn phòng gạch nung terracotta với gờ mái xám than sang trọng
                width = 14.2f; depth = 13.8f; h = 34.0f; style = 2;
            } else if (localX == 3) {
                // Tháp văn phòng lăng kính hiện đại (Prism Tower) với đỉnh vát góc
                width = 14.0f; depth = 14.0f; h = 46.0f; style = 3;
            } else {
                // Cao ốc văn phòng thương mại trắng hiện đại có biển quảng cáo LED
                width = 14.5f; depth = 14.0f; h = 38.0f; style = 0;
            }
        }
        // localZ == 5: HÀNG HẬU CẢNH (CAO NHẤT - CỤM 4 SIÊU CAO ỐC BIỂU TƯỢNG)
        else {
            if (localX == 1) {
                // Tháp gạch hoàng thổ Ochre với cột cờ tung bay trên đỉnh
                width = 14.8f; depth = 14.2f; h = 64.0f; style = 1;
            } else if (localX == 3) {
                // SIÊU CAO ỐC KÍNH XANH AZURE CHỌC TRỜI (TÒA NHÀ CAO NHẤT VỚI CỘT KIM THU LÔI ANTENNA)
                width = 16.0f; depth = 15.5f; h = 80.0f; style = 0;
            } else {
                // CAO ỐC BẠC CORPORATE VỚI TẦNG ÁP MÁI LAM NGỌC & BÃI ĐÁP TRỰC THĂNG (HELIPAD TOWER)
                width = 15.5f; depth = 15.0f; h = 74.0f; style = 5;
            }
        }
    }

    return Building{ x, z, width, depth, h, style };
}

void emitWindowQuad(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, const Vec3& normal) {
    glNormal3f(normal.x, normal.y, normal.z);
    glVertex3f(a.x, a.y, a.z); glVertex3f(b.x, b.y, b.z); glVertex3f(c.x, c.y, c.z); glVertex3f(d.x, d.y, d.z);
}

/**
 * Vẽ cửa sổ thanh lịch đồng nhất (như Hình 2: kính xanh navy trên tường trắng, hoặc gạch nung trên tường vàng)
 */
void drawCleanWindows(float w, float h, float d, int style) {
    int floors = std::max(3, std::min(18, (int)(h / 3.4f)));
    int colsW  = std::max(2, std::min(6,  (int)(w / 2.2f)));
    int colsD  = std::max(2, std::min(6,  (int)(d / 2.2f)));
    float floorStep = (h - 1.2f) / floors;
    float paneH = std::min(1.8f, floorStep * 0.65f);

    // Xác định màu kính ban ngày theo phong cách tòa nhà trong Hình 2:
    // - Tòa Trắng/Bạc: kính xanh dương thẫm navy sang trọng (như cao ốc trắng bên phải hình 2)
    // - Tòa Vàng: kính nâu gạch nung / đỏ đô (như tòa nhà vàng ở giữa hình 2)
    // - Tòa Cam: kính xanh đá phiến
    // Kính xanh nhạt tương phản mạnh với các mảng tường màu đỏ, cam, vàng và xanh.
    float winR = 0.25f, winG = 0.50f, winB = 0.65f;
    if (style == 2) { winR = 0.82f; winG = 0.94f; winB = 0.98f; }

    glBegin(GL_QUADS);
    for (int row = 0; row < floors; row++) {
        float y0 = -h * 0.5f + 0.6f + row * floorStep;
        float y1 = y0 + paneH;

        for (int col = 0; col < colsW; col++) {
            float cell = w / colsW, cx = -w * 0.5f + (col + 0.5f) * cell, pw = cell * 0.72f;
            float x0 = cx - pw * 0.5f, x1 = cx + pw * 0.5f;
            float zf = -d * 0.5f - 0.03f, zb = d * 0.5f + 0.03f;

            // Ban đêm: ~65% phòng bật đèn vàng ấm cúng, ~35% phòng tắt đèn giữ kính xanh navy
            bool lit = ((row * 5 + col * 3 + style * 7) % 3) != 0;
            if (lit && nightFactor > 0.05f) {
                glColor3f(1.0f * nightFactor, 0.88f * nightFactor, 0.52f * nightFactor);
            } else {
                glColor3f(winR, winG, winB);
            }

            emitWindowQuad(Vec3(x0, y0, zf), Vec3(x0, y1, zf), Vec3(x1, y1, zf), Vec3(x1, y0, zf), Vec3(0, 0, -1));
            emitWindowQuad(Vec3(x0, y0, zb), Vec3(x1, y0, zb), Vec3(x1, y1, zb), Vec3(x0, y1, zb), Vec3(0, 0, 1));
        }

        for (int col = 0; col < colsD; col++) {
            float cell = d / colsD, cz = -d * 0.5f + (col + 0.5f) * cell, pw = cell * 0.72f;
            float z0 = cz - pw * 0.5f, z1 = cz + pw * 0.5f;
            float xr = w * 0.5f + 0.03f, xl = -w * 0.5f - 0.03f;

            bool lit = ((row * 7 + col * 2 + style * 11) % 3) != 0;
            if (lit && nightFactor > 0.05f) {
                glColor3f(1.0f * nightFactor, 0.88f * nightFactor, 0.52f * nightFactor);
            } else {
                glColor3f(winR, winG, winB);
            }

            emitWindowQuad(Vec3(xr, y0, z0), Vec3(xr, y1, z0), Vec3(xr, y1, z1), Vec3(xr, y0, z1), Vec3(1, 0, 0));
            emitWindowQuad(Vec3(xl, y0, z1), Vec3(xl, y1, z1), Vec3(xl, y1, z0), Vec3(xl, y0, z0), Vec3(-1, 0, 0));
        }
    }
    glEnd();
}

void drawCylinderWindows(float radius, float height, int style) {
    int floors = std::max(4, std::min(18, (int)(height / 3.4f))), sectors = 18;
    float step = (height - 1.0f) / floors, paneH = std::min(1.8f, step * 0.65f), r = radius + 0.03f;

    glBegin(GL_QUADS);
    for (int row = 0; row < floors; row++) {
        float y0 = -height * 0.5f + 0.6f + row * step, y1 = y0 + paneH;
        for (int col = 0; col < sectors; col++) {
            float a0 = 2.0f * PI * (col + 0.15f) / sectors, a1 = 2.0f * PI * (col + 0.75f) / sectors;
            Vec3 n = normalized(Vec3(std::cos((a0 + a1) * 0.5f), 0.0f, std::sin((a0 + a1) * 0.5f)));

            bool lit = ((row * 5 + col * 3 + style * 7) % 3) != 0;
            if (lit && nightFactor > 0.05f) {
                glColor3f(1.0f * nightFactor, 0.88f * nightFactor, 0.52f * nightFactor);
            } else {
                glColor3f(0.16f, 0.28f, 0.44f);
            }

            Vec3 p0(r * std::cos(a0), y0, r * std::sin(a0)), p1(r * std::cos(a0), y1, r * std::sin(a0));
            Vec3 p2(r * std::cos(a1), y1, r * std::sin(a1)), p3(r * std::cos(a1), y0, r * std::sin(a1));
            emitWindowQuad(p0, p1, p2, p3, n);
        }
    }
    glEnd();
}

void drawRooftopHelipad(float size) {
    glPushMatrix();
    // Nền bãi đáp xám than
    setMaterial(0.22f, 0.25f, 0.28f, 20.0f, 0.2f);
    drawCylinder(size * 0.44f, 0.08f, 24);

    // Vành tròn viền vàng rực
    glTranslatef(0.0f, 0.082f, 0.0f);
    setMaterial(0.98f, 0.80f, 0.15f, 30.0f, 0.4f);
    drawDuctRing(size * 0.40f, size * 0.435f, 0.02f, 24);

    // Chữ H màu trắng sắc nét ở tâm
    setMaterial(0.96f, 0.98f, 1.0f, 40.0f, 0.5f);
    float legW = size * 0.055f, legL = size * 0.44f, barW = size * 0.24f;
    glPushMatrix();
    glTranslatef(-barW * 0.5f, 0.01f, 0.0f);
    glScalef(legW, 0.02f, legL);
    glutSolidCube(1.0);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(barW * 0.5f, 0.01f, 0.0f);
    glScalef(legW, 0.02f, legL);
    glutSolidCube(1.0);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0.0f, 0.01f, 0.0f);
    glScalef(barW, 0.02f, legW);
    glutSolidCube(1.0);
    glPopMatrix();

    // 4 Đèn viền xanh ngọc nhấp nháy góc bãi đáp
    bool hStrobe = std::fmod(animTime, 0.8f) < 0.4f;
    setMaterial(hStrobe ? 0.2f : 0.05f, hStrobe ? 0.95f : 0.3f, 0.85f, 80.0f, 0.9f);
    for (int hx = -1; hx <= 1; hx += 2) {
        for (int hz = -1; hz <= 1; hz += 2) {
            glPushMatrix();
            glTranslatef(hx * size * 0.41f, 0.06f, hz * size * 0.41f);
            glScalef(0.14f, 0.08f, 0.14f);
            glutSolidSphere(1.0, 8, 6);
            glPopMatrix();
        }
    }
    glPopMatrix();
}

/**
 * Vẽ Biển quảng cáo màn hình LED / Neon hiện đại trên mặt đứng tòa nhà
 */
void drawBuildingBillboard(float w, float h, float d, int style) {
    int seed = (int)(w * 7.0f + h * 13.0f + d * 17.0f + style);
    int variant = std::abs(seed) % 3;
    float boardW = std::min(w * 0.65f, 7.5f);
    float boardH = std::min(h * 0.20f, 4.2f);
    float boardY = -h * 0.10f;
    float boardZ = d * 0.5f + 0.12f;

    glPushMatrix();
    glTranslatef(0.0f, boardY, boardZ);

    // 1. Khung kim loại viền biển quảng cáo
    setMaterial(0.12f, 0.14f, 0.18f, 20.0f);
    drawCube(boardW + 0.35f, boardH + 0.35f, 0.18f);

    // 2. Màn hình LED / Neon phát sáng
    float pulse = 0.88f + 0.12f * std::sin(animTime * 3.5f + seed);
    float r = 0.10f, g = 0.82f, b = 0.95f; // Neon Cyan
    if (variant == 1) { r = 0.95f; g = 0.22f; b = 0.65f; } // Neon Magenta
    else if (variant == 2) { r = 0.98f; g = 0.70f; b = 0.15f; } // Amber Gold

    glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT);
    glDisable(GL_LIGHTING);

    // Nền màn hình LED
    glColor3f(r * pulse, g * pulse, b * pulse);
    glBegin(GL_QUADS);
        glVertex3f(-boardW * 0.5f, -boardH * 0.5f, 0.10f);
        glVertex3f( boardW * 0.5f, -boardH * 0.5f, 0.10f);
        glVertex3f( boardW * 0.5f,  boardH * 0.5f, 0.10f);
        glVertex3f(-boardW * 0.5f,  boardH * 0.5f, 0.10f);
    glEnd();

    // Dải họa tiết đồ họa LED (thanh equalizer bars động)
    glColor3f(1.0f, 1.0f, 1.0f);
    int bars = 5;
    float barSpacing = boardW / (float)(bars + 1);
    for (int i = 0; i < bars; ++i) {
        float barX = -boardW * 0.5f + barSpacing * (i + 1);
        float barH = (boardH * 0.32f) * (0.35f + 0.65f * std::sin(animTime * 4.2f + i * 1.3f));
        glBegin(GL_QUADS);
            glVertex3f(barX - 0.20f, -boardH * 0.35f, 0.11f);
            glVertex3f(barX + 0.20f, -boardH * 0.35f, 0.11f);
            glVertex3f(barX + 0.20f, -boardH * 0.35f + barH, 0.11f);
            glVertex3f(barX - 0.20f, -boardH * 0.35f + barH, 0.11f);
        glEnd();
    }

    glPopAttrib();
    glPopMatrix();
}

/**
 * Vẽ Tòa Nhà Procedural (Bảng màu kiến trúc thanh lịch lấy cảm hứng chính xác từ Hình 2)
 */
void drawBuilding(const Building& b) {
    const float buildingPalette[6][3] = {
        { 0.88f, 0.91f, 0.93f }, // Style 0: Trắng sứ / Kính xanh Azure
        { 0.88f, 0.68f, 0.32f }, // Style 1: Vàng đất Ochre / Amber ấm
        { 0.82f, 0.48f, 0.34f }, // Style 2: Terracotta / Lavender tím
        { 0.28f, 0.65f, 0.76f }, // Style 3: Tháp lăng kính Prism
        { 0.22f, 0.66f, 0.72f }, // Style 4: Mái lam ngọc cổ điển
        { 0.78f, 0.82f, 0.85f }  // Style 5: Bạc Corporate / Be ấm
    };

    int styleIdx = b.style % 6;
    float colR = buildingPalette[styleIdx][0];
    float colG = buildingPalette[styleIdx][1];
    float colB = buildingPalette[styleIdx][2];

    // Điều chỉnh màu chuyên biệt theo từng công trình trong hình mẫu:
    if (styleIdx == 0 && b.h > 76.0f) {
        // Tòa chọc trời kính Azure cao nhất
        colR = 0.20f; colG = 0.58f; colB = 0.78f;
    } else if (styleIdx == 1 && b.h < 25.0f) {
        // Shophouse vàng amber ấm ở mặt tiền
        colR = 0.92f; colG = 0.74f; colB = 0.36f;
    } else if (styleIdx == 2 && b.h < 20.0f) {
        // Shophouse tím lavender ở mặt tiền
        colR = 0.68f; colG = 0.62f; colB = 0.78f;
    } else if (styleIdx == 5 && b.h < 15.0f) {
        // Khối thương mại dịch vụ thấp tầng mặt tiền
        colR = 0.86f; colG = 0.82f; colB = 0.72f;
    }

    bool detailed = std::fabs(b.x - drone.x) < 95.0f && std::fabs(b.z - drone.z) < 145.0f;

    glPushMatrix();
    glTranslatef(b.x, b.h * 0.5f - 0.25f, b.z);
    setMaterial(colR, colG, colB, 14.0f, 0.15f);

    if (styleIdx == 3) {
        // THÁP VĂN PHÒNG LĂNG KÍNH (PRISM TOWER NHƯ HÌNH MẪU)
        drawCube(b.w, b.h, b.d);
        if (detailed) {
            drawCleanWindows(b.w, b.h, b.d, b.style);
        }
        // Đỉnh vát góc lăng kính hiện đại (Angled Faceted Crown)
        glPushMatrix();
        glTranslatef(0.0f, b.h * 0.5f, 0.0f);
        setMaterial(0.22f, 0.60f, 0.72f, 40.0f, 0.35f);
        float hw = b.w * 0.5f, hd = b.d * 0.5f, wedgeH = 4.8f;
        glBegin(GL_TRIANGLES);
            glNormal3f(0.0f, 0.6f, -0.8f);
            glVertex3f(-hw, 0.0f, -hd);
            glVertex3f( hw, 0.0f, -hd);
            glVertex3f(  0.0f, wedgeH, hd);

            glNormal3f(0.8f, 0.6f, 0.0f);
            glVertex3f( hw, 0.0f, -hd);
            glVertex3f( hw, 0.0f,  hd);
            glVertex3f(  0.0f, wedgeH, hd);

            glNormal3f(-0.8f, 0.6f, 0.0f);
            glVertex3f(-hw, 0.0f,  hd);
            glVertex3f(-hw, 0.0f, -hd);
            glVertex3f(  0.0f, wedgeH, hd);
        glEnd();
        glPopMatrix();

    } else if (styleIdx == 4) {
        // TÒA THÁP MÁI XANH NGỌC LAM CỔ ĐIỂN
        drawCube(b.w, b.h * 0.80f, b.d);
        if (detailed) {
            drawCleanWindows(b.w, b.h * 0.80f, b.d, b.style);
        }
        setMaterial(0.15f, 0.52f, 0.62f, 10.0f, 0.08f);
        glPushMatrix();
        glTranslatef(0.0f, b.h * 0.40f + 1.2f, 0.0f);
        drawCube(b.w * 0.85f, 2.4f, b.d * 0.85f);
        glTranslatef(0.0f, 1.2f, 0.0f);
        drawCone(std::min(b.w, b.d) * 0.40f, 4.5f, 20);
        glPopMatrix();

    } else if (styleIdx == 1) {
        // TÒA NHÀ VÀNG ĐẤT OCHRE / AMBER
        drawCube(b.w, b.h, b.d);
        if (detailed) {
            drawCleanWindows(b.w, b.h, b.d, b.style);
        }

        if (b.h > 50.0f) {
            // THÁP OCHRE CÓ CỘT CỜ TRÊN ĐỈNH (FLAGPOLE TOWER NHƯ HÌNH MẪU)
            setMaterial(0.75f, 0.55f, 0.22f, 20.0f);
            glPushMatrix();
            glTranslatef(0.0f, b.h * 0.5f + 0.35f, 0.0f);
            drawCube(b.w * 0.88f, 0.70f, b.d * 0.88f);
            glPopMatrix();

            // Cột cờ và lá cờ tung bay trong gió
            glPushMatrix();
            glTranslatef(0.0f, b.h * 0.5f + 0.7f, 0.0f);
            setMaterial(0.85f, 0.88f, 0.90f, 60.0f);
            drawCylinder(0.12f, 8.5f, 10);
            float flagWave = std::sin(animTime * 4.5f) * 0.28f;
            setMaterial(0.20f, 0.55f, 0.85f, 20.0f);
            glBegin(GL_TRIANGLES);
                glNormal3f(0.0f, 0.0f, 1.0f);
                glVertex3f(0.0f, 8.2f, 0.0f);
                glVertex3f(2.6f, 7.6f, flagWave);
                glVertex3f(0.0f, 7.0f, 0.0f);
            glEnd();
            glPopMatrix();

        } else {
            // TÒA BOUTIQUE VÀNG AMBER MẶT TIỀN (CÓ KHỐI PENTHOUSE VÒM & MÁI CHE)
            setMaterial(0.78f, 0.42f, 0.46f, 20.0f);
            glPushMatrix();
            glTranslatef(0.0f, b.h * 0.5f + 0.6f, 0.0f);
            glScalef(b.w * 0.32f, 1.2f, b.d * 0.22f);
            glutSolidSphere(1.0, 16, 12);
            glPopMatrix();

            // Mái che cửa hàng vươn ra lối đi bộ
            glPushMatrix();
            glTranslatef(0.0f, -b.h * 0.5f + 2.8f, b.d * 0.5f + 0.6f);
            setMaterial(0.22f, 0.50f, 0.75f, 15.0f);
            glBegin(GL_QUADS);
                glNormal3f(0.0f, 0.7f, 0.7f);
                glVertex3f(-b.w * 0.45f, 0.3f, -0.6f);
                glVertex3f( b.w * 0.45f, 0.3f, -0.6f);
                glVertex3f( b.w * 0.45f, -0.2f, 0.7f);
                glVertex3f(-b.w * 0.45f, -0.2f, 0.7f);
            glEnd();
            glPopMatrix();
        }

    } else if (styleIdx == 2) {
        // TÒA NHÀ TERRACOTTA / TÍM LAVENDER
        drawCube(b.w, b.h, b.d);
        if (detailed) {
            drawCleanWindows(b.w, b.h, b.d, b.style);
        }

        if (b.h < 20.0f) {
            // NHÀ PHỐ SHOPHOUSE TÍM LAVENDER (CÓ HÀNG CỘT SẢNH ĐÓN & KI-ỐT GÓC)
            glPushMatrix();
            glTranslatef(0.0f, -b.h * 0.5f, b.d * 0.5f + 0.45f);
            setMaterial(0.92f, 0.94f, 0.95f, 20.0f);
            glPushMatrix();
            glTranslatef(0.0f, 3.2f, 0.0f);
            drawCube(b.w * 0.55f, 0.50f, 1.0f);
            glPopMatrix();
            for (int col = -1; col <= 1; col += 2) {
                glPushMatrix();
                glTranslatef(col * b.w * 0.22f, 0.0f, 0.35f);
                drawCylinder(0.20f, 3.0f, 12);
                glPopMatrix();
            }
            glPopMatrix();

            // Ki-ốt phụ góc phố
            setMaterial(0.85f, 0.78f, 0.65f, 10.0f);
            glPushMatrix();
            glTranslatef(b.w * 0.42f + 1.2f, -b.h * 0.5f + 2.5f, 0.0f);
            drawCube(2.4f, 5.0f, b.d * 0.7f);
            glPopMatrix();

        } else {
            // TÒA NHÀ VĂN PHÒNG TERRACOTTA GỜ MÁI XÁM THAN
            setMaterial(0.22f, 0.25f, 0.28f, 15.0f, 0.1f);
            glPushMatrix();
            glTranslatef(0.0f, b.h * 0.5f + 0.25f, 0.0f);
            drawCube(b.w * 0.96f, 0.50f, b.d * 0.96f);
            glPopMatrix();
        }

    } else {
        // STYLE 0 & 5: SIÊU CAO ỐC KÍNH XANH AZURE HOẶC CAO ỐC BẠC CORPORATE
        drawCube(b.w, b.h, b.d);
        if (detailed) {
            drawCleanWindows(b.w, b.h, b.d, b.style);
        }

        float roofY = b.h * 0.5f + 0.02f;

        if (styleIdx == 0 && b.h > 76.0f) {
            // SIÊU CAO ỐC AZURE SPIRE SKYSCRAPER (TÒA CAO NHẤT CÓ KIM THU LÔI VÚT CAO)
            glPushMatrix();
            glTranslatef(0.0f, roofY, 0.0f);
            setMaterial(0.85f, 0.90f, 0.94f, 40.0f);
            drawCube(b.w * 0.65f, 2.0f, b.d * 0.65f);

            glTranslatef(0.0f, 1.0f, 0.0f);
            setMaterial(0.92f, 0.94f, 0.96f, 60.0f);
            drawCone(1.3f, 11.5f, 16);
            glTranslatef(0.0f, 11.5f, 0.0f);
            drawCylinder(0.12f, 3.5f, 8);

            bool spireBlink = std::fmod(animTime, 1.0f) < 0.5f;
            if (spireBlink) {
                setMaterial(1.0f, 0.15f, 0.15f, 100.0f, 1.0f);
                GLfloat glow[] = { 1.0f, 0.15f, 0.15f, 1.0f };
                glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, glow);
            } else {
                setMaterial(0.35f, 0.05f, 0.05f, 10.0f);
            }
            glTranslatef(0.0f, 3.5f, 0.0f);
            glScalef(0.30f, 0.30f, 0.30f);
            glutSolidSphere(1.0, 10, 8);
            glPopMatrix();

        } else if (styleIdx == 5 && b.h > 65.0f) {
            // CAO ỐC BẠC CORPORATE VỚI TẦNG ÁP MÁI LAM NGỌC & BÃI ĐÁP TRỰC THĂNG
            setMaterial(0.22f, 0.65f, 0.70f, 30.0f, 0.3f);
            glPushMatrix();
            glTranslatef(0.0f, roofY + 1.2f, 0.0f);
            drawCube(b.w * 0.85f, 2.4f, b.d * 0.85f);
            glPopMatrix();

            // Kiểm tra chính xác 2 tòa nhà Landmark Helipad
            bool isHelipad = (std::abs(b.x - HELIPAD_1_X) < 2.0f && std::abs(b.z - HELIPAD_1_Z) < 2.0f) ||
                             (std::abs(b.x - HELIPAD_2_X) < 2.0f && std::abs(b.z - HELIPAD_2_Z) < 2.0f);
            if (isHelipad) {
                glPushMatrix();
                glTranslatef(0.0f, roofY + 2.45f, 0.0f);
                drawRooftopHelipad(std::min(b.w, b.d) * 0.75f);
                glPopMatrix();
            }

        } else if (styleIdx == 5 && b.h < 15.0f) {
            // KHỐI THƯƠNG MẠI DỊCH VỤ THẤP TẦNG (CÓ MÁI ĐÓN KÍNH)
            setMaterial(0.20f, 0.50f, 0.70f, 30.0f);
            glPushMatrix();
            glTranslatef(0.0f, -b.h * 0.5f + 2.8f, b.d * 0.5f + 0.5f);
            drawCube(b.w * 0.65f, 0.20f, 1.2f);
            glPopMatrix();

        } else {
            // TÒA NHÀ TRẮNG HIỆN ĐẠI TIÊU CHUẨN
            setMaterial(0.95f, 0.96f, 0.98f, 50.0f);
            glPushMatrix();
            glTranslatef(0.0f, roofY, 0.0f);
            drawCube(b.w * 0.96f, 0.4f, b.d * 0.96f);
            glPopMatrix();
        }
    }

    // Đèn tín hiệu hàng không chớp đỏ cảnh báo trên nóc các tòa cao ốc
    if (b.h > 34.0f && styleIdx != 3) {
        glPushMatrix();
        glTranslatef(b.w * 0.22f, b.h * 0.5f + 0.2f, b.d * 0.22f);
        setMaterial(0.72f, 0.76f, 0.80f, 40.0f, 0.5f);
        drawCylinder(0.10f, 5.5f, 10);

        bool beaconBlink = std::fmod(animTime, 1.0f) < 0.5f;
        if (beaconBlink) {
            setMaterial(1.0f, 0.15f, 0.15f, 100.0f, 1.0f);
            GLfloat glow[] = { 1.0f, 0.15f, 0.15f, 1.0f };
            glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, glow);
        } else {
            setMaterial(0.35f, 0.05f, 0.05f, 10.0f, 0.1f);
        }
        glTranslatef(0.0f, 5.5f, 0.0f);
        glScalef(0.24f, 0.24f, 0.24f);
        glutSolidSphere(1.0, 10, 8);
        glPopMatrix();
    }

    // Các cụm thông gió màu trắng làm sân thượng dễ đọc ở góc nhìn cao.
    if (styleIdx != 3) {
        setMaterial(0.94f, 0.96f, 0.95f, 4.0f, 0.0f);
        for (int i = 0; i < 3; ++i) {
            float px = (i - 1) * b.w * 0.27f;
            float pz = (i % 2 == 0 ? -1.0f : 1.0f) * b.d * 0.24f;
            glPushMatrix();
            glTranslatef(px, b.h * 0.5f + 0.30f, pz);
            drawCube(b.w * 0.17f, 0.58f, b.d * 0.16f);
            glTranslatef(0.0f, 0.33f, 0.0f);
            setMaterial(0.67f, 0.72f, 0.73f, 3.0f, 0.0f);
            drawCube(b.w * 0.12f, 0.10f, b.d * 0.11f);
            glPopMatrix();
            setMaterial(0.94f, 0.96f, 0.95f, 4.0f, 0.0f);
        }
    }

    // Biển quảng cáo màn hình LED Neon cho các tòa nhà cao tầng
    if (b.h > 24.0f && (styleIdx == 1 || styleIdx == 2 || styleIdx == 5)) {
        drawBuildingBillboard(b.w, b.h, b.d, b.style);
    }

    glPopMatrix();
}

bool sphereAabbHit(const Vec3& p, float radius, const Building& b) {
    float halfX = b.w * 0.5f, halfZ = b.d * 0.5f;
    if (b.style == 3) halfX = halfZ = std::min(b.w, b.d) * 0.44f;
    float qx = clampf(p.x, b.x - halfX, b.x + halfX);
    float qy = clampf(p.y, -0.25f, b.h - 0.25f);
    float qz = clampf(p.z, b.z - halfZ, b.z + halfZ);
    Vec3 d = p - Vec3(qx, qy, qz);
    return dot(d, d) < radius * radius;
}

bool checkBuildingHitAndResolve(const Vec3& p, float radius, const Building& b, Vec3& outNormal, float& outDist) {
    float halfX = b.w * 0.5f, halfZ = b.d * 0.5f;
    if (b.style == 3) halfX = halfZ = std::min(b.w, b.d) * 0.44f;
    float qx = clampf(p.x, b.x - halfX, b.x + halfX);
    float qy = clampf(p.y, -0.25f, b.h - 0.25f);
    float qz = clampf(p.z, b.z - halfZ, b.z + halfZ);
    Vec3 d = p - Vec3(qx, qy, qz);
    float distSq = dot(d, d);
    float dist = std::sqrt(distSq);
    outDist = dist;
    if (distSq < radius * radius) {
        if (dist > 1e-4f) outNormal = d * (1.0f / dist);
        else outNormal = Vec3(0, 1, 0);
        return true;
    }
    return false;
}

// =========================================================================================
// 8. ĐƯỜNG PHỐ, THẢM CỎ & CÂY XANH ĐÔ THỊ (CHUẨN HÌNH 2)
// =========================================================================================
void drawPowerSpan(const Vec3& a, const Vec3& b, float y, float sag) {
    Vec3 start(a.x, y, a.z), end(b.x, y, b.z), delta = end - start;
    Vec3 cp[4] = {
        start,
        start + delta * (1.0f / 3.0f) + Vec3(0.0f, -sag, 0.0f),
        start + delta * (2.0f / 3.0f) + Vec3(0.0f, -sag, 0.0f),
        end
    };
    drawBezierCable(cp);
}

void drawUtilityPole(float x, float z, bool roadRunsAlongZ) {
    glPushMatrix();
    glTranslatef(x, -0.28f, z);

    // Chân cột đế vuông và thân trụ xám than hiện đại
    setMaterial(0.20f, 0.22f, 0.26f, 45.0f, 0.5f);
    glPushMatrix();
    glTranslatef(0.0f, 0.15f, 0.0f);
    drawCube(0.42f, 0.30f, 0.42f);
    glPopMatrix();
    drawCylinder(0.12f, 6.2f, 12);

    // Cần đèn chữ L vươn ra lòng đường
    glPushMatrix();
    glTranslatef(0.0f, 6.0f, 0.0f);
    if (roadRunsAlongZ) {
        glTranslatef(0.70f, 0.0f, 0.0f);
        drawCube(1.45f, 0.10f, 0.12f);
        glTranslatef(0.65f, -0.15f, 0.0f);
    } else {
        glTranslatef(0.0f, 0.0f, 0.70f);
        drawCube(0.12f, 0.10f, 1.45f);
        glTranslatef(0.0f, -0.15f, 0.65f);
    }

    // Chụp đèn LED hiện đại
    setMaterial(0.14f, 0.16f, 0.20f, 60.0f, 0.5f);
    glPushMatrix();
    glScalef(0.38f, 0.12f, 0.38f);
    glutSolidCube(1.0);
    glPopMatrix();

    // Mặt kính LED phát sáng
    setMaterial(1.0f, 0.95f, 0.65f, 5.0f, 0.0f);
    if (nightFactor > 0.02f) {
        GLfloat glow[] = { nightFactor * 1.0f, nightFactor * 0.85f, nightFactor * 0.45f, 1.0f };
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, glow);
    }
    glPushMatrix();
    glTranslatef(0.0f, -0.06f, 0.0f);
    glScalef(0.28f, 0.06f, 0.28f);
    glutSolidSphere(1.0, 10, 8);
    glPopMatrix();

    glPopMatrix();

    // Vệt sáng vàng ấm rọi mặt đường vào ban đêm
    if (nightFactor > 0.05f) {
        glPushMatrix();
        glTranslatef(roadRunsAlongZ ? 1.35f : 0.0f, 0.02f, roadRunsAlongZ ? 0.0f : 1.35f);
        glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_LIGHTING);
        glColor4f(1.0f, 0.88f, 0.45f, nightFactor * 0.20f);
        glBegin(GL_TRIANGLE_FAN);
            glVertex3f(0.0f, 0.0f, 0.0f);
            for (int i = 0; i <= 16; ++i) {
                float a = 2.0f * PI * i / 16.0f;
                glColor4f(1.0f, 0.82f, 0.35f, 0.0f);
                glVertex3f(std::cos(a) * 4.4f, 0.0f, std::sin(a) * 4.4f);
            }
        glEnd();
        glPopAttrib();
        glPopMatrix();
    }

    glPopMatrix();
}

/** Dựng cây xanh đô thị 3 phong cách (tán rộng đa tầng, tùng búp, cây anh đào phong) */
void drawStylizedTree(float x, float z, int variant) {
    glPushMatrix();
    glTranslatef(x, -0.28f, z);

    // Bồn cây viền đá bao quanh gốc cây
    setMaterial(0.60f, 0.63f, 0.66f, 15.0f, 0.1f);
    glPushMatrix();
    glTranslatef(0.0f, 0.08f, 0.0f);
    drawDuctRing(1.15f, 1.40f, 0.16f, 16);
    glPopMatrix();
    // Đất trồng màu nâu ấm
    setMaterial(0.28f, 0.18f, 0.10f, 5.0f, 0.0f);
    glPushMatrix();
    glTranslatef(0.0f, 0.06f, 0.0f);
    glScalef(1.15f, 0.05f, 1.15f);
    glutSolidCube(1.0);
    glPopMatrix();

    // Thân cây gỗ tự nhiên
    setMaterial(0.36f, 0.23f, 0.13f, 6.0f, 0.04f);
    drawCylinder(0.22f, 4.2f, 12);

    if (variant == 0) {
        // KIỂU 1: CÂY ĐÔ THỊ 3 TẦNG TÁN LÁ XANH MẠ TƯƠI MÁT
        setMaterial(0.18f, 0.58f, 0.16f, 8.0f, 0.05f);
        glPushMatrix();
        glTranslatef(0.0f, 4.0f, 0.0f);
        glScalef(1.85f, 1.25f, 1.85f);
        glutSolidSphere(1.0, 14, 10);
        glPopMatrix();

        setMaterial(0.28f, 0.70f, 0.22f, 10.0f, 0.08f);
        glPushMatrix();
        glTranslatef(0.35f, 4.95f, 0.22f);
        glScalef(1.45f, 1.15f, 1.45f);
        glutSolidSphere(1.0, 12, 9);
        glPopMatrix();

        setMaterial(0.38f, 0.82f, 0.26f, 12.0f, 0.10f);
        glPushMatrix();
        glTranslatef(-0.20f, 5.75f, -0.15f);
        glScalef(1.05f, 0.95f, 1.05f);
        glutSolidSphere(1.0, 10, 8);
        glPopMatrix();

    } else if (variant == 1) {
        // KIỂU 2: CÂY TÙNG BÚT / THÔNG CẢNH QUAN 3 TẦNG CHÓP
        for (int tier = 0; tier < 3; ++tier) {
            float ty = 2.8f + tier * 1.55f;
            float tr = 1.75f - tier * 0.42f;
            setMaterial(0.10f + tier * 0.05f, 0.42f + tier * 0.07f, 0.18f, 8.0f, 0.04f);
            glPushMatrix();
            glTranslatef(0.0f, ty, 0.0f);
            drawCone(tr, 2.3f, 14);
            glPopMatrix();
        }
    } else {
        // KIỂU 3: CÂY PHONG / HOA ANH ĐÀO SẮC CAM HỒNG ẤM ÁP
        setMaterial(0.92f, 0.48f, 0.35f, 10.0f, 0.08f);
        glPushMatrix();
        glTranslatef(0.0f, 4.1f, 0.0f);
        glScalef(1.75f, 1.30f, 1.75f);
        glutSolidSphere(1.0, 14, 10);
        glPopMatrix();

        setMaterial(0.98f, 0.66f, 0.46f, 12.0f, 0.10f);
        glPushMatrix();
        glTranslatef(0.30f, 5.05f, -0.20f);
        glScalef(1.30f, 1.10f, 1.30f);
        glutSolidSphere(1.0, 12, 9);
        glPopMatrix();
    }

    glPopMatrix();
}

// Trạng thái chu kỳ đèn tín hiệu giao thông (Tổng chu kỳ: 14 giây)
inline bool isTrafficGreenForZ(float t) {
    float cycle = std::fmod(t, 14.0f);
    return cycle < 5.8f;
}
inline bool isTrafficYellowForZ(float t) {
    float cycle = std::fmod(t, 14.0f);
    return cycle >= 5.8f && cycle < 7.0f;
}
inline bool isTrafficGreenForX(float t) {
    float cycle = std::fmod(t, 14.0f);
    return cycle >= 7.0f && cycle < 12.8f;
}
inline bool isTrafficYellowForX(float t) {
    float cycle = std::fmod(t, 14.0f);
    return cycle >= 12.8f;
}

/**
 * Vẽ Cột đèn giao thông hiện đại kiểu cần vươn (Cantilever mast arm) tại góc ngã tư
 * headingDir:
 *   0: Đón dòng xe đi hướng +Z (mặt đèn nhìn về -Z)
 *   1: Đón dòng xe đi hướng -Z (mặt đèn nhìn về +Z)
 *   2: Đón dòng xe đi hướng +X (mặt đèn nhìn về -X)
 *   3: Đón dòng xe đi hướng -X (mặt đèn nhìn về +X)
 */
void drawModernTrafficLight(float x, float z, int headingDir) {
    glPushMatrix();
    glTranslatef(x, -0.28f, z);

    float rotY = 0.0f;
    if (headingDir == 1) rotY = 180.0f;
    else if (headingDir == 2) rotY = 90.0f;
    else if (headingDir == 3) rotY = -90.0f;
    glRotatef(rotY, 0.0f, 1.0f, 0.0f);

    // 1. Thân cột thép xám than chịu lực
    setMaterial(0.14f, 0.16f, 0.19f, 60.0f, 0.6f);
    drawCylinder(0.12f, 5.2f, 10);

    // Cần vươn ngang ra mép làn xe
    glPushMatrix();
    glTranslatef(-0.6f, 4.8f, 0.0f);
    glRotatef(90.0f, 0.0f, 0.0f, 1.0f);
    drawCylinder(0.08f, 1.2f, 8);
    glPopMatrix();

    // 2. Hộp đèn giao thông (Signal Head)
    glPushMatrix();
    glTranslatef(-1.15f, 4.4f, 0.0f);
    setMaterial(0.10f, 0.11f, 0.13f, 50.0f, 0.2f);
    glScalef(0.32f, 1.25f, 0.38f);
    glutSolidCube(1.0);
    glPopMatrix();

    // Xác định màu đèn theo chu kỳ
    bool isGreen = false, isYellow = false, isRed = false;
    if (headingDir == 0 || headingDir == 1) {
        isGreen = isTrafficGreenForZ(animTime);
        isYellow = isTrafficYellowForZ(animTime);
        isRed = !isGreen && !isYellow;
    } else {
        isGreen = isTrafficGreenForX(animTime);
        isYellow = isTrafficYellowForX(animTime);
        isRed = !isGreen && !isYellow;
    }

    // 3. Ba bóng đèn: Đỏ (trên), Vàng (giữa), Xanh (dưới)
    float lightZ = -0.19f;

    // Đèn đỏ (trên)
    if (isRed) {
        setMaterial(1.0f, 0.08f, 0.08f, 100.0f, 1.0f);
        GLfloat glow[] = { 1.0f, 0.08f, 0.08f, 1.0f };
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, glow);
    } else {
        setMaterial(0.20f, 0.04f, 0.04f, 20.0f, 0.0f);
    }
    glPushMatrix();
    glTranslatef(-1.15f, 4.78f, lightZ);
    glScalef(0.10f, 0.10f, 0.06f);
    glutSolidSphere(1.0, 10, 8);
    glPopMatrix();

    // Đèn vàng (giữa)
    if (isYellow) {
        setMaterial(1.0f, 0.85f, 0.05f, 100.0f, 1.0f);
        GLfloat glow[] = { 1.0f, 0.85f, 0.05f, 1.0f };
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, glow);
    } else {
        setMaterial(0.20f, 0.18f, 0.03f, 20.0f, 0.0f);
    }
    glPushMatrix();
    glTranslatef(-1.15f, 4.40f, lightZ);
    glScalef(0.10f, 0.10f, 0.06f);
    glutSolidSphere(1.0, 10, 8);
    glPopMatrix();

    // Đèn xanh (dưới)
    if (isGreen) {
        setMaterial(0.05f, 1.0f, 0.20f, 100.0f, 1.0f);
        GLfloat glow[] = { 0.05f, 1.0f, 0.20f, 1.0f };
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, glow);
    } else {
        setMaterial(0.03f, 0.20f, 0.06f, 20.0f, 0.0f);
    }
    glPushMatrix();
    glTranslatef(-1.15f, 4.02f, lightZ);
    glScalef(0.10f, 0.10f, 0.06f);
    glutSolidSphere(1.0, 10, 8);
    glPopMatrix();

    glPopMatrix();
}

/**
 * Vẽ Trạm chờ xe buýt hiện đại với mái kính cường lực và hộp đèn lộ trình
 */
void drawBusShelter(float x, float z, bool faceZ) {
    glPushMatrix();
    glTranslatef(x, -0.28f, z);
    if (!faceZ) glRotatef(90.0f, 0.0f, 1.0f, 0.0f);

    // 1. Đế bê tông vỉa hè
    setMaterial(0.70f, 0.72f, 0.75f, 10.0f);
    glPushMatrix();
    glTranslatef(0.0f, 0.05f, 0.0f);
    drawCube(5.4f, 0.10f, 2.2f);
    glPopMatrix();

    // 2. Cột thép chịu lực màu xám than
    setMaterial(0.18f, 0.20f, 0.24f, 30.0f);
    for (int sx = -1; sx <= 1; sx += 2) {
        glPushMatrix();
        glTranslatef(sx * 2.2f, 1.4f, -0.85f);
        drawCylinder(0.08f, 2.7f, 10);
        glPopMatrix();
    }

    // 3. Mái che kính vòm cong hiện đại
    setMaterial(0.18f, 0.20f, 0.24f, 30.0f);
    glPushMatrix();
    glTranslatef(0.0f, 2.75f, -0.1f);
    drawCube(5.6f, 0.12f, 2.4f);
    glPopMatrix();

    // 4. Vách kính cường lực trong suốt phía sau
    glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glDisable(GL_LIGHTING);
    glColor4f(0.35f, 0.65f, 0.85f, 0.38f);
    glBegin(GL_QUADS);
        glVertex3f(-2.1f, 0.15f, -0.85f);
        glVertex3f( 2.1f, 0.15f, -0.85f);
        glVertex3f( 2.1f, 2.65f, -0.85f);
        glVertex3f(-2.1f, 2.65f, -0.85f);
    glEnd();
    glPopAttrib();

    // 5. Ghế chờ cho hành khách
    setMaterial(0.55f, 0.35f, 0.20f, 5.0f);
    glPushMatrix();
    glTranslatef(0.0f, 0.50f, -0.50f);
    drawCube(3.4f, 0.10f, 0.60f);
    glPopMatrix();
    // Chân ghế
    setMaterial(0.15f, 0.15f, 0.18f, 20.0f);
    for (int leg = -1; leg <= 1; leg += 2) {
        glPushMatrix();
        glTranslatef(leg * 1.3f, 0.25f, -0.50f);
        drawCube(0.12f, 0.45f, 0.45f);
        glPopMatrix();
    }

    // 6. Biển thông tin tuyến xe buýt phát sáng
    glPushMatrix();
    glTranslatef(2.35f, 1.4f, -0.85f);
    setMaterial(0.15f, 0.18f, 0.22f, 20.0f);
    drawCube(0.12f, 1.8f, 0.95f);
    // Hộp đèn LED thông tin
    float busPulse = 0.9f + 0.1f * std::sin(animTime * 2.0f);
    glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT);
    glDisable(GL_LIGHTING);
    glColor3f(0.95f * busPulse, 0.85f * busPulse, 0.25f * busPulse);
    glBegin(GL_QUADS);
        glVertex3f(-0.07f, -0.7f, -0.4f);
        glVertex3f(-0.07f,  0.7f, -0.4f);
        glVertex3f(-0.07f,  0.7f,  0.4f);
        glVertex3f(-0.07f, -0.7f,  0.4f);
    glEnd();
    glPopAttrib();
    glPopMatrix();

    glPopMatrix();
}

void drawStreetProps(const Vec3& center) {
    const float avenue = CITY_GRID_SPACING * 6.0f;
    int gx0 = std::max(CITY_MIN_ROAD_INDEX, (int)std::floor((center.x - 175) / CITY_GRID_SPACING));
    int gx1 = std::min(CITY_MAX_ROAD_INDEX, (int)std::floor((center.x + 175) / CITY_GRID_SPACING));
    int gz0 = std::max(CITY_MIN_ROAD_INDEX, (int)std::floor((center.z - 175) / CITY_GRID_SPACING));
    int gz1 = std::min(CITY_MAX_ROAD_INDEX, (int)std::floor((center.z + 175) / CITY_GRID_SPACING));

    for (int gx = gx0; gx <= gx1; gx++) {
        if (!isStreetIndex(gx)) continue;
        float roadX = gx * CITY_GRID_SPACING;
        int seg0 = std::max(CITY_MIN_DISTRICT, (int)std::floor((center.z - 190.0f) / avenue) - 1);
        int seg1 = std::min(CITY_MAX_DISTRICT, (int)std::floor((center.z + 190.0f) / avenue) + 1);
        for (int seg = seg0; seg <= seg1; ++seg) {
            float zBase = seg * avenue;
            for (int side = -1; side <= 1; side += 2) {
                if ((gx == CITY_MIN_ROAD_INDEX && side < 0) || (gx == CITY_MAX_ROAD_INDEX && side > 0)) continue;
                float treeX = roadX + side * 11.5f;
                drawUtilityPole(roadX + side * 9.6f, zBase + 26.25f, true);
                drawUtilityPole(roadX + side * 9.6f, zBase + 78.75f, true);
                drawStylizedTree(treeX, zBase + 52.5f, std::abs(seg * 3 + gx + side) % 3);
            }

            // Trạm chờ xe buýt hiện đại bên lề đại lộ
            if (gx % 2 == 0) {
                drawBusShelter(roadX + 11.8f, zBase + 45.0f, true);
            }
        }
    }

    for (int gz = gz0; gz <= gz1; gz++) {
        if (!isStreetIndex(gz)) continue;
        float roadZ = gz * CITY_GRID_SPACING;
        int seg0 = std::max(CITY_MIN_DISTRICT, (int)std::floor((center.x - 190.0f) / avenue) - 1);
        int seg1 = std::min(CITY_MAX_DISTRICT, (int)std::floor((center.x + 190.0f) / avenue) + 1);
        for (int seg = seg0; seg <= seg1; ++seg) {
            float xBase = seg * avenue;
            for (int side = -1; side <= 1; side += 2) {
                if ((gz == CITY_MIN_ROAD_INDEX && side < 0) || (gz == CITY_MAX_ROAD_INDEX && side > 0)) continue;
                float treeZ = roadZ + side * 11.5f;
                drawUtilityPole(xBase + 26.25f, roadZ + side * 9.6f, false);
                drawUtilityPole(xBase + 78.75f, roadZ + side * 9.6f, false);
                drawStylizedTree(xBase + 52.5f, treeZ, std::abs(seg * 3 + gz + side) % 3);
            }
        }
    }

    // CỘT ĐÈN GIAO THÔNG ĐẦY ĐỦ CHO TẤT CẢ CÁC HƯỚNG TẠI MỖI NGÃ TƯ
    int ix0 = std::max(CITY_MIN_DISTRICT, (int)std::floor((center.x - 190.0f) / avenue) - 1);
    int ix1 = std::min(CITY_MAX_DISTRICT + 1, (int)std::floor((center.x + 190.0f) / avenue) + 1);
    int iz0 = std::max(CITY_MIN_DISTRICT, (int)std::floor((center.z - 190.0f) / avenue) - 1);
    int iz1 = std::min(CITY_MAX_DISTRICT + 1, (int)std::floor((center.z + 190.0f) / avenue) + 1);

    for (int ix = ix0; ix <= ix1; ++ix) {
        for (int iz = iz0; iz <= iz1; ++iz) {
            float x = ix * avenue, z = iz * avenue;
            // 4 Cột đèn tại 4 hướng tiếp cận ngã tư, đặt trên vỉa hè ngay sát vạch dừng xe:
            if (iz > CITY_MIN_DISTRICT)
                drawModernTrafficLight(x + 9.2f, z - 13.5f, 0); // Nhánh Nam: đón xe đi +Z
            if (iz <= CITY_MAX_DISTRICT)
                drawModernTrafficLight(x - 9.2f, z + 13.5f, 1); // Nhánh Bắc: đón xe đi -Z
            if (ix > CITY_MIN_DISTRICT)
                drawModernTrafficLight(x - 13.5f, z + 9.2f, 2); // Nhánh Tây: đón xe đi +X
            if (ix <= CITY_MAX_DISTRICT)
                drawModernTrafficLight(x + 13.5f, z - 9.2f, 3); // Nhánh Đông: đón xe đi -X
        }
    }
}

// Mũi tên chỉ hướng làn xe màu trắng muốt trên mặt đường
void drawRoadArrow(float x, float z, bool alongZ, float dir) {
    glPushMatrix();
    glTranslatef(x, -0.32f, z);
    if (alongZ) {
        if (dir < 0.0f) glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
    } else {
        glRotatef(dir > 0.0f ? 90.0f : -90.0f, 0.0f, 1.0f, 0.0f);
    }

    // Thân mũi tên (Shaft)
    glBegin(GL_QUADS);
        glNormal3f(0.0f, 1.0f, 0.0f);
        glVertex3f(-0.20f, 0.0f, -1.6f);
        glVertex3f( 0.20f, 0.0f, -1.6f);
        glVertex3f( 0.20f, 0.0f,  0.5f);
        glVertex3f(-0.20f, 0.0f,  0.5f);
    glEnd();

    // Đầu mũi tên nhọn (Arrowhead)
    glBegin(GL_TRIANGLES);
        glNormal3f(0.0f, 1.0f, 0.0f);
        glVertex3f( 0.0f,  0.0f,  1.9f);
        glVertex3f(-0.75f, 0.0f,  0.4f);
        glVertex3f( 0.75f, 0.0f,  0.4f);
    glEnd();
    glPopMatrix();
}

// Vạch sang đường người đi bộ (Zebra Crossing) qua tuyến đường chạy dọc trục Z
// Các vạch sọc chạy song song với hướng xe chạy (theo trục Z) chuẩn mực quốc tế
void drawCrosswalkZRoad(float x, float z, float side) {
    float cz = z + side * 10.9f;
    const float stripeW = 0.50f;
    const float halfL = 1.9f;
    for (int s = -6; s <= 6; ++s) {
        float sx = x + s * 1.02f;
        glBegin(GL_QUADS);
            glNormal3f(0.0f, 1.0f, 0.0f);
            glVertex3f(sx - stripeW * 0.5f, -0.32f, cz + halfL);
            glVertex3f(sx + stripeW * 0.5f, -0.32f, cz + halfL);
            glVertex3f(sx + stripeW * 0.5f, -0.32f, cz - halfL);
            glVertex3f(sx - stripeW * 0.5f, -0.32f, cz - halfL);
        glEnd();
    }
}

// Vạch sang đường người đi bộ (Zebra Crossing) qua tuyến đường chạy ngang trục X
// Các vạch sọc chạy song song với hướng xe chạy (theo trục X) chuẩn mực quốc tế
void drawCrosswalkXRoad(float x, float z, float side) {
    float cx = x + side * 10.9f;
    const float stripeW = 0.50f;
    const float halfL = 1.9f;
    for (int s = -6; s <= 6; ++s) {
        float sz = z + s * 1.02f;
        glBegin(GL_QUADS);
            glNormal3f(0.0f, 1.0f, 0.0f);
            glVertex3f(cx - halfL, -0.32f, sz + stripeW * 0.5f);
            glVertex3f(cx + halfL, -0.32f, sz + stripeW * 0.5f);
            glVertex3f(cx + halfL, -0.32f, sz - stripeW * 0.5f);
            glVertex3f(cx - halfL, -0.32f, sz - stripeW * 0.5f);
        glEnd();
    }
}

/**
 * Mặt Đường Nhựa Xám Tím Sẫm & Thảm Cỏ Xanh Tươi Mát (Chuẩn xác như Hình 2)
 */
void drawRoadAndBlocks(const Vec3& center) {
    (void)center;
    // Mặt nền phủ luôn phần rộng của các tuyến đường biên.
    const float minX = -CITY_EDGE - ROAD_HALF_WIDTH, maxX = CITY_EDGE + ROAD_HALF_WIDTH;
    const float minZ = -CITY_EDGE - ROAD_HALF_WIDTH, maxZ = CITY_EDGE + ROAD_HALF_WIDTH;

    // 1. Thảm cỏ xanh non rực rỡ bao bọc toàn bộ nền phố.
    setMaterial(0.30f, 0.64f, 0.24f, 8.0f, 0.02f);
    glBegin(GL_QUADS);
        glNormal3f(0.0f, 1.0f, 0.0f);
        glVertex3f(minX, -0.40f, maxZ);
        glVertex3f(maxX, -0.40f, maxZ);
        glVertex3f(maxX, -0.40f, minZ);
        glVertex3f(minX, -0.40f, minZ);
    glEnd();

    int gx0 = CITY_MIN_ROAD_INDEX, gx1 = CITY_MAX_ROAD_INDEX;
    int gz0 = CITY_MIN_ROAD_INDEX, gz1 = CITY_MAX_ROAD_INDEX;

    // Vỉa hè xám sáng sạch sẽ, có viền gờ dừng trước giao lộ.
    setMaterial(0.72f, 0.76f, 0.78f, 15.0f, 0.08f);
    const float avenueSpacing = CITY_GRID_SPACING * 6.0f;
    for (int gx = gx0; gx <= gx1; gx++) {
        if (!isStreetIndex(gx)) continue;
        float x = gx * CITY_GRID_SPACING;
        for (int side = -1; side <= 1; side += 2) {
            if ((gx == CITY_MIN_ROAD_INDEX && side < 0) || (gx == CITY_MAX_ROAD_INDEX && side > 0)) continue;
            float edge = x + side * 8.55f;
            int seg0 = CITY_MIN_DISTRICT, seg1 = CITY_MAX_DISTRICT;
            for (int seg = seg0; seg <= seg1; ++seg) {
                float zA = std::max(minZ, seg * avenueSpacing + 8.0f);
                float zB = std::min(maxZ, (seg + 1) * avenueSpacing - 8.0f);
                if (zA >= zB) continue;
                glBegin(GL_QUADS);
                    glNormal3f(0.0f, 1.0f, 0.0f);
                    glVertex3f(edge - 0.45f, -0.29f, zB);
                    glVertex3f(edge + 0.45f, -0.29f, zB);
                    glVertex3f(edge + 0.45f, -0.29f, zA);
                    glVertex3f(edge - 0.45f, -0.29f, zA);
                glEnd();
            }
        }
    }
    for (int gz = gz0; gz <= gz1; gz++) {
        if (!isStreetIndex(gz)) continue;
        float z = gz * CITY_GRID_SPACING;
        for (int side = -1; side <= 1; side += 2) {
            if ((gz == CITY_MIN_ROAD_INDEX && side < 0) || (gz == CITY_MAX_ROAD_INDEX && side > 0)) continue;
            float edge = z + side * 8.55f;
            int seg0 = CITY_MIN_DISTRICT, seg1 = CITY_MAX_DISTRICT;
            for (int seg = seg0; seg <= seg1; ++seg) {
                float xA = std::max(minX, seg * avenueSpacing + 8.0f);
                float xB = std::min(maxX, (seg + 1) * avenueSpacing - 8.0f);
                if (xA >= xB) continue;
                glBegin(GL_QUADS);
                    glNormal3f(0.0f, 1.0f, 0.0f);
                    glVertex3f(xA, -0.29f, edge + 0.45f);
                    glVertex3f(xB, -0.29f, edge + 0.45f);
                    glVertex3f(xB, -0.29f, edge - 0.45f);
                    glVertex3f(xA, -0.29f, edge - 0.45f);
                glEnd();
            }
        }
    }

    // 2. Đại lộ xám than như mô hình minh họa, rộng và nổi bật trên nền cỏ.
    setMaterial(0.16f, 0.17f, 0.19f, 3.0f, 0.0f);
    for (int gx = gx0; gx <= gx1; gx++) {
        if (isStreetIndex(gx)) {
            float x = gx * CITY_GRID_SPACING;
            glBegin(GL_QUADS);
                glNormal3f(0.0f, 1.0f, 0.0f);
                glVertex3f(x - 8.0f, -0.34f, maxZ);
                glVertex3f(x + 8.0f, -0.34f, maxZ);
                glVertex3f(x + 8.0f, -0.34f, minZ);
                glVertex3f(x - 8.0f, -0.34f, minZ);
            glEnd();
        }
    }
    for (int gz = gz0; gz <= gz1; gz++) {
        if (isStreetIndex(gz)) {
            float z = gz * CITY_GRID_SPACING;
            glBegin(GL_QUADS);
                glNormal3f(0.0f, 1.0f, 0.0f);
                glVertex3f(minX, -0.34f, z + 8.0f);
                glVertex3f(maxX, -0.34f, z + 8.0f);
                glVertex3f(maxX, -0.34f, z - 8.0f);
                glVertex3f(minX, -0.34f, z - 8.0f);
            glEnd();
        }
    }

    // 3. VẠCH KẺ ĐƯỜNG ĐÔ THỊ CHUẨN XÁC, HỢP LÝ (REALISTIC URBAN ROAD MARKINGS)
    setMaterial(0.96f, 0.97f, 0.98f, 20.0f, 0.0f);

    // A. VẠCH TIM ĐƯỜNG & VẠCH BIÊN DỌC CÁC ĐẠI LỘ
    // 1. Tuyến đường chạy dọc trục Z (Avenues along Z)
    for (int gx = gx0; gx <= gx1; gx++) {
        if (!isStreetIndex(gx)) continue;
        float x = gx * CITY_GRID_SPACING;

        for (int seg = CITY_MIN_DISTRICT; seg < CITY_MAX_DISTRICT + 1; ++seg) {
            float z0 = seg * avenueSpacing;
            float z1 = (seg + 1) * avenueSpacing;
            float segMid = (z0 + z1) * 0.5f;

            // a. Vạch liền tim đường trước 2 đầu ngã tư (No-Overtaking Solid Lines)
            glBegin(GL_QUADS);
                glNormal3f(0.0f, 1.0f, 0.0f);
                glVertex3f(x - 0.12f, -0.32f, z0 + 29.0f);
                glVertex3f(x + 0.12f, -0.32f, z0 + 29.0f);
                glVertex3f(x + 0.12f, -0.32f, z0 + 14.2f);
                glVertex3f(x - 0.12f, -0.32f, z0 + 14.2f);

                glVertex3f(x - 0.12f, -0.32f, z1 - 14.2f);
                glVertex3f(x + 0.12f, -0.32f, z1 - 14.2f);
                glVertex3f(x + 0.12f, -0.32f, z1 - 29.0f);
                glVertex3f(x - 0.12f, -0.32f, z1 - 29.0f);
            glEnd();

            // b. Vạch đứt quãng tim đường ở giữa đoạn phố (Dashed Center Lines)
            for (int d = -4; d < 4; ++d) {
                float dashCenter = segMid + (d + 0.5f) * 5.6f;
                glBegin(GL_QUADS);
                    glNormal3f(0.0f, 1.0f, 0.0f);
                    glVertex3f(x - 0.12f, -0.32f, dashCenter + 1.5f);
                    glVertex3f(x + 0.12f, -0.32f, dashCenter + 1.5f);
                    glVertex3f(x + 0.12f, -0.32f, dashCenter - 1.5f);
                    glVertex3f(x - 0.12f, -0.32f, dashCenter - 1.5f);
                glEnd();
            }

            // c. Vạch biên mép đường (Solid White Shoulder Lines) sát vỉa hè hai bên
            for (int side = -1; side <= 1; side += 2) {
                float edgeX = x + side * 7.15f;
                glBegin(GL_QUADS);
                    glNormal3f(0.0f, 1.0f, 0.0f);
                    glVertex3f(edgeX - 0.08f, -0.32f, z1 - 12.8f);
                    glVertex3f(edgeX + 0.08f, -0.32f, z1 - 12.8f);
                    glVertex3f(edgeX + 0.08f, -0.32f, z0 + 12.8f);
                    glVertex3f(edgeX - 0.08f, -0.32f, z0 + 12.8f);
                glEnd();
            }
        }
    }

    // 2. Tuyến đường chạy ngang trục X (Avenues along X)
    for (int gz = gz0; gz <= gz1; gz++) {
        if (!isStreetIndex(gz)) continue;
        float z = gz * CITY_GRID_SPACING;

        for (int seg = CITY_MIN_DISTRICT; seg < CITY_MAX_DISTRICT + 1; ++seg) {
            float x0 = seg * avenueSpacing;
            float x1 = (seg + 1) * avenueSpacing;
            float segMid = (x0 + x1) * 0.5f;

            // a. Vạch liền tim đường trước 2 đầu ngã tư (No-Overtaking Solid Lines)
            glBegin(GL_QUADS);
                glNormal3f(0.0f, 1.0f, 0.0f);
                glVertex3f(x0 + 29.0f, -0.32f, z - 0.12f);
                glVertex3f(x0 + 29.0f, -0.32f, z + 0.12f);
                glVertex3f(x0 + 14.2f, -0.32f, z + 0.12f);
                glVertex3f(x0 + 14.2f, -0.32f, z - 0.12f);

                glVertex3f(x1 - 14.2f, -0.32f, z - 0.12f);
                glVertex3f(x1 - 14.2f, -0.32f, z + 0.12f);
                glVertex3f(x1 - 29.0f, -0.32f, z + 0.12f);
                glVertex3f(x1 - 29.0f, -0.32f, z - 0.12f);
            glEnd();

            // b. Vạch đứt quãng tim đường ở giữa đoạn phố
            for (int d = -4; d < 4; ++d) {
                float dashCenter = segMid + (d + 0.5f) * 5.6f;
                glBegin(GL_QUADS);
                    glNormal3f(0.0f, 1.0f, 0.0f);
                    glVertex3f(dashCenter + 1.5f, -0.32f, z - 0.12f);
                    glVertex3f(dashCenter + 1.5f, -0.32f, z + 0.12f);
                    glVertex3f(dashCenter - 1.5f, -0.32f, z + 0.12f);
                    glVertex3f(dashCenter - 1.5f, -0.32f, z - 0.12f);
                glEnd();
            }

            // c. Vạch biên mép đường (Solid White Shoulder Lines) sát vỉa hè hai bên
            for (int side = -1; side <= 1; side += 2) {
                float edgeZ = z + side * 7.15f;
                glBegin(GL_QUADS);
                    glNormal3f(0.0f, 1.0f, 0.0f);
                    glVertex3f(x1 - 12.8f, -0.32f, edgeZ - 0.08f);
                    glVertex3f(x1 - 12.8f, -0.32f, edgeZ + 0.08f);
                    glVertex3f(x0 + 12.8f, -0.32f, edgeZ + 0.08f);
                    glVertex3f(x0 + 12.8f, -0.32f, edgeZ - 0.08f);
                glEnd();
            }
        }
    }

    // B. CÁC GIAO LỘ NGÃ TƯ: VẠCH QUA ĐƯỜNG, VẠCH DỪNG XE, MŨI TÊN CHỈ HƯỚNG
    for (int ix = -2; ix <= 2; ++ix) {
        for (int iz = -2; iz <= 2; ++iz) {
            float x = ix * avenueSpacing, z = iz * avenueSpacing;

            // Nhánh Bắc/Nam (Đường chạy dọc trục Z qua ngã tư)
            // Nhánh -Z (side = -1): xe đi hướng +Z về ngã tư
            if (iz > -2) {
                drawCrosswalkZRoad(x, z, -1.0f);
                // Vạch dừng xe (Stop Line) ở làn xe chạy tới (bên phải tim đường: x > 0)
                glBegin(GL_QUADS);
                    glNormal3f(0.0f, 1.0f, 0.0f);
                    glVertex3f(x + 0.18f, -0.32f, z - 13.95f);
                    glVertex3f(x + 7.15f, -0.32f, z - 13.95f);
                    glVertex3f(x + 7.15f, -0.32f, z - 14.45f);
                    glVertex3f(x + 0.18f, -0.32f, z - 14.45f);
                glEnd();
                // Mũi tên chỉ hướng trên mặt đường làn vào
                drawRoadArrow(x + 3.6f, z - 21.0f, true, 1.0f);
            }
            // Nhánh +Z (side = +1): xe đi hướng -Z về ngã tư
            if (iz < 2) {
                drawCrosswalkZRoad(x, z, 1.0f);
                // Vạch dừng xe ở làn xe chạy tới (bên phải tim đường: x < 0)
                glBegin(GL_QUADS);
                    glNormal3f(0.0f, 1.0f, 0.0f);
                    glVertex3f(x - 7.15f, -0.32f, z + 14.45f);
                    glVertex3f(x - 0.18f, -0.32f, z + 14.45f);
                    glVertex3f(x - 0.18f, -0.32f, z + 13.95f);
                    glVertex3f(x - 7.15f, -0.32f, z + 13.95f);
                glEnd();
                // Mũi tên chỉ hướng trên mặt đường làn vào
                drawRoadArrow(x - 3.6f, z + 21.0f, true, -1.0f);
            }

            // Nhánh Tây/Đông (Đường chạy ngang trục X qua ngã tư)
            // Nhánh -X (side = -1): xe đi hướng +X về ngã tư
            if (ix > -2) {
                drawCrosswalkXRoad(x, z, -1.0f);
                // Vạch dừng xe ở làn xe chạy tới (bên phải tim đường: z > 0)
                glBegin(GL_QUADS);
                    glNormal3f(0.0f, 1.0f, 0.0f);
                    glVertex3f(x - 13.95f, -0.32f, z + 0.18f);
                    glVertex3f(x - 13.95f, -0.32f, z + 7.15f);
                    glVertex3f(x - 14.45f, -0.32f, z + 7.15f);
                    glVertex3f(x - 14.45f, -0.32f, z + 0.18f);
                glEnd();
                // Mũi tên chỉ hướng trên mặt đường làn vào
                drawRoadArrow(x - 21.0f, z + 3.6f, false, 1.0f);
            }
            // Nhánh +X (side = +1): xe đi hướng -X về ngã tư
            if (ix < 2) {
                drawCrosswalkXRoad(x, z, 1.0f);
                // Vạch dừng xe ở làn xe chạy tới (bên phải tim đường: z < 0)
                glBegin(GL_QUADS);
                    glNormal3f(0.0f, 1.0f, 0.0f);
                    glVertex3f(x + 14.45f, -0.32f, z - 7.15f);
                    glVertex3f(x + 14.45f, -0.32f, z - 0.18f);
                    glVertex3f(x + 13.95f, -0.32f, z - 0.18f);
                    glVertex3f(x + 13.95f, -0.32f, z - 7.15f);
                glEnd();
                // Mũi tên chỉ hướng trên mặt đường làn vào
                drawRoadArrow(x + 21.0f, z - 3.6f, false, -1.0f);
            }
        }
    }

}

// Khởi tạo dàn xe cộ đô thị trang nhã (như Hình 2: xe van trắng, taxi vàng, xe con thanh lịch)
void initCityCars() {
    cityCars.clear();
    Color4 carColors[] = {
        Color4(0.95f, 0.96f, 0.98f), // Xe trắng muốt (như chiếc xe van trắng trong hình 2)
        Color4(0.95f, 0.75f, 0.15f), // Taxi vàng (như chiếc xe vàng trong hình 2)
        Color4(0.85f, 0.30f, 0.20f), // Xe đỏ gạch thanh lịch
        Color4(0.20f, 0.50f, 0.80f), // Xe xanh dương
        Color4(0.90f, 0.92f, 0.95f)  // Xe bạc
    };

    for (int i = -2; i <= 2; i++) {
        float avenueX = i * (CITY_GRID_SPACING * 6.0f);
        cityCars.push_back({ avenueX + 2.5f, -100.0f + i * 40.0f, 20.0f, 1.0f, carColors[std::abs(i * 3) % 5], false });
        cityCars.push_back({ avenueX - 2.5f,  100.0f - i * 40.0f, 18.0f, -1.0f, carColors[std::abs(i * 2 + 1) % 5], false });
    }
    for (int i = -2; i <= 2; i++) {
        float avenueZ = i * (CITY_GRID_SPACING * 6.0f);
        cityCars.push_back({ -100.0f + i * 35.0f, avenueZ + 2.5f, 19.0f, 1.0f, carColors[std::abs(i * 4 + 2) % 5], true });
        cityCars.push_back({  100.0f - i * 35.0f, avenueZ - 2.5f, 21.0f, -1.0f, carColors[std::abs(i * 5 + 3) % 5], true });
    }
}

void updateCityCars(float dt) {
    float limit = CITY_EDGE - 15.0f;
    bool zGreen = isTrafficGreenForZ(animTime);
    bool xGreen = isTrafficGreenForX(animTime);

    for (auto& c : cityCars) {
        bool shouldStop = false;

        if (c.runsAlongX) {
            // Xe chạy theo trục X
            float nextIntersectionX = 0.0f;
            if (c.dir > 0.0f) {
                // Hướng +X: tìm ngã tư gần nhất phía trước
                nextIntersectionX = std::ceil((c.x + 0.1f) / 105.0f) * 105.0f;
                float stopLineX = nextIntersectionX - 14.2f;
                float distToStop = stopLineX - c.x;
                // Nếu đèn đỏ/vàng cho trục X và xe đang áp sát vạch dừng (khoảng cách 0.5m -> 20m)
                if (!xGreen && distToStop > 0.3f && distToStop < 20.0f) {
                    shouldStop = true;
                }
            } else {
                // Hướng -X
                nextIntersectionX = std::floor((c.x - 0.1f) / 105.0f) * 105.0f;
                float stopLineX = nextIntersectionX + 14.2f;
                float distToStop = c.x - stopLineX;
                if (!xGreen && distToStop > 0.3f && distToStop < 20.0f) {
                    shouldStop = true;
                }
            }
        } else {
            // Xe chạy theo trục Z
            float nextIntersectionZ = 0.0f;
            if (c.dir > 0.0f) {
                // Hướng +Z: tìm ngã tư gần nhất phía trước
                nextIntersectionZ = std::ceil((c.z + 0.1f) / 105.0f) * 105.0f;
                float stopLineZ = nextIntersectionZ - 14.2f;
                float distToStop = stopLineZ - c.z;
                if (!zGreen && distToStop > 0.3f && distToStop < 20.0f) {
                    shouldStop = true;
                }
            } else {
                // Hướng -Z
                nextIntersectionZ = std::floor((c.z - 0.1f) / 105.0f) * 105.0f;
                float stopLineZ = nextIntersectionZ + 14.2f;
                float distToStop = c.z - stopLineZ;
                if (!zGreen && distToStop > 0.3f && distToStop < 20.0f) {
                    shouldStop = true;
                }
            }
        }

        // Giữ khoảng cách an toàn với xe cùng làn phía trước (không tông đuôi xe khác)
        for (const auto& other : cityCars) {
            if (&other == &c) continue;
            if (other.runsAlongX != c.runsAlongX) continue;
            if (other.dir != c.dir) continue;

            if (c.runsAlongX) {
                if (std::fabs(c.z - other.z) < 1.2f) {
                    float gap = (other.x - c.x) * c.dir;
                    if (gap > 0.2f && gap < 8.5f) {
                        shouldStop = true;
                    }
                }
            } else {
                if (std::fabs(c.x - other.x) < 1.2f) {
                    float gap = (other.z - c.z) * c.dir;
                    if (gap > 0.2f && gap < 8.5f) {
                        shouldStop = true;
                    }
                }
            }
        }

        // Tăng / giảm tốc êm ái
        float targetSpeed = shouldStop ? 0.0f : c.speed;
        if (c.currentSpeed > targetSpeed) {
            c.currentSpeed = std::max(targetSpeed, c.currentSpeed - 24.0f * dt); // Phanh hãm
        } else if (c.currentSpeed < targetSpeed) {
            c.currentSpeed = std::min(targetSpeed, c.currentSpeed + 16.0f * dt); // Đạp ga tăng tốc
        }

        // Cập nhật vị trí
        if (c.runsAlongX) {
            c.x += c.dir * c.currentSpeed * dt;
            if (c.x > limit)  c.x = -limit;
            if (c.x < -limit) c.x = limit;
        } else {
            c.z += c.dir * c.currentSpeed * dt;
            if (c.z > limit)  c.z = -limit;
            if (c.z < -limit) c.z = limit;
        }
    }
}

void drawCityCar(const CityCar& c) {
    glPushMatrix();
    glTranslatef(c.x, -0.34f, c.z);
    if (c.runsAlongX) {
        glRotatef(c.dir > 0 ? 90.0f : -90.0f, 0.0f, 1.0f, 0.0f);
    } else {
        glRotatef(c.dir > 0 ? 0.0f : 180.0f, 0.0f, 1.0f, 0.0f);
    }

    // 0. Bóng đổ mềm gầm xe (Car drop shadow)
    glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_LIGHTING);
    glDepthMask(GL_FALSE);
    glColor4f(0.04f, 0.05f, 0.07f, 0.40f);
    glBegin(GL_QUADS);
        glVertex3f(-1.15f, 0.035f, -2.15f);
        glVertex3f( 1.15f, 0.035f, -2.15f);
        glVertex3f( 1.15f, 0.035f,  2.15f);
        glVertex3f(-1.15f, 0.035f,  2.15f);
    glEnd();
    glPopAttrib();

    // Thân xe
    setMaterial(c.color.r, c.color.g, c.color.b, 5.0f, 0.0f);
    glPushMatrix();
    glTranslatef(0.0f, 0.60f, 0.0f);
    glScalef(1.8f, 0.65f, 3.8f);
    glutSolidCube(1.0);
    glPopMatrix();

    // Bốn bánh đen nhỏ, đặt sát mặt đường để xe không bị lơ lửng.
    setMaterial(0.10f, 0.11f, 0.13f, 3.0f, 0.0f);
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sz = -1; sz <= 1; sz += 2) {
            glPushMatrix();
            glTranslatef(sx * 0.82f, 0.30f, sz * 1.08f);
            glScalef(0.30f, 0.30f, 0.48f);
            glutSolidSphere(1.0, 10, 8);
            glPopMatrix();
        }
    }

    // Cabin kính xanh xám
    setMaterial(0.76f, 0.91f, 0.97f, 4.0f, 0.0f);
    glPushMatrix();
    glTranslatef(0.0f, 1.00f, -0.2f);
    glScalef(1.5f, 0.50f, 2.0f);
    glutSolidCube(1.0);
    glPopMatrix();

    // Đèn pha trước phát sáng ban đêm
    setMaterial(1.0f, 0.95f, 0.75f, 80.0f);
    if (nightFactor > 0.05f) {
        GLfloat glow[] = { nightFactor * 1.0f, nightFactor * 0.95f, nightFactor * 0.75f, 1.0f };
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, glow);
    }
    glPushMatrix();
    glTranslatef(0.6f, 0.60f, 1.92f);
    glScalef(0.35f, 0.20f, 0.10f);
    glutSolidCube(1.0);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(-0.6f, 0.60f, 1.92f);
    glScalef(0.35f, 0.20f, 0.10f);
    glutSolidCube(1.0);
    glPopMatrix();

    // Đèn hậu & đèn phanh đỏ: sáng rực khi xe giảm tốc/dừng chờ đèn đỏ
    bool isBraking = c.currentSpeed < c.speed * 0.75f;
    if (isBraking) {
        setMaterial(1.0f, 0.05f, 0.05f, 100.0f, 1.0f);
        GLfloat brakeGlow[] = { 1.0f, 0.08f, 0.08f, 1.0f };
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, brakeGlow);
    } else {
        setMaterial(0.95f, 0.15f, 0.15f, 80.0f);
        if (nightFactor > 0.05f) {
            GLfloat glow[] = { nightFactor * 1.0f, nightFactor * 0.1f, nightFactor * 0.1f, 1.0f };
            glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, glow);
        }
    }
    glPushMatrix();
    glTranslatef(0.6f, 0.60f, -1.92f);
    glScalef(0.35f, 0.20f, 0.10f);
    glutSolidCube(1.0);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(-0.6f, 0.60f, -1.92f);
    glScalef(0.35f, 0.20f, 0.10f);
    glutSolidCube(1.0);
    glPopMatrix();

    glPopMatrix();
}

void drawGroundRect(float x, float z, float w, float d, float y) {
    glBegin(GL_QUADS);
    glNormal3f(0.0f, 1.0f, 0.0f);
    // Thứ tự đỉnh giữ mặt trên hướng +Y khi bật GL_CULL_FACE.
    glVertex3f(x - w * 0.5f, y, z + d * 0.5f);
    glVertex3f(x + w * 0.5f, y, z + d * 0.5f);
    glVertex3f(x + w * 0.5f, y, z - d * 0.5f);
    glVertex3f(x - w * 0.5f, y, z - d * 0.5f);
    glEnd();
}

// Vẽ một mảng đất hình ellipse bằng tam giác quạt, giữ pháp tuyến hướng lên.
void drawEllipseGround(float x, float z, float radiusX, float radiusZ, float y) {
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(x, y, z);
    for (int i = 0; i <= 64; ++i) {
        float angle = -2.0f * PI * i / 64.0f;
        glVertex3f(x + std::cos(angle) * radiusX, y, z + std::sin(angle) * radiusZ);
    }
    glEnd();
}

void drawRingLayer(float x, float z, float outerW, float outerD,
                   float innerW, float innerD, float y, float h,
                   float r, float g, float b) {
    setMaterial(r, g, b, 12.0f, 0.12f);
    float bandX = (outerW - innerW) * 0.5f;
    float bandZ = (outerD - innerD) * 0.5f;
    glPushMatrix(); glTranslatef(x, y, z - (outerD + innerD) * 0.25f); drawCube(outerW, h, bandZ); glPopMatrix();
    glPushMatrix(); glTranslatef(x, y, z + (outerD + innerD) * 0.25f); drawCube(outerW, h, bandZ); glPopMatrix();
    glPushMatrix(); glTranslatef(x - (outerW + innerW) * 0.25f, y, z); drawCube(bandX, h, innerD); glPopMatrix();
    glPushMatrix(); glTranslatef(x + (outerW + innerW) * 0.25f, y, z); drawCube(bandX, h, innerD); glPopMatrix();
}

void drawPark(float x, float z) {
    // Công viên ở riêng một ô phố, gồm lối dạo, hàng cây, ghế nghỉ và đài phun nước.
    setMaterial(0.25f, 0.60f, 0.28f, 4.0f, 0.0f);
    drawGroundRect(x, z, 76.0f, 72.0f, -0.20f);
    setMaterial(0.72f, 0.76f, 0.70f, 4.0f, 0.0f);
    glPushMatrix(); glTranslatef(x, -0.13f, z); drawCube(3.2f, 0.12f, 72.0f); glPopMatrix();
    glPushMatrix(); glTranslatef(x, -0.12f, z); drawCube(76.0f, 0.12f, 3.2f); glPopMatrix();
    // --- ĐÀI PHUN NƯỚC NGHỆ THUẬT CÓ HOẠT ẢNH (ANIMATED CASCADING FOUNTAIN) ---
    // 1. Hồ chứa nước tầng 1 (Base stone basin)
    setMaterial(0.82f, 0.84f, 0.88f, 20.0f);
    glPushMatrix();
    glTranslatef(x, -0.08f, z);
    drawCylinder(5.4f, 0.65f, 24);
    glPopMatrix();

    // Mặt nước tầng 1
    glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    setMaterial(0.20f, 0.65f, 0.82f, 60.0f, 0.5f);
    glPushMatrix();
    glTranslatef(x, 0.42f, z);
    drawCylinder(4.9f, 0.08f, 24);
    glPopMatrix();

    // 2. Trụ và tháp đĩa tầng 2 (Upper tier pedestal & bowl)
    setMaterial(0.85f, 0.87f, 0.90f, 25.0f);
    glPushMatrix();
    glTranslatef(x, 0.42f, z);
    drawCylinder(1.3f, 1.25f, 16);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(x, 1.60f, z);
    drawCylinder(2.6f, 0.38f, 20);
    glPopMatrix();

    // 3. Cột nước phun trung tâm (Central pulsing water jet)
    float jetH = 2.4f + 0.7f * std::sin(animTime * 5.0f);
    float jetWarp = 0.08f * std::cos(animTime * 8.0f);

    // Đèn LED chiếu sáng đài phun nước ban đêm (Color-cycling LED)
    float ledR = 0.3f + 0.5f * std::sin(animTime * 1.5f);
    float ledG = 0.5f + 0.4f * std::sin(animTime * 1.5f + 2.0f);
    float ledB = 0.7f + 0.3f * std::sin(animTime * 1.5f + 4.0f);
    if (timeOfDay < 6.0f || timeOfDay > 18.0f) {
        glColor4f(ledR, ledG, ledB, 0.85f);
    } else {
        glColor4f(0.85f, 0.95f, 1.0f, 0.75f);
    }
    glDisable(GL_LIGHTING);

    // Cột nước chính vươn cao
    glPushMatrix();
    glTranslatef(x + jetWarp, 1.95f, z);
    glScalef(0.42f, jetH, 0.42f);
    glutSolidSphere(1.0, 12, 10);
    glPopMatrix();

    // Đầu tia nước văng
    glPushMatrix();
    glTranslatef(x, 1.95f + jetH * 0.95f, z);
    glScalef(0.65f, 0.35f, 0.65f);
    glutSolidSphere(1.0, 10, 8);
    glPopMatrix();

    // 4. 6 tia nước uốn cong rơi từ tầng trên xuống tầng dưới
    for (int k = 0; k < 6; ++k) {
        float ang = k * (2.0f * 3.14159265f / 6.0f) + animTime * 0.4f;
        float sprayR = 2.2f + 1.6f * std::fmod(animTime * 1.2f + (float)k * 0.2f, 1.0f);
        float sprayY = 1.85f - 1.4f * std::pow(sprayR / 3.8f, 2.0f);
        glPushMatrix();
        glTranslatef(x + std::cos(ang) * sprayR, sprayY, z + std::sin(ang) * sprayR);
        glScalef(0.20f, 0.20f, 0.20f);
        glutSolidSphere(1.0, 6, 6);
        glPopMatrix();
    }
    glPopAttrib();

    const float treePos[8][2] = {
        {-29,-25},{-10,-28},{11,-28},{30,-24}, {-29,25},{-10,29},{12,28},{30,24}
    };
    for (int i = 0; i < 8; ++i) drawStylizedTree(x + treePos[i][0], z + treePos[i][1], i % 2);

    // Ghế gỗ nhỏ dọc lối đi trung tâm.
    setMaterial(0.42f, 0.25f, 0.13f, 5.0f, 0.0f);
    for (int side = -1; side <= 1; side += 2) {
        glPushMatrix(); glTranslatef(x + side * 9.0f, 0.20f, z + 11.0f); drawCube(4.2f, 0.22f, 0.70f); glPopMatrix();
        for (int leg = -1; leg <= 1; leg += 2) {
            glPushMatrix(); glTranslatef(x + side * 9.0f + leg * 1.45f, -0.04f, z + 11.0f); drawCube(0.16f, 0.50f, 0.18f); glPopMatrix();
        }
    }
}

void drawShoppingMall(float x, float z) {
    // Trung tâm thương mại nhiều tầng kính, sân trước và mái vòm Bezier trên giếng trời.
    Building mall = { x, z, 46.0f, 38.0f, 17.0f, 2 };
    setMaterial(0.87f, 0.93f, 0.95f, 14.0f, 0.15f);
    glPushMatrix(); glTranslatef(x, 0.0f, z); drawCube(49.0f, 0.55f, 41.0f); glPopMatrix();
    drawBuilding(mall);

    setMaterial(0.10f, 0.56f, 0.72f, 45.0f, 0.35f);
    for (int i = -2; i <= 2; ++i) {
        glPushMatrix(); glTranslatef(x + i * 7.8f, 5.0f, z - 19.25f); drawCube(5.7f, 7.2f, 0.20f); glPopMatrix();
    }
    setMaterial(0.97f, 0.97f, 0.91f, 20.0f, 0.15f);
    glPushMatrix(); glTranslatef(x, 1.2f, z - 21.2f); drawCube(19.0f, 0.38f, 5.5f); glPopMatrix();
    drawBezierDome(x, 17.2f, z);
}

void drawSchool(float x, float z) {
    // Khuôn viên trường học cố định với khối lớp học, sân thể thao và cột cờ.
    setMaterial(0.34f, 0.64f, 0.30f, 4.0f, 0.0f);
    drawGroundRect(x, z, 76.0f, 72.0f, -0.20f);
    setMaterial(0.74f, 0.77f, 0.74f, 5.0f, 0.0f);
    drawGroundRect(x + 3.0f, z + 22.0f, 52.0f, 3.2f, -0.10f);
    drawGroundRect(x - 20.0f, z - 1.0f, 3.2f, 42.0f, -0.10f);

    setMaterial(0.90f, 0.84f, 0.63f, 10.0f, 0.06f);
    glPushMatrix(); glTranslatef(x, 4.8f, z - 5.0f); drawCube(34.0f, 10.0f, 24.0f); glPopMatrix();
    setMaterial(0.18f, 0.48f, 0.72f, 12.0f, 0.05f);
    glPushMatrix(); glTranslatef(x, 9.9f, z - 5.0f); drawCube(36.0f, 0.55f, 26.0f); glPopMatrix();

    setMaterial(0.22f, 0.60f, 0.82f, 32.0f, 0.18f);
    for (int col = -2; col <= 2; ++col) {
        for (int row = 0; row < 2; ++row) {
            float wx = x + col * 6.0f;
            float wy = 3.3f + row * 3.0f;
            glPushMatrix(); glTranslatef(wx, wy, z - 17.15f); drawCube(3.7f, 1.7f, 0.18f); glPopMatrix();
            glPushMatrix(); glTranslatef(wx, wy, z + 7.15f); drawCube(3.7f, 1.7f, 0.18f); glPopMatrix();
        }
    }

    setMaterial(0.84f, 0.91f, 0.94f, 10.0f, 0.05f);
    glPushMatrix(); glTranslatef(x + 23.0f, 3.0f, z - 6.0f); drawCube(15.0f, 6.0f, 20.0f); glPopMatrix();
    setMaterial(0.86f, 0.22f, 0.16f, 8.0f, 0.0f);
    glPushMatrix(); glTranslatef(x - 25.0f, 2.6f, z - 13.0f); drawCylinder(0.12f, 10.0f, 8); glPopMatrix();
    glPushMatrix(); glTranslatef(x - 23.0f, 9.0f, z - 13.0f); drawCube(3.8f, 1.7f, 0.12f); glPopMatrix();

    // Sân bóng rổ nhỏ trong sân trường.
    setMaterial(0.83f, 0.46f, 0.28f, 5.0f, 0.0f);
    drawGroundRect(x, z + 29.0f, 28.0f, 13.0f, -0.08f);
    setMaterial(0.94f, 0.94f, 0.87f, 5.0f, 0.0f);
    drawGroundRect(x, z + 22.6f, 0.18f, 12.0f, -0.05f);
    drawGroundRect(x, z + 29.0f, 26.0f, 0.18f, -0.05f);
}

void drawHospital(float x, float z) {
    // Bệnh viện gồm tháp điều trị trắng xanh, khối cấp cứu thấp và biểu tượng chữ thập đỏ.
    setMaterial(0.72f, 0.78f, 0.76f, 4.0f, 0.0f);
    drawGroundRect(x, z, 70.0f, 66.0f, -0.20f);
    Building tower = { x - 4.0f, z + 1.0f, 30.0f, 28.0f, 27.0f, 0 };
    drawBuilding(tower);

    setMaterial(0.91f, 0.95f, 0.96f, 12.0f, 0.08f);
    glPushMatrix(); glTranslatef(x + 20.0f, 4.8f, z - 2.0f); drawCube(20.0f, 9.5f, 27.0f); glPopMatrix();
    setMaterial(0.15f, 0.54f, 0.73f, 30.0f, 0.2f);
    glPushMatrix(); glTranslatef(x + 20.0f, 9.65f, z - 2.0f); drawCube(21.0f, 0.35f, 28.0f); glPopMatrix();

    // Biển hiệu chữ thập ở mặt tiền phía Nam.
    setMaterial(0.97f, 0.98f, 0.96f, 15.0f, 0.1f);
    glPushMatrix(); glTranslatef(x - 4.0f, 14.0f, z - 14.25f); drawCube(5.4f, 5.4f, 0.55f); glPopMatrix();
    setMaterial(0.88f, 0.13f, 0.16f, 12.0f, 0.0f);
    glPushMatrix(); glTranslatef(x - 4.0f, 14.0f, z - 14.62f); drawCube(1.2f, 4.0f, 0.16f); glPopMatrix();
    glPushMatrix(); glTranslatef(x - 4.0f, 14.0f, z - 14.62f); drawCube(4.0f, 1.2f, 0.16f); glPopMatrix();

    setMaterial(0.78f, 0.82f, 0.82f, 6.0f, 0.0f);
    glPushMatrix(); glTranslatef(x + 1.0f, 0.0f, z + 19.0f); drawCube(3.0f, 0.18f, 22.0f); glPopMatrix();
}

void drawStadium(float x, float z) {
    // Sân vận động mở mái: các tầng khán đài tạo thành vành bao quanh sân cỏ ở giữa.
    setMaterial(0.19f, 0.49f, 0.22f, 5.0f, 0.0f);
    drawGroundRect(x, z, 39.0f, 24.0f, -0.10f);
    setMaterial(0.95f, 0.95f, 0.88f, 8.0f, 0.0f);
    for (int i = -3; i <= 3; ++i) drawGroundRect(x + i * 5.0f, z, 0.18f, 23.0f, -0.06f);
    drawGroundRect(x, z, 38.0f, 0.20f, -0.05f);
    glPushMatrix(); glTranslatef(x, 0.0f, z); glRotatef(90.0f, 1.0f, 0.0f, 0.0f); glutSolidTorus(0.10, 2.2, 8, 24); glPopMatrix();

    drawRingLayer(x, z, 66.0f, 52.0f, 39.0f, 24.0f, 2.3f, 4.6f, 0.77f, 0.82f, 0.84f);
    drawRingLayer(x, z, 61.0f, 47.0f, 39.0f, 24.0f, 6.7f, 4.0f, 0.20f, 0.50f, 0.70f);
    drawRingLayer(x, z, 66.0f, 52.0f, 39.0f, 24.0f, 10.0f, 0.65f, 0.95f, 0.96f, 0.92f);

    setMaterial(0.91f, 0.93f, 0.94f, 18.0f, 0.1f);
    for (int sx = -1; sx <= 1; sx += 2) for (int sz = -1; sz <= 1; sz += 2) {
        glPushMatrix(); glTranslatef(x + sx * 29.5f, 5.6f, z + sz * 22.0f); drawCylinder(0.40f, 8.7f, 12); glPopMatrix();
    }
}

Building amenityCollider(int districtX, int districtZ, int amenity) {
    float x = (districtX + 0.5f) * CITY_GRID_SPACING * 6.0f;
    float z = (districtZ + 0.5f) * CITY_GRID_SPACING * 6.0f;
    if (amenity == AMENITY_MALL) return Building{ x, z, 46.0f, 38.0f, 17.0f, 0 };
    if (amenity == AMENITY_SCHOOL) return Building{ x + 6.0f, z - 5.0f, 48.0f, 30.0f, 11.0f, 0 };
    if (amenity == AMENITY_HOSPITAL) return Building{ x + 5.0f, z, 52.0f, 31.0f, 27.0f, 0 };
    return Building{ x, z, 66.0f, 52.0f, 10.5f, 0 };
}

bool sphereStadiumRingHit(const Vec3& p, float radius, const Building& b) {
    if (p.y + radius < -0.2f || p.y - radius > b.h) return false;
    float dx = std::fabs(p.x - b.x), dz = std::fabs(p.z - b.z);
    bool insideOuter = dx < b.w * 0.5f + radius && dz < b.d * 0.5f + radius;
    bool outsideField = dx > 19.5f - radius || dz > 12.0f - radius;
    return insideOuter && outsideField;
}

void drawCommercialPodium(int dx, int dz) {
    float districtSize = CITY_GRID_SPACING * 6.0f;
    float x0 = dx * districtSize;
    float z0 = dz * districtSize;

    // Khối đế thương mại liên hoàn nằm ở khu vực phía Đông - Trung tâm của ô phố (kết nối các tháp cao ốc)
    float px = x0 + 68.0f;
    float pz = z0 + 58.0f;
    float pw = 48.0f;
    float pd = 42.0f;
    float ph = 7.2f;

    glPushMatrix();
    glTranslatef(px, ph * 0.5f - 0.28f, pz);

    // 1. Thân khối đế thương mại màu đá trắng sứ hiện đại
    setMaterial(0.88f, 0.90f, 0.92f, 15.0f, 0.1f);
    drawCube(pw, ph, pd);

    // 2. Dải kính mặt tiền tầng trệt và tầng 2 (Retail Curtain Glass)
    setMaterial(0.18f, 0.55f, 0.72f, 40.0f, 0.35f);
    // Kính mặt Nam nhìn ra quảng trường
    glPushMatrix();
    glTranslatef(0.0f, -0.6f, -pd * 0.5f - 0.05f);
    drawCube(pw * 0.88f, ph * 0.68f, 0.12f);
    glPopMatrix();
    // Kính mặt Đông nhìn ra đại lộ
    glPushMatrix();
    glTranslatef(pw * 0.5f + 0.05f, -0.6f, 0.0f);
    drawCube(0.12f, ph * 0.68f, pd * 0.82f);
    glPopMatrix();

    // 3. Sàn sân thượng (Rooftop Deck Terrace) màu xám sẫm
    setMaterial(0.32f, 0.36f, 0.40f, 10.0f, 0.05f);
    glPushMatrix();
    glTranslatef(0.0f, ph * 0.5f + 0.15f, 0.0f);
    drawCube(pw * 0.98f, 0.30f, pd * 0.98f);
    glPopMatrix();

    // 4. Lan can an toàn bao quanh sân thượng (Safety Balustrade)
    setMaterial(0.92f, 0.94f, 0.96f, 30.0f, 0.3f);
    float railH = 0.85f;
    float topY = ph * 0.5f + 0.30f + railH * 0.5f;
    // Lan can cạnh Nam
    glPushMatrix();
    glTranslatef(0.0f, topY, -pd * 0.48f);
    drawCube(pw * 0.94f, railH, 0.15f);
    glPopMatrix();
    // Lan can cạnh Đông
    glPushMatrix();
    glTranslatef(pw * 0.48f, topY, 0.0f);
    drawCube(0.15f, railH, pd * 0.94f);
    glPopMatrix();

    glPopMatrix();

    // 5. Cây xanh cảnh quan và ghế nghỉ trên sân thượng khối đế
    drawStylizedTree(px + 12.0f, pz - 10.0f, 0);
    drawStylizedTree(px - 14.0f, pz + 8.0f, 1);
}

void drawDistrictPlaza(int dx, int dz) {
    float districtSize = CITY_GRID_SPACING * 6.0f;
    float x0 = dx * districtSize;
    float z0 = dz * districtSize;

    // Sân quảng trường đi bộ lát đá phía trước các shophouse (South Frontage Plaza)
    float plazaX = x0 + 52.5f;
    float plazaZ = z0 + 17.5f;
    float plazaW = 86.0f;
    float plazaD = 18.0f;

    setMaterial(0.72f, 0.74f, 0.76f, 8.0f, 0.02f);
    glPushMatrix();
    glTranslatef(plazaX, -0.28f, plazaZ);
    glBegin(GL_QUADS);
        glNormal3f(0.0f, 1.0f, 0.0f);
        glVertex3f(-plazaW * 0.5f, 0.0f,  plazaD * 0.5f);
        glVertex3f( plazaW * 0.5f, 0.0f,  plazaD * 0.5f);
        glVertex3f( plazaW * 0.5f, 0.0f, -plazaD * 0.5f);
        glVertex3f(-plazaW * 0.5f, 0.0f, -plazaD * 0.5f);
    glEnd();
    glPopMatrix();

    // Hàng cây xanh bóng mát và ghế ngồi trong quảng trường
    float treeOffsets[3] = { -28.0f, 0.0f, 28.0f };
    for (int t = 0; t < 3; ++t) {
        drawStylizedTree(plazaX + treeOffsets[t], z0 + 10.5f, t % 3);
        // Ghế gỗ công viên cạnh bồn cây
        setMaterial(0.42f, 0.25f, 0.13f, 5.0f, 0.0f);
        glPushMatrix();
        glTranslatef(plazaX + treeOffsets[t] + 4.2f, 0.22f, z0 + 10.5f);
        drawCube(3.2f, 0.24f, 0.75f);
        glPopMatrix();
    }
}

void drawDistrictAmenities(const Vec3& center) {
    const float districtSize = CITY_GRID_SPACING * 6.0f;
    int dx0 = std::max(CITY_MIN_DISTRICT, (int)std::floor((center.x - 250.0f) / districtSize));
    int dx1 = std::min(CITY_MAX_DISTRICT, (int)std::floor((center.x + 250.0f) / districtSize));
    int dz0 = std::max(CITY_MIN_DISTRICT, (int)std::floor((center.z - 250.0f) / districtSize));
    int dz1 = std::min(CITY_MAX_DISTRICT, (int)std::floor((center.z + 250.0f) / districtSize));
    for (int dx = dx0; dx <= dx1; ++dx) for (int dz = dz0; dz <= dz1; ++dz) {
        int type = districtAmenity(dx, dz);
        float x = (dx + 0.5f) * districtSize, z = (dz + 0.5f) * districtSize;
        float ox = x - center.x, oz = z - center.z;
        if (ox * ox + oz * oz > 285.0f * 285.0f) continue;

        if (type == AMENITY_CITY) {
            drawCommercialPodium(dx, dz);
            drawDistrictPlaza(dx, dz);
        } else if (type == AMENITY_PARK) {
            drawPark(x, z);
        } else if (type == AMENITY_MALL) {
            drawShoppingMall(x, z);
        } else if (type == AMENITY_STADIUM) {
            drawStadium(x, z);
        } else if (type == AMENITY_SCHOOL) {
            drawSchool(x, z);
        } else if (type == AMENITY_HOSPITAL) {
            drawHospital(x, z);
        }
    }
}

/**
 * Vẽ Cột Tuabin gió phong điện (Wind Turbine) trên sườn núi
 */
void drawSingleWindTurbine(float x, float y, float z, float bladeAngle) {
    glPushMatrix();
    glTranslatef(x, y, z);

    // 1. Chân đế bê tông vững chắc
    setMaterial(0.65f, 0.68f, 0.72f, 15.0f);
    drawCylinder(2.2f, 1.5f, 16);

    // 2. Tháp tuabin hình trụ côn trắng muốt vút cao 38m
    setMaterial(0.95f, 0.96f, 0.98f, 30.0f, 0.4f);
    glPushMatrix();
    glTranslatef(0.0f, 1.5f, 0.0f);
    drawCylinder(1.2f, 36.0f, 16);
    glPopMatrix();

    // 3. Cabin phát điện (Nacelle) trên đỉnh tháp
    glPushMatrix();
    glTranslatef(0.0f, 37.5f, 0.0f);
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f); // Mũi tuabin quay về phía thành phố (hướng Đông)

    setMaterial(0.92f, 0.93f, 0.95f, 40.0f, 0.3f);
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, -0.6f);
    glScalef(1.4f, 1.5f, 4.2f);
    glutSolidCube(1.0);
    glPopMatrix();

    // Đèn LED đỏ cảnh báo hàng không chớp nháy trên nóc nacelle
    bool beacon = std::fmod(animTime, 1.2f) < 0.6f;
    if (beacon) {
        setMaterial(1.0f, 0.1f, 0.1f, 80.0f, 1.0f);
        GLfloat glow[] = { 1.0f, 0.1f, 0.1f, 1.0f };
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, glow);
    } else {
        setMaterial(0.3f, 0.05f, 0.05f, 10.0f, 0.0f);
    }
    glPushMatrix();
    glTranslatef(0.0f, 1.0f, -0.6f);
    glScalef(0.22f, 0.22f, 0.22f);
    glutSolidSphere(1.0, 8, 6);
    glPopMatrix();

    // 4. Trục xoay mũi nhọn (Spinner)
    setMaterial(0.96f, 0.97f, 0.99f, 50.0f, 0.5f);
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, 1.6f);
    drawCone(1.1f, 1.8f, 16);

    // 5. Ba cánh quạt khí động học khổng lồ (Rotor Blades)
    glRotatef(bladeAngle, 0.0f, 0.0f, 1.0f);
    for (int b = 0; b < 3; ++b) {
        glPushMatrix();
        glRotatef(b * 120.0f, 0.0f, 0.0f, 1.0f);
        glTranslatef(0.0f, 7.5f, 0.1f);
        glScalef(0.65f, 15.0f, 0.14f);
        glutSolidCube(1.0);
        glPopMatrix();
    }
    glPopMatrix();

    glPopMatrix();
    glPopMatrix();
}

void drawWindTurbines() {
    float bladeRot = animTime * 85.0f;
    drawSingleWindTurbine(-248.0f, 12.0f, -180.0f, bladeRot);
    drawSingleWindTurbine(-252.0f, 18.0f,  -60.0f, bladeRot + 45.0f);
    drawSingleWindTurbine(-250.0f, 16.0f,   60.0f, bladeRot + 90.0f);
    drawSingleWindTurbine(-248.0f, 14.0f,  180.0f, bladeRot + 135.0f);
}

/**
 * Vẽ Khinh khí cầu ngắm cảnh bồng bềnh trên bầu trời (Scenic Hot Air Balloon)
 */
void drawHotAirBalloon() {
    float bx = 120.0f + std::cos(animTime * 0.06f) * 45.0f;
    float bz = -60.0f + std::sin(animTime * 0.06f) * 55.0f;
    float by = 86.0f + std::sin(animTime * 0.35f) * 2.2f;

    glPushMatrix();
    glTranslatef(bx, by, bz);
    glRotatef(std::sin(animTime * 0.2f) * 4.0f, 0.0f, 0.0f, 1.0f);

    const float goreColors[4][3] = {
        { 0.92f, 0.25f, 0.22f }, // Đỏ san hô
        { 0.98f, 0.82f, 0.20f }, // Vàng rực
        { 0.95f, 0.96f, 0.98f }, // Trắng ngà
        { 0.22f, 0.65f, 0.85f }  // Lam ngọc
    };

    // Vòm tròn phía trên
    glPushMatrix();
    glTranslatef(0.0f, 3.5f, 0.0f);
    glScalef(6.8f, 8.2f, 6.8f);
    setMaterial(0.92f, 0.25f, 0.22f, 20.0f, 0.2f);
    glutSolidSphere(1.0, 20, 16);
    glPopMatrix();

    // Dải sọc màu trang trí quanh thân khí cầu
    for (int s = 0; s < 4; ++s) {
        setMaterial(goreColors[s][0], goreColors[s][1], goreColors[s][2], 25.0f);
        glPushMatrix();
        glTranslatef(0.0f, 2.8f - s * 0.8f, 0.0f);
        drawCylinder(6.85f - s * 0.35f, 0.65f, 24);
        glPopMatrix();
    }

    // Đáy côn thu
    setMaterial(0.88f, 0.35f, 0.20f, 20.0f);
    glPushMatrix();
    glTranslatef(0.0f, 0.8f, 0.0f);
    glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
    drawCone(5.8f, 5.0f, 20);
    glPopMatrix();

    // Ngọn lửa đốt khí nóng
    setMaterial(1.0f, 0.65f, 0.1f, 80.0f, 1.0f);
    GLfloat flameGlow[] = { 1.0f, 0.65f, 0.1f, 1.0f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, flameGlow);
    glPushMatrix();
    glTranslatef(0.0f, -4.5f, 0.0f);
    glScalef(0.45f, 0.9f + 0.3f * std::sin(animTime * 14.0f), 0.45f);
    glutSolidSphere(1.0, 10, 8);
    glPopMatrix();

    // Giỏ mây (Gondola)
    setMaterial(0.48f, 0.32f, 0.18f, 8.0f, 0.0f);
    glPushMatrix();
    glTranslatef(0.0f, -7.2f, 0.0f);
    drawCube(2.4f, 1.6f, 2.4f);
    glPopMatrix();

    // Bốn dây cáp treo
    setMaterial(0.25f, 0.25f, 0.28f, 10.0f);
    for (int cx = -1; cx <= 1; cx += 2) {
        for (int cz = -1; cz <= 1; cz += 2) {
            glPushMatrix();
            glTranslatef(cx * 0.95f, -5.8f, cz * 0.95f);
            drawCylinder(0.035f, 2.6f, 6);
            glPopMatrix();
        }
    }

    glPopMatrix();
}

void drawBoundaryScenery() {
    // Biển và bãi cát cố định ở phía Đông; mặt nước cũng khép hai đầu khu demo.
    setMaterial(0.06f, 0.40f, 0.68f, 50.0f, 0.55f);
    drawGroundRect(465.0f, 0.0f, 470.0f, 1400.0f, -0.39f);
    drawGroundRect(10.0f, 465.0f, 440.0f, 470.0f, -0.39f);
    drawGroundRect(10.0f, -465.0f, 440.0f, 470.0f, -0.39f);

    // Mặt nước biển tĩnh thanh lịch phong cách vector

    // Bãi cát vàng mịn màng ven bờ biển
    setMaterial(0.88f, 0.80f, 0.58f, 10.0f, 0.05f);
    drawGroundRect(228.0f, 0.0f, 20.0f, 440.0f, -0.365f);
    drawGroundRect(10.0f, 228.0f, 440.0f, 20.0f, -0.365f);
    drawGroundRect(10.0f, -228.0f, 440.0f, 20.0f, -0.365f);

    // Dải chân đồi đá thoai thoải phía Tây (Foothills buffer lùi xa ngoài rìa đường xá)
    setMaterial(0.26f, 0.32f, 0.28f, 5.0f, 0.0f);
    drawGroundRect(-330.0f, 0.0f, 210.0f, 880.0f, -0.385f);

    // LỚP NÚI 1: DÃY ĐẠI NGÀN HẬU CẢNH PHÍA XA (3 ngọn núi khổng lồ mờ sương, lùi sâu về x = -520m)
    // Đảm bảo chân núi xa nhất tại x = -520 + 195 = -325m, cách đường xá hơn 105m!
    struct BackgroundPeak { float x, z, r, h; };
    const BackgroundPeak bgPeaks[3] = {
        { -520.0f, -140.0f, 180.0f, 360.0f },
        { -535.0f,    5.0f, 195.0f, 400.0f },
        { -520.0f,  150.0f, 180.0f, 350.0f }
    };
    for (int i = 0; i < 3; ++i) {
        setMaterial(0.18f, 0.25f, 0.38f, 12.0f, 0.06f); // Lam chàm sương mờ xa xăm (Atmospheric Depth Haze)
        glPushMatrix();
        glTranslatef(bgPeaks[i].x, -0.38f, bgPeaks[i].z);
        drawCone(bgPeaks[i].r, bgPeaks[i].h, 8);
        // Chỏm tuyết vĩnh cửu trắng muốt trên đỉnh
        setMaterial(0.94f, 0.96f, 0.99f, 25.0f, 0.2f);
        glTranslatef(0.0f, bgPeaks[i].h * 0.65f, 0.0f);
        drawCone(bgPeaks[i].r * 0.40f, bgPeaks[i].h * 0.38f, 8);
        glPopMatrix();
    }

    // LỚP NÚI 2: DÃY NÚI TRUNG CẢNH HÙNG VĨ (Chỉ 4 ngọn núi biểu tượng phân bổ thoáng đãng)
    // Đặt lùi tại x = -370m, bán kính r <= 95m => Chân núi chạm tối đa x = -275m,
    // cách đường xá (x = -218m) hơn 55m và cách nhà (x = -192m) hơn 80m! TUYỆT ĐỐI KHÔNG LẤN ĐƯỜNG/NHÀ.
    struct ForegroundPeak { float x, z, r, h, colorR, colorG, colorB; };
    const ForegroundPeak fgPeaks[4] = {
        { -375.0f, -160.0f, 95.0f, 240.0f, 0.24f, 0.38f, 0.33f },
        { -365.0f,  -50.0f, 90.0f, 265.0f, 0.28f, 0.42f, 0.36f },
        { -368.0f,   55.0f, 92.0f, 255.0f, 0.25f, 0.39f, 0.34f },
        { -375.0f,  165.0f, 95.0f, 235.0f, 0.27f, 0.41f, 0.35f }
    };
    for (int i = 0; i < 4; ++i) {
        setMaterial(fgPeaks[i].colorR, fgPeaks[i].colorG, fgPeaks[i].colorB, 10.0f, 0.05f);
        glPushMatrix();
        glTranslatef(fgPeaks[i].x, -0.38f, fgPeaks[i].z);
        drawCone(fgPeaks[i].r, fgPeaks[i].h, 8);
        // Đỉnh tuyết phủ sắc nét
        setMaterial(0.92f, 0.95f, 0.98f, 20.0f, 0.15f);
        glTranslatef(0.0f, fgPeaks[i].h * 0.62f, 0.0f);
        drawCone(fgPeaks[i].r * 0.42f, fgPeaks[i].h * 0.40f, 8);
        glPopMatrix();
    }

    // Các cụm tuabin gió phong điện trên sườn núi phía Tây
    drawWindTurbines();
}

void drawBridge() {
    // Cầu đường bộ cố định nối đại lộ phía Đông qua biển đến đảo ngoài khơi.
    setMaterial(0.50f, 0.56f, 0.59f, 18.0f, 0.12f);
    glPushMatrix(); glTranslatef(350.0f, 0.72f, 0.0f); drawCube(300.0f, 1.0f, 20.0f); glPopMatrix();

    // Đoạn dốc nối mặt đường trong thành phố với mặt cầu.
    setMaterial(0.34f, 0.36f, 0.38f, 10.0f, 0.05f);
    glPushMatrix();
    glTranslatef(205.0f, 0.54f, 0.0f);
    glRotatef(2.5f, 0.0f, 0.0f, 1.0f);
    drawCube(26.0f, 0.50f, 19.0f);
    glPopMatrix();

    setMaterial(0.16f, 0.18f, 0.20f, 7.0f, 0.0f);
    drawGroundRect(350.0f, 0.0f, 300.0f, 14.0f, 1.24f);
    setMaterial(0.94f, 0.95f, 0.91f, 15.0f, 0.0f);
    for (int i = 0; i < 23; ++i) {
        float x = 207.0f + i * 13.0f;
        drawGroundRect(x, 0.0f, 5.0f, 0.22f, 1.27f);
    }

    // Lan can trắng chạy dọc hai mép cầu.
    setMaterial(0.88f, 0.91f, 0.91f, 24.0f, 0.1f);
    for (int side = -1; side <= 1; side += 2) {
        glPushMatrix(); glTranslatef(350.0f, 2.25f, side * 9.2f); drawCube(300.0f, 0.38f, 0.48f); glPopMatrix();
        for (int i = 0; i <= 30; ++i) {
            glPushMatrix(); glTranslatef(200.0f + i * 10.0f, 1.70f, side * 9.2f); drawCube(0.24f, 0.72f, 0.24f); glPopMatrix();
        }
    }

    // Trụ cầu trong biển và hai tháp cáp màu xanh thép.
    setMaterial(0.61f, 0.67f, 0.68f, 18.0f, 0.14f);
    for (int i = 0; i < 6; ++i) {
        float x = 230.0f + i * 46.0f;
        glPushMatrix(); glTranslatef(x, -8.0f, 0.0f); drawCube(4.0f, 17.0f, 15.0f); glPopMatrix();
    }
    const float towerX[2] = { 270.0f, 360.0f };
    setMaterial(0.16f, 0.38f, 0.52f, 24.0f, 0.18f);
    for (float x : towerX) {
        for (int side = -1; side <= 1; side += 2) {
            glPushMatrix(); glTranslatef(x, 14.0f, side * 8.5f); drawCube(2.4f, 27.0f, 2.4f); glPopMatrix();
        }
        glPushMatrix(); glTranslatef(x, 27.6f, 0.0f); drawCube(3.0f, 1.6f, 20.0f); glPopMatrix();
    }
    for (int side = -1; side <= 1; side += 2) {
        drawPowerSpan(Vec3(towerX[0], 0.0f, side * 8.5f), Vec3(towerX[1], 0.0f, side * 8.5f), 26.8f, 4.5f);
        setMaterial(0.78f, 0.83f, 0.84f, 18.0f, 0.08f);
        for (int i = 1; i < 9; ++i) {
            float x = towerX[0] + i * 10.0f;
            float u = (x - towerX[0]) / (towerX[1] - towerX[0]);
            float cableY = 26.8f - 4.5f * 6.0f * u * (1.0f - u);
            glPushMatrix(); glTranslatef(x, (cableY + 1.7f) * 0.5f, side * 8.5f); drawCube(0.14f, cableY - 1.7f, 0.14f); glPopMatrix();
        }
    }
}

// Nhà thấp tầng màu sắc đồng bộ với khu đô thị, dựng hoàn toàn từ các khối cơ bản.
void drawIslandHouse(float x, float z, float width, float height, float depth,
                     float r, float g, float b) {
    setMaterial(r, g, b, 10.0f, 0.02f);
    glPushMatrix(); glTranslatef(x, height * 0.5f, z); drawCube(width, height, depth); glPopMatrix();

    setMaterial(r * 0.70f, g * 0.72f, b * 0.76f, 12.0f, 0.04f);
    glPushMatrix(); glTranslatef(x, height + 0.38f, z); drawCube(width + 1.4f, 0.72f, depth + 1.4f); glPopMatrix();

    setMaterial(0.63f, 0.84f, 0.91f, 28.0f, 0.22f);
    float frontZ = z + (z > 0.0f ? -1.0f : 1.0f) * (depth * 0.5f + 0.11f);
    int rows = height > 8.0f ? 2 : 1;
    for (int row = 0; row < rows; ++row) {
        float wy = 2.2f + row * 3.2f;
        for (int col = -1; col <= 1; ++col) {
            if (row == 0 && col == 0) continue;
            glPushMatrix(); glTranslatef(x + col * width * 0.27f, wy, frontZ); drawCube(width * 0.17f, 1.35f, 0.14f); glPopMatrix();
        }
    }
    setMaterial(0.45f, 0.27f, 0.16f, 7.0f, 0.0f);
    glPushMatrix(); glTranslatef(x, 1.15f, frontZ); drawCube(1.55f, 2.3f, 0.18f); glPopMatrix();
}

// Cây trang trí trên đảo gồm thân trụ và tán lá hình cầu.
void drawIslandTree(float x, float z, float scale) {
    setMaterial(0.38f, 0.24f, 0.12f, 5.0f, 0.0f);
    glPushMatrix(); glTranslatef(x, 2.0f * scale, z); drawCylinder(0.32f * scale, 4.0f * scale, 9); glPopMatrix();
    setMaterial(0.12f, 0.50f, 0.20f, 7.0f, 0.02f);
    glPushMatrix(); glTranslatef(x, 4.8f * scale, z); glScalef(2.0f * scale, 2.3f * scale, 2.0f * scale); glutSolidSphere(1.0, 14, 10); glPopMatrix();
    setMaterial(0.22f, 0.66f, 0.23f, 8.0f, 0.02f);
    glPushMatrix(); glTranslatef(x + 0.9f * scale, 5.0f * scale, z + 0.3f * scale); glScalef(1.25f * scale, 1.45f * scale, 1.25f * scale); glutSolidSphere(1.0, 12, 9); glPopMatrix();
}

void drawIsland() {
    // Đảo nhỏ cố định ở đầu cầu: bờ cát ellipse, lõi cỏ, đường nội bộ và khu dân cư.
    setMaterial(0.78f, 0.67f, 0.43f, 5.0f, 0.0f);
    drawEllipseGround(570.0f, 0.0f, 105.0f, 128.0f, -0.30f);
    setMaterial(0.31f, 0.61f, 0.28f, 5.0f, 0.0f);
    drawEllipseGround(570.0f, 0.0f, 91.0f, 114.0f, -0.16f);

    // Dốc cầu hạ xuống mặt đường của đảo, nối liền mặt cầu và tuyến nội bộ.
    setMaterial(0.31f, 0.33f, 0.35f, 9.0f, 0.02f);
    glPushMatrix(); glTranslatef(515.0f, 0.62f, 0.0f); glRotatef(-1.8f, 0.0f, 0.0f, 1.0f); drawCube(40.0f, 0.50f, 14.0f); glPopMatrix();

    setMaterial(0.18f, 0.20f, 0.22f, 8.0f, 0.0f);
    drawGroundRect(592.0f, 0.0f, 114.0f, 9.0f, -0.025f);
    drawGroundRect(590.0f, 0.0f, 8.0f, 174.0f, -0.020f);
    setMaterial(0.91f, 0.92f, 0.86f, 10.0f, 0.0f);
    for (int i = 0; i < 8; ++i) drawGroundRect(542.0f + i * 14.0f, 0.0f, 5.0f, 0.22f, 0.0f);
    for (int i = -4; i <= 4; ++i) drawGroundRect(590.0f, i * 14.0f, 0.22f, 5.0f, 0.005f);

    // Dãy nhà dân thấp tầng đặt hai bên đường, chừa khoảng thông thoáng ở giao lộ.
    drawIslandHouse(550.0f,  29.0f, 15.0f, 7.0f, 12.0f, 0.90f, 0.76f, 0.52f);
    drawIslandHouse(574.0f,  31.0f, 16.0f, 9.0f, 13.0f, 0.82f, 0.88f, 0.91f);
    drawIslandHouse(610.0f,  27.0f, 17.0f, 8.0f, 13.0f, 0.89f, 0.62f, 0.42f);
    drawIslandHouse(550.0f, -29.0f, 15.0f, 8.0f, 12.0f, 0.78f, 0.86f, 0.72f);
    drawIslandHouse(574.0f, -32.0f, 16.0f, 7.0f, 13.0f, 0.92f, 0.79f, 0.63f);
    drawIslandHouse(612.0f, -27.0f, 17.0f, 9.0f, 13.0f, 0.73f, 0.83f, 0.91f);

    // Bãi cây xanh và lối dạo ở phần phía Đông của đảo.
    setMaterial(0.24f, 0.56f, 0.25f, 4.0f, 0.0f);
    drawEllipseGround(635.0f, 66.0f, 18.0f, 27.0f, -0.04f);
    setMaterial(0.76f, 0.73f, 0.62f, 5.0f, 0.0f);
    drawGroundRect(635.0f, 66.0f, 3.0f, 40.0f, 0.0f);
    drawGroundRect(635.0f, 66.0f, 32.0f, 3.0f, 0.005f);
    drawIslandTree(627.0f, 57.0f, 1.05f);
    drawIslandTree(643.0f, 60.0f, 0.90f);
    drawIslandTree(629.0f, 76.0f, 0.95f);

    // Hải đăng nhỏ làm mốc nhận biết đảo từ phía thành phố.
    setMaterial(0.91f, 0.90f, 0.82f, 16.0f, 0.04f);
    glPushMatrix(); glTranslatef(637.0f, 12.0f, -76.0f); drawCylinder(3.6f, 24.0f, 20); glPopMatrix();
    setMaterial(0.83f, 0.23f, 0.16f, 12.0f, 0.0f);
    glPushMatrix(); glTranslatef(637.0f, 23.0f, -76.0f); drawCube(7.4f, 2.6f, 7.4f); glPopMatrix();
    setMaterial(0.42f, 0.75f, 0.88f, 35.0f, 0.25f);
    glPushMatrix(); glTranslatef(637.0f, 25.5f, -76.0f); glScalef(3.1f, 1.6f, 3.1f); glutSolidSphere(1.0, 16, 12); glPopMatrix();
    setMaterial(0.81f, 0.23f, 0.16f, 12.0f, 0.0f);
    glPushMatrix(); glTranslatef(637.0f, 28.0f, -76.0f); drawCone(4.4f, 4.8f, 16); glPopMatrix();

    // Cầu tàu ngắm biển ở mép Đông đảo, có cọc gỗ đỡ mặt sàn.
    setMaterial(0.50f, 0.34f, 0.20f, 7.0f, 0.0f);
    glPushMatrix(); glTranslatef(666.0f, 0.0f, 0.0f); drawCube(32.0f, 0.70f, 10.0f); glPopMatrix();
    for (int i = 0; i < 5; ++i) for (int side = -1; side <= 1; side += 2) {
        glPushMatrix(); glTranslatef(652.0f + i * 7.0f, -2.0f, side * 4.0f); drawCylinder(0.38f, 4.2f, 8); glPopMatrix();
    }
}

void drawCity(const Vec3& center) {
    drawRoadAndBlocks(center);
    drawBoundaryScenery();
    drawIsland();
    drawBridge();

    int gx0 = std::max(CITY_MIN_ROAD_INDEX, (int)std::floor((center.x - 185.0f) / CITY_GRID_SPACING));
    int gx1 = std::min(CITY_MAX_ROAD_INDEX, (int)std::floor((center.x + 185.0f) / CITY_GRID_SPACING));
    int gz0 = std::max(CITY_MIN_ROAD_INDEX, (int)std::floor((center.z - 185.0f) / CITY_GRID_SPACING));
    int gz1 = std::min(CITY_MAX_ROAD_INDEX, (int)std::floor((center.z + 185.0f) / CITY_GRID_SPACING));

    for (int gx = gx0; gx <= gx1; gx++) {
        for (int gz = gz0; gz <= gz1; gz++) {
            if (!hasBuildingAt(gx, gz)) continue;
            float x = gx * CITY_GRID_SPACING, z = gz * CITY_GRID_SPACING;
            float dx = x - center.x, dz = z - center.z;
            if (dx * dx + dz * dz < 205.0f * 205.0f) {
                drawBuilding(getBuilding(gx, gz));
            }
        }
    }

    drawDistrictAmenities(center);
    drawStreetProps(center);

    for (const auto& c : cityCars) {
        if (std::fabs(c.x - center.x) < 180.0f && std::fabs(c.z - center.z) < 180.0f) {
            drawCityCar(c);
        }
    }
}

// =========================================================================================
// 9. BẦU TRỜI XANH NGỌC & MÂY TRẮNG BỒNG BỀNH (CHUẨN HÌNH 2)
// =========================================================================================
void updateLightingAndSky() {
    if (timeAuto && !paused) {
        timeOfDay += 0.016f * timeSpeed;
        if (timeOfDay >= 24.0f) timeOfDay -= 24.0f;
        if (timeOfDay < 0.0f)  timeOfDay += 24.0f;
    }

    animTime += 0.016f;

    if (timeOfDay >= 8.0f && timeOfDay <= 16.5f) {
        nightFactor = 0.0f;
    } else if (timeOfDay > 16.5f && timeOfDay < 19.5f) {
        nightFactor = (timeOfDay - 16.5f) / 3.0f;
    } else if (timeOfDay >= 19.5f || timeOfDay < 5.0f) {
        nightFactor = 1.0f;
    } else {
        nightFactor = 1.0f - (timeOfDay - 5.0f) / 3.0f;
    }
    nightFactor = clampf(nightFactor, 0.0f, 1.0f);

    // Bầu trời ban ngày trong xanh nắng vàng như Hình 2:
    Color4 skyHorizon, lightDiffuse, lightAmbient;

    if (timeOfDay >= 0.0f && timeOfDay < 5.0f) {
        skyHorizon = Color4(0.08f, 0.12f, 0.22f);
        lightDiffuse = Color4(0.35f, 0.42f, 0.60f);
        lightAmbient = Color4(0.20f, 0.22f, 0.28f);
    } else if (timeOfDay >= 5.0f && timeOfDay < 7.5f) {
        float t = (timeOfDay - 5.0f) / 2.5f;
        skyHorizon = Color4::lerp(Color4(0.95f, 0.55f, 0.35f), Color4(0.65f, 0.82f, 0.95f), t);
        lightDiffuse = Color4::lerp(Color4(1.0f, 0.65f, 0.35f), Color4(1.0f, 0.98f, 0.92f), t);
        lightAmbient = Color4::lerp(Color4(0.24f, 0.22f, 0.28f), Color4(0.38f, 0.44f, 0.52f), t);
    } else if (timeOfDay >= 7.5f && timeOfDay < 16.5f) {
        // Nắng vàng rực rỡ và chân trời xanh ngọc dịu mát
        skyHorizon = Color4(0.65f, 0.82f, 0.95f);
        lightDiffuse = Color4(1.0f, 0.98f, 0.92f);
        lightAmbient = Color4(0.38f, 0.44f, 0.52f);
    } else if (timeOfDay >= 16.5f && timeOfDay < 19.5f) {
        float t = (timeOfDay - 16.5f) / 3.0f;
        skyHorizon = Color4::lerp(Color4(0.65f, 0.82f, 0.95f), Color4(0.08f, 0.12f, 0.22f), t);
        lightDiffuse = Color4::lerp(Color4(1.0f, 0.60f, 0.28f), Color4(0.35f, 0.42f, 0.60f), t);
        lightAmbient = Color4::lerp(Color4(0.38f, 0.30f, 0.32f), Color4(0.20f, 0.22f, 0.28f), t);
    } else {
        skyHorizon = Color4(0.08f, 0.12f, 0.22f);
        lightDiffuse = Color4(0.35f, 0.42f, 0.60f);
        lightAmbient = Color4(0.20f, 0.22f, 0.28f);
    }

    glClearColor(skyHorizon.r, skyHorizon.g, skyHorizon.b, 1.0f);

    if (!parallelProjection) {
        glEnable(GL_FOG);
        GLfloat fogColor[] = { skyHorizon.r, skyHorizon.g, skyHorizon.b, 1.0f };
        glFogfv(GL_FOG_COLOR, fogColor);
        glFogi(GL_FOG_MODE, GL_LINEAR);
        glFogf(GL_FOG_START, 260.0f);
        glFogf(GL_FOG_END, 650.0f);
    } else {
        glDisable(GL_FOG);
    }

    // Góc chiếu sáng mặt trời tạo bóng nghiêng tự nhiên như Hình 2
    float sunAngle = (timeOfDay - 6.0f) * (2.0f * PI / 24.0f);
    float lightDist = 280.0f;
    float lx = lightDist * std::cos(sunAngle);
    float ly = lightDist * std::sin(sunAngle);
    float lz = 100.0f;
    if (ly < 0.0f) { lx = -lx; ly = -ly; lz = -lz; }

    GLfloat lightPos[] = { lx, ly, lz, 0.0f };
    GLfloat diffMat[]  = { lightDiffuse.r, lightDiffuse.g, lightDiffuse.b, 1.0f };
    GLfloat ambMat[]   = { lightAmbient.r, lightAmbient.g, lightAmbient.b, 1.0f };
    GLfloat specMat[]  = { 0.5f, 0.5f, 0.5f, 1.0f };

    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffMat);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  ambMat);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specMat);
}

/**
 * Vẽ bầu trời gradient đa tầng sống động: xanh ngọc ban ngày, hoàng hôn tím cam, đêm trăng sao.
 */
void drawSkyGradient() {
    glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_DEPTH_BUFFER_BIT | GL_TRANSFORM_BIT);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_FOG);
    glDepthMask(GL_FALSE);

    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity(); glOrtho(0, 1, 0, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity();

    // Bầu trời ban ngày phong cách Stylized Cyan / Azure Blue
    Color4 cZenith(0.18f, 0.48f, 0.82f);
    Color4 cMid(0.42f, 0.72f, 0.94f);
    Color4 cHorizon(0.68f, 0.84f, 0.96f);

    float tDay = timeOfDay;
    if (tDay >= 5.0f && tDay < 7.5f) {
        float f = (tDay - 5.0f) / 2.5f;
        cZenith  = Color4::lerp(Color4(0.15f, 0.12f, 0.35f), Color4(0.18f, 0.48f, 0.82f), f);
        cMid     = Color4::lerp(Color4(0.95f, 0.48f, 0.35f), Color4(0.42f, 0.72f, 0.94f), f);
        cHorizon = Color4::lerp(Color4(0.98f, 0.75f, 0.45f), Color4(0.68f, 0.84f, 0.96f), f);
    } else if (tDay >= 16.5f && tDay < 19.5f) {
        float f = (tDay - 16.5f) / 3.0f;
        cZenith  = Color4::lerp(Color4(0.18f, 0.48f, 0.82f), Color4(0.06f, 0.08f, 0.20f), f);
        cMid     = Color4::lerp(Color4(0.95f, 0.45f, 0.28f), Color4(0.16f, 0.14f, 0.28f), f);
        cHorizon = Color4::lerp(Color4(0.98f, 0.68f, 0.36f), Color4(0.08f, 0.12f, 0.22f), f);
    } else if (tDay >= 19.5f || tDay < 5.0f) {
        cZenith  = Color4(0.04f, 0.06f, 0.15f);
        cMid     = Color4(0.08f, 0.12f, 0.24f);
        cHorizon = Color4(0.06f, 0.10f, 0.20f);
    }

    glBegin(GL_QUADS);
        glColor3f(cHorizon.r, cHorizon.g, cHorizon.b); glVertex2f(0.0f, 0.0f);
        glColor3f(cHorizon.r, cHorizon.g, cHorizon.b); glVertex2f(1.0f, 0.0f);
        glColor3f(cMid.r, cMid.g, cMid.b);             glVertex2f(1.0f, 0.55f);
        glColor3f(cMid.r, cMid.g, cMid.b);             glVertex2f(0.0f, 0.55f);

        glColor3f(cMid.r, cMid.g, cMid.b);             glVertex2f(0.0f, 0.55f);
        glColor3f(cMid.r, cMid.g, cMid.b);             glVertex2f(1.0f, 0.55f);
        glColor3f(cZenith.r, cZenith.g, cZenith.b);    glVertex2f(1.0f, 1.0f);
        glColor3f(cZenith.r, cZenith.g, cZenith.b);    glVertex2f(0.0f, 1.0f);
    glEnd();

    // Sao nhỏ chỉ hiện rõ vào ban đêm, được tạo bằng tọa độ giả ngẫu nhiên cố định.
    if (timeOfDay >= 19.5f || timeOfDay < 5.0f) {
        glPointSize(1.7f);
        glBegin(GL_POINTS);
        for (int i = 0; i < 96; ++i) {
            unsigned int a = hashInt(i, 0x8142u), b = hashInt(i, 0x2917u);
            glColor3f(0.76f + (a & 255u) / 1200.0f, 0.82f + (b & 255u) / 1300.0f, 1.0f);
            glVertex2f(((a >> 8) & 0xffffu) / 65535.0f, 0.40f + ((b >> 8) & 0xffffu) / 109225.0f);
        }
        glEnd();
    }

    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW);
    glPopAttrib();
}

void drawFluffyCloud(float x, float y, float z, float scale) {
    glPushMatrix();
    glTranslatef(x, y, z);
    glScalef(scale, scale * 0.52f, scale);

    setMaterial(0.96f, 0.98f, 1.0f, 25.0f, 0.3f);
    glPushMatrix();
    glScalef(5.8f, 2.6f, 4.0f);
    glutSolidSphere(1.0, 12, 10);
    glPopMatrix();

    glPushMatrix(); glTranslatef(3.2f, 0.4f, 0.8f); glScalef(3.8f, 2.4f, 3.2f); glutSolidSphere(1.0, 10, 8); glPopMatrix();
    glPushMatrix(); glTranslatef(-3.2f, 0.3f, -0.6f); glScalef(3.6f, 2.2f, 3.0f); glutSolidSphere(1.0, 10, 8); glPopMatrix();
    glPushMatrix(); glTranslatef(1.0f, 1.1f, -1.0f); glScalef(3.4f, 2.2f, 2.8f); glutSolidSphere(1.0, 10, 8); glPopMatrix();
    glPushMatrix(); glTranslatef(-1.2f, 0.7f, 1.2f); glScalef(3.0f, 2.0f, 2.6f); glutSolidSphere(1.0, 10, 8); glPopMatrix();

    glPopMatrix();
}

void drawClouds(const Vec3& center) {
    if (nightFactor > 0.85f) return;
    float cloudWind = animTime * 1.8f;
    for (int i = 0; i < 14; ++i) {
        float cx = -260.0f + std::fmod((i * 75.0f + cloudWind), 850.0f) - 150.0f;
        float cz = -260.0f + (float)((i * 68) % 520);
        float cy = 88.0f + (i % 3) * 18.0f;
        float sc = 2.2f + (i % 4) * 0.65f;
        drawFluffyCloud(cx, cy, cz, sc);
    }
}

void drawSunAndMoon(const Vec3& center) {
    glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT);
    glDisable(GL_LIGHTING);
    glDisable(GL_FOG);

    float sunAngle = (timeOfDay - 6.0f) * (2.0f * PI / 24.0f);
    float dist = 280.0f;
    float lx = dist * std::cos(sunAngle);
    float ly = dist * std::sin(sunAngle);
    float lz = -110.0f;

    if (ly > -10.0f) {
        // Mặt trời vàng rực ban ngày + quầng hào quang
        glPushMatrix();
        glTranslatef(center.x + lx, center.y + ly, center.z + lz);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(1.0f, 0.92f, 0.50f, 0.25f);
        glutSolidSphere(26.0, 18, 14);
        glColor4f(1.0f, 0.85f, 0.40f, 0.45f);
        glutSolidSphere(18.0, 18, 14);
        glColor3f(1.0f, 0.98f, 0.75f);
        glutSolidSphere(12.0, 24, 18);
        glPopMatrix();
    }
    
    if (ly < 25.0f) {
        // Mặt trăng bạc ban đêm + ánh sáng trăng
        glPushMatrix();
        glTranslatef(center.x - lx, center.y - ly, center.z - lz);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(0.6f, 0.75f, 1.0f, 0.25f);
        glutSolidSphere(18.0, 18, 14);
        glColor3f(0.94f, 0.96f, 1.0f);
        glutSolidSphere(10.0, 24, 18);
        glPopMatrix();
    }

    glPopAttrib();
}

// =========================================================================================
// 10. MA TRẬN QUAN SÁT & GIAO DIỆN RADAR HUD SÁNG ĐẸP
// =========================================================================================
void setCamera() {
    float hoverBob = (1.0f - std::min(1.0f, length(droneVelocity) * 0.12f)) * std::sin(animTime * 3.2f) * 0.05f;
    Vec3 target = drone + Vec3(0.0f, 0.12f + hoverBob, 0.0f);
    Vec3 forward = lookDirection();

    // Rung lắc Camera khi va chạm (Camera Shake)
    Vec3 shake(0.0f, 0.0f, 0.0f);
    if (cameraShakeTimer > 0.0f) {
        float amp = cameraShakeTimer * 0.55f;
        shake = Vec3(std::sin(animTime * 52.0f) * amp, std::cos(animTime * 45.0f) * amp, std::sin(animTime * 36.0f) * amp * 0.5f);
    }

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if (parallelProjection) {
        float dist = 45.0f * zoomD;
        if (parallelPlane == 0)      gluLookAt(target.x, target.y, target.z + dist, target.x, target.y, target.z, 0, 1, 0);
        else if (parallelPlane == 1) gluLookAt(target.x + dist, target.y, target.z, target.x, target.y, target.z, 0, 1, 0);
        else                         gluLookAt(target.x, target.y + dist, target.z, target.x, target.y, target.z, 0, 0, -1);
    } else if (cameraMode == 0) {
        // FPV - Góc nhìn từ cụm Gimbal Camera của Drone
        Vec3 eye = drone + Vec3(0.0f, -0.05f + hoverBob, -0.32f) + shake;
        Vec3 look = eye + forward * 38.0f;
        gluLookAt(eye.x, eye.y, eye.z, look.x, look.y, look.z, 0, 1, 0);
    } else if (cameraMode == 1) {
        // Camera thứ ba bám theo sau lưng Drone với Smooth Damp & Spring Arm
        float r = cameraR * zoomD;
        float speedFactor = std::min(1.0f, length(droneVelocity) / forwardSpeed);
        float dynamicR = r * (1.0f + speedFactor * 0.16f); // Giãn nhẹ camera khi bay tốc độ cao

        Vec3 desiredEye = drone - forward * dynamicR + Vec3(0.0f, dynamicR * 0.54f + hoverBob, 0.0f);
        Vec3 desiredLook = drone + forward * 6.0f + Vec3(0.0f, -4.0f + hoverBob, 0.0f);

        if (!smoothCamInitialized) {
            smoothCameraEye = desiredEye;
            smoothCameraLook = desiredLook;
            smoothCamInitialized = true;
        } else {
            smoothCameraEye = smoothCameraEye + (desiredEye - smoothCameraEye) * 0.16f;
            smoothCameraLook = smoothCameraLook + (desiredLook - smoothCameraLook) * 0.22f;
        }

        Vec3 eye = smoothCameraEye + shake;
        Vec3 look = smoothCameraLook + shake;
        gluLookAt(eye.x, eye.y, eye.z, look.x, look.y, look.z, 0, 1, 0);
    } else {
        // Quỹ đạo tự do quanh Drone
        float r = cameraR * zoomD;
        Vec3 eye(target.x + r * std::cos(phi) * std::sin(theta),
                 target.y + r * std::sin(phi),
                 target.z + r * std::cos(phi) * std::cos(theta));
        eye = eye + shake;
        gluLookAt(eye.x, eye.y, eye.z, target.x, target.y, target.z, 0, 1, 0);
    }
}

void drawText(float x, float y, const std::string& text, void* font = GLUT_BITMAP_HELVETICA_18) {
    glRasterPos2f(x, y);
    for (unsigned char c : text) glutBitmapCharacter(font, c);
}

void drawHud() {
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity(); gluOrtho2D(0, windowWidth, 0, windowHeight);
    glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity();

    // 1. Dòng thông tin trạng thái Drone góc trên bên trái
    glColor4f(0.12f, 0.16f, 0.24f, 0.85f);
    glBegin(GL_QUADS);
        glVertex2f(16, windowHeight - 74);
        glVertex2f(440, windowHeight - 74);
        glVertex2f(440, windowHeight - 16);
        glVertex2f(16, windowHeight - 16);
    glEnd();
    glColor3f(0.20f, 0.65f, 0.85f);
    glLineWidth(1.6f);
    glBegin(GL_LINE_LOOP);
        glVertex2f(16, windowHeight - 74);
        glVertex2f(440, windowHeight - 74);
        glVertex2f(440, windowHeight - 16);
        glVertex2f(16, windowHeight - 16);
    glEnd();
    glLineWidth(1.0f);

    int hours = (int)timeOfDay;
    int mins  = (int)((timeOfDay - hours) * 60.0f);
    std::string phaseStr;
    if (timeOfDay >= 5.0f && timeOfDay < 7.5f) phaseStr = "SUNRISE";
    else if (timeOfDay >= 7.5f && timeOfDay < 16.5f) phaseStr = "DAY";
    else if (timeOfDay >= 16.5f && timeOfDay < 19.5f) phaseStr = "SUNSET";
    else phaseStr = "NIGHT";

    std::ostringstream line1;
    line1 << "DRONE CITYWALK | ALT " << (int)drone.y << " m | SPEED " << (int)length(droneVelocity) << " m/s";
    glColor3f(1.0f, 1.0f, 1.0f);
    drawText(26, windowHeight - 40, line1.str(), GLUT_BITMAP_HELVETICA_12);

    std::ostringstream line2;
    std::string modeStr = cameraMode == 0 ? "FPV" : (cameraMode == 1 ? "FOLLOW" : "ORBIT");
    if (parallelProjection) {
        modeStr += parallelPlane == 0 ? " | ORTHO XOY" : (parallelPlane == 1 ? " | ORTHO YOZ" : " | ORTHO ZOX");
    } else {
        modeStr += " | PERSPECTIVE";
    }
    line2 << "CAM " << modeStr << " | " << std::setfill('0') << std::setw(2) << hours << ":" << std::setw(2) << mins << " " << phaseStr;
    glColor3f(0.95f, 0.85f, 0.35f);
    drawText(26, windowHeight - 62, line2.str(), GLUT_BITMAP_HELVETICA_12);

    // 2. RADAR MINIMAP GÓC TRÊN BÊN PHẢI (Thiết kế nền sáng sạch sẽ, hài hòa)
    int mapW = 186, mapH = 144, mx = windowWidth - 206, my = windowHeight - 166;
    glColor4f(0.94f, 0.96f, 0.98f, 0.92f); // Nền xám bạc thanh lịch
    glBegin(GL_QUADS);
        glVertex2f(mx, my); glVertex2f(mx + mapW, my);
        glVertex2f(mx + mapW, my + mapH); glVertex2f(mx, my + mapH);
    glEnd();
    glColor3f(0.30f, 0.45f, 0.65f);
    glLineWidth(1.6f);
    glBegin(GL_LINE_LOOP);
        glVertex2f(mx, my); glVertex2f(mx + mapW, my);
        glVertex2f(mx + mapW, my + mapH); glVertex2f(mx, my + mapH);
    glEnd();
    glLineWidth(1.0f);

    float scale = 0.55f, centerX = mx + mapW * 0.5f, centerY = my + mapH * 0.5f;
    glEnable(GL_SCISSOR_TEST);
    glScissor(mx, my, mapW, mapH);

    // Các tuyến phố trên Radar (Màu xám sẫm)
    glColor3f(0.35f, 0.32f, 0.40f);
    int roadX0 = std::max(CITY_MIN_DISTRICT, (int)std::floor((drone.x - 140) / (CITY_GRID_SPACING * 6)));
    int roadX1 = std::min(CITY_MAX_DISTRICT + 1, (int)std::floor((drone.x + 140) / (CITY_GRID_SPACING * 6)));
    int roadZ0 = std::max(CITY_MIN_DISTRICT, (int)std::floor((drone.z - 140) / (CITY_GRID_SPACING * 6)));
    int roadZ1 = std::min(CITY_MAX_DISTRICT + 1, (int)std::floor((drone.z + 140) / (CITY_GRID_SPACING * 6)));

    for (int k = roadX0; k <= roadX1; k++) {
        float x = centerX + (k * 6.0f * CITY_GRID_SPACING - drone.x) * scale;
        glBegin(GL_QUADS);
            glVertex2f(x - 6.5f * scale, my); glVertex2f(x + 6.5f * scale, my);
            glVertex2f(x + 6.5f * scale, my + mapH); glVertex2f(x - 6.5f * scale, my + mapH);
        glEnd();
    }
    for (int k = roadZ0; k <= roadZ1; k++) {
        float z = centerY + (drone.z - k * 6.0f * CITY_GRID_SPACING) * scale;
        glBegin(GL_QUADS);
            glVertex2f(mx, z - 6.5f * scale); glVertex2f(mx + mapW, z - 6.5f * scale);
            glVertex2f(mx + mapW, z + 6.5f * scale); glVertex2f(mx, z + 6.5f * scale);
        glEnd();
    }

    // Đảo ngoài khơi và cầu nối được đánh dấu cố định trên radar.
    float islandMapX = centerX + (570.0f - drone.x) * scale;
    float islandMapY = centerY + drone.z * scale;
    glColor3f(0.78f, 0.66f, 0.42f);
    glBegin(GL_TRIANGLE_FAN);
        glVertex2f(islandMapX, islandMapY);
        for (int i = 0; i <= 48; ++i) {
            float a = 2.0f * PI * i / 48.0f;
            glVertex2f(islandMapX + std::cos(a) * 105.0f * scale,
                      islandMapY + std::sin(a) * 128.0f * scale);
        }
    glEnd();
    glColor3f(0.31f, 0.61f, 0.28f);
    glBegin(GL_TRIANGLE_FAN);
        glVertex2f(islandMapX, islandMapY);
        for (int i = 0; i <= 48; ++i) {
            float a = 2.0f * PI * i / 48.0f;
            glVertex2f(islandMapX + std::cos(a) * 91.0f * scale,
                      islandMapY + std::sin(a) * 114.0f * scale);
        }
    glEnd();

    // Đường cầu cố định kéo dài từ mép Đông thành phố đến tận đảo.
    glColor3f(0.22f, 0.25f, 0.29f);
    float bridgeY = centerY + drone.z * scale;
    float bridgeX0 = centerX + (200.0f - drone.x) * scale;
    float bridgeX1 = centerX + (500.0f - drone.x) * scale;
    glBegin(GL_QUADS);
        glVertex2f(bridgeX0, bridgeY - 5.0f * scale);
        glVertex2f(bridgeX1, bridgeY - 5.0f * scale);
        glVertex2f(bridgeX1, bridgeY + 5.0f * scale);
        glVertex2f(bridgeX0, bridgeY + 5.0f * scale);
    glEnd();

    // Các khối nhà cao ốc trên Radar (Xanh đá phiến nhã nhặn)
    glColor3f(0.55f, 0.65f, 0.75f);
    int gx0 = (int)std::floor((drone.x - 130) / CITY_GRID_SPACING);
    int gx1 = (int)std::floor((drone.x + 130) / CITY_GRID_SPACING);
    int gz0 = (int)std::floor((drone.z - 130) / CITY_GRID_SPACING);
    int gz1 = (int)std::floor((drone.z + 130) / CITY_GRID_SPACING);

    for (int gx = gx0; gx <= gx1; gx++) {
        for (int gz = gz0; gz <= gz1; gz++) {
            if (!hasBuildingAt(gx, gz)) continue;
            Building b = getBuilding(gx, gz);
            float rx = centerX + (b.x - drone.x) * scale;
            float ry = centerY + (drone.z - b.z) * scale;
            glBegin(GL_QUADS);
                glVertex2f(rx - b.w * scale * 0.5f, ry - b.d * scale * 0.5f);
                glVertex2f(rx + b.w * scale * 0.5f, ry - b.d * scale * 0.5f);
                glVertex2f(rx + b.w * scale * 0.5f, ry + b.d * scale * 0.5f);
                glVertex2f(rx - b.w * scale * 0.5f, ry + b.d * scale * 0.5f);
            glEnd();
        }
    }

    // Mũi tên đỏ chỉ hướng Drone ở tâm radar
    glColor3f(0.95f, 0.15f, 0.15f);
    glPushMatrix();
    glTranslatef(centerX, centerY, 0.0f);
    glRotatef(theta * RAD2DEG, 0.0f, 0.0f, 1.0f);
    glBegin(GL_TRIANGLES);
        glVertex2f(0.0f, 8.0f);
        glVertex2f(-5.0f, -4.0f);
        glVertex2f(5.0f, -4.0f);
    glEnd();
    glPopMatrix();

    glDisable(GL_SCISSOR_TEST);

    // 3. Dòng hướng dẫn phím tắt phía dưới màn hình
    glColor4f(0.12f, 0.16f, 0.24f, 0.88f);
    glBegin(GL_QUADS);
        glVertex2f(0.0f, 0.0f); glVertex2f(windowWidth, 0.0f);
        glVertex2f(windowWidth, 44.0f); glVertex2f(0.0f, 44.0f);
    glEnd();
    glColor3f(0.20f, 0.65f, 0.85f);
    glBegin(GL_LINES);
        glVertex2f(0.0f, 44.0f); glVertex2f(windowWidth, 44.0f);
    glEnd();

    glColor3f(1.0f, 1.0f, 1.0f);
    drawText(20, 26, "W/S MOVE  |  A/D STRAFE  |  T/G ALTITUDE  |  MOUSE LOOK  |  C CAMERA", GLUT_BITMAP_HELVETICA_12);
    drawText(20, 8,  "SPACE PAUSE  |  Y DAY CYCLE  |  +/- TIME  |  P PROJECTION  |  1-3 ORTHO  |  M WIREFRAME  |  R RESET  |  ESC QUIT", GLUT_BITMAP_HELVETICA_12);

    if (paused) {
        glColor3f(0.95f, 0.2f, 0.2f);
        drawText(windowWidth * 0.46f, windowHeight * 0.52f, "PAUSED");
    }

    // THÔNG BÁO HẠ CÁNH THÀNH CÔNG (LANDED BADGE)
    if (isDroneLanded) {
        float pulse = 0.85f + 0.15f * std::sin(animTime * 6.0f);
        glColor4f(0.06f, 0.42f * pulse, 0.28f * pulse, 0.92f);
        glBegin(GL_QUADS);
            glVertex2f(windowWidth * 0.5f - 230, windowHeight * 0.5f + 40);
            glVertex2f(windowWidth * 0.5f + 230, windowHeight * 0.5f + 40);
            glVertex2f(windowWidth * 0.5f + 230, windowHeight * 0.5f + 110);
            glVertex2f(windowWidth * 0.5f - 230, windowHeight * 0.5f + 110);
        glEnd();

        glColor3f(1.0f, 0.88f, 0.25f);
        glLineWidth(2.0f);
        glBegin(GL_LINE_LOOP);
            glVertex2f(windowWidth * 0.5f - 230, windowHeight * 0.5f + 40);
            glVertex2f(windowWidth * 0.5f + 230, windowHeight * 0.5f + 40);
            glVertex2f(windowWidth * 0.5f + 230, windowHeight * 0.5f + 110);
            glVertex2f(windowWidth * 0.5f - 230, windowHeight * 0.5f + 110);
        glEnd();
        glLineWidth(1.0f);

        glColor3f(1.0f, 1.0f, 1.0f);
        drawText(windowWidth * 0.5f - 188, windowHeight * 0.5f + 85,
                 "★ DA HA CANH AN TOAN TAI HELIPAD CAO OC ★", GLUT_BITMAP_HELVETICA_12);

        glColor3f(1.0f, 0.95f, 0.35f);
        drawText(windowWidth * 0.5f - 165, windowHeight * 0.5f + 56,
                 "[CAT CANH]: BAM T HOAC SPACE DE BAY TIEP", GLUT_BITMAP_HELVETICA_12);
    }

    // 4. CẢNH BÁO KHOẢNG CÁCH VẬT CẢN (PROXIMITY WARNING)
    if (proximityDist < 6.5f) {
        float alertPulse = 0.55f + 0.45f * std::sin(animTime * 12.0f);
        glColor4f(0.88f * alertPulse + 0.1f, 0.12f, 0.12f, 0.88f);
        glBegin(GL_QUADS);
            glVertex2f(windowWidth * 0.5f - 145, windowHeight - 58);
            glVertex2f(windowWidth * 0.5f + 145, windowHeight - 58);
            glVertex2f(windowWidth * 0.5f + 145, windowHeight - 20);
            glVertex2f(windowWidth * 0.5f - 145, windowHeight - 20);
        glEnd();
        glColor3f(1.0f, 0.95f, 0.30f);
        std::ostringstream alertStr;
        alertStr << "!! CANH BAO: VAT CAN GAN (" << std::fixed << std::setprecision(1) << proximityDist << "m) !!";
        drawText(windowWidth * 0.5f - 126, windowHeight - 44, alertStr.str(), GLUT_BITMAP_HELVETICA_12);
    }

    // 5. TÂM NGẮM & ĐƯỜNG CHÂN TRỜI OSD BUỒNG LÁI FPV
    if (cameraMode == 0 && !parallelProjection) {
        float cx = windowWidth * 0.5f, cy = windowHeight * 0.5f;
        glPushMatrix();
        glTranslatef(cx, cy, 0.0f);
        glRotatef(droneRoll, 0.0f, 0.0f, 1.0f);

        // Đường chân trời FPV (Artificial Horizon)
        glColor4f(0.2f, 0.88f, 0.95f, 0.70f);
        glLineWidth(2.0f);
        glBegin(GL_LINES);
            glVertex2f(-75.0f, -dronePitch * 1.6f);
            glVertex2f(-22.0f, -dronePitch * 1.6f);
            glVertex2f( 22.0f, -dronePitch * 1.6f);
            glVertex2f( 75.0f, -dronePitch * 1.6f);
        glEnd();

        // Thang góc nghiêng pitch ladder
        glLineWidth(1.2f);
        glBegin(GL_LINES);
            glVertex2f(-32.0f, -dronePitch * 1.6f + 25.0f); glVertex2f(-12.0f, -dronePitch * 1.6f + 25.0f);
            glVertex2f( 12.0f, -dronePitch * 1.6f + 25.0f); glVertex2f( 32.0f, -dronePitch * 1.6f + 25.0f);
            glVertex2f(-32.0f, -dronePitch * 1.6f - 25.0f); glVertex2f(-12.0f, -dronePitch * 1.6f - 25.0f);
            glVertex2f( 12.0f, -dronePitch * 1.6f - 25.0f); glVertex2f( 32.0f, -dronePitch * 1.6f - 25.0f);
        glEnd();

        // Tâm ngắm FPV Reticle
        glLineWidth(1.6f);
        glBegin(GL_LINES);
            glVertex2f(0.0f, -10.0f); glVertex2f(0.0f, 10.0f);
            glVertex2f(-10.0f, 0.0f); glVertex2f(10.0f, 0.0f);
        glEnd();
        glLineWidth(1.0f);
        glPopMatrix();
    }

    // 6. VIỀN ĐỎ NHẤP NHÁY KHI VA CHẠM (COLLISION VIGNETTE)
    if (collisionFlashTimer > 0.0f) {
        float alpha = clampf(collisionFlashTimer * 0.85f, 0.0f, 0.65f);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(0.95f, 0.10f, 0.10f, alpha);
        glLineWidth(10.0f);
        glBegin(GL_LINE_LOOP);
            glVertex2f(5, 5); glVertex2f(windowWidth - 5, 5);
            glVertex2f(windowWidth - 5, windowHeight - 5); glVertex2f(5, windowHeight - 5);
        glEnd();
        glLineWidth(1.0f);
    }

    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW);
    glPopAttrib();
}

// =========================================================================================
// 11. HÀM HIỂN THỊ CHÍNH (DISPLAY CALLBACK)
// =========================================================================================
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, windowWidth, windowHeight);

    updateLightingAndSky();
    drawSkyGradient();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = (float)windowWidth / std::max(1, windowHeight);

    if (parallelProjection) {
        float half = 26.0f * zoomD;
        glOrtho(-half * aspect, half * aspect, -half, half, -450.0f, 450.0f);
    } else {
        gluPerspective(62.0 * (cameraMode == 0 ? zoomD : 1.0f), aspect, 0.1, 550.0);
    }

    setCamera();

    glPolygonMode(GL_FRONT_AND_BACK, wireframeMode ? GL_LINE : GL_FILL);

    drawSunAndMoon(drone);
    drawClouds(drone);
    drawHotAirBalloon();
    drawCity(drone);
    drawDroneShadow();
    drawDroneDownwash();
    drawDrone();

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDisable(GL_FOG);

    drawHud();

    glutSwapBuffers();
}

// =========================================================================================
// 12. LOGIC VẬT LÝ DRONE & VÒNG LẶP ĐIỀU KHIỂN
// =========================================================================================
void onDroneCollision(const Vec3& hitNormal) {
    if (invulnerableTimer > 0.0f) return;
    float curSpeed = length(droneVelocity);
    droneVelocity = hitNormal * std::max(7.5f, curSpeed * 0.75f) + Vec3(0.0f, 1.8f, 0.0f);
    cameraShakeTimer = 0.40f;
    collisionFlashTimer = 0.55f;
    invulnerableTimer = 0.45f;
}

void playerHit() {
    onDroneCollision(Vec3(0.0f, 1.0f, 0.0f));
}

void updateGame(float dt) {
    if (paused) return;

    float throttle = (keys['w'] || keys['W'] || specialKeys[GLUT_KEY_UP] ? 1.0f : 0.0f) -
                     (keys['s'] || keys['S'] || specialKeys[GLUT_KEY_DOWN] ? 1.0f : 0.0f);
    float side     = (keys['d'] || keys['D'] || specialKeys[GLUT_KEY_RIGHT] ? 1.0f : 0.0f) -
                     (keys['a'] || keys['A'] || specialKeys[GLUT_KEY_LEFT] ? 1.0f : 0.0f);
    float lift     = (keys['t'] || keys['T'] ? 1.0f : 0.0f) -
                     (keys['g'] || keys['G'] ? 1.0f : 0.0f);

    Vec3 wish = lookDirection() * throttle + strafeDirection() * side;
    if (length(wish) > 1.0f) wish = normalized(wish);

    Vec3 targetVelocity = wish * forwardSpeed + Vec3(0.0f, lift * 7.5f, 0.0f);
    if (length(targetVelocity) > forwardSpeed) targetVelocity = normalized(targetVelocity) * forwardSpeed;

    // Quán tính trôi êm ái khi nhả phím
    float blend = 1.0f - std::exp(-4.8f * dt);
    droneVelocity = droneVelocity + (targetVelocity - droneVelocity) * blend;
    drone = drone + droneVelocity * dt;

    float radius = 0.95f; // Sải cánh gọn gàng, tỉ lệ chuẩn

    // Biên bản đồ đô thị và đảo (bật nảy khi chạm biên)
    if (drone.x <= DRONE_MIN_X + radius) {
        drone.x = DRONE_MIN_X + radius + 0.1f;
        onDroneCollision(Vec3(1.0f, 0.0f, 0.0f));
    } else if (drone.x >= DRONE_MAX_X - radius) {
        drone.x = DRONE_MAX_X - radius - 0.1f;
        onDroneCollision(Vec3(-1.0f, 0.0f, 0.0f));
    }
    if (drone.z <= -DRONE_LIMIT_Z + radius) {
        drone.z = -DRONE_LIMIT_Z + radius + 0.1f;
        onDroneCollision(Vec3(0.0f, 0.0f, 1.0f));
    } else if (drone.z >= DRONE_LIMIT_Z - radius) {
        drone.z = DRONE_LIMIT_Z - radius - 0.1f;
        onDroneCollision(Vec3(0.0f, 0.0f, -1.0f));
    }

    if (drone.y < 1.2f) {
        drone.y = 1.2f;
        onDroneCollision(Vec3(0.0f, 1.0f, 0.0f));
    } else if (drone.y > 140.0f) {
        drone.y = 140.0f;
        onDroneCollision(Vec3(0.0f, -1.0f, 0.0f));
    }

    // KIỂM TRA HẠ CÁNH AN TOÀN TRÊN 2 BÃI ĐÁP TRỰC THĂNG (HELIPAD LANDING DETECTION)
    float dPad1 = std::hypot(drone.x - HELIPAD_1_X, drone.z - HELIPAD_1_Z);
    float dPad2 = std::hypot(drone.x - HELIPAD_2_X, drone.z - HELIPAD_2_Z);

    if (dPad1 < 4.8f && std::abs(drone.y - HELIPAD_1_Y) < 1.15f && length(droneVelocity) < 4.5f) {
        drone.y = HELIPAD_1_Y;
        isDroneLanded = true;
    } else if (dPad2 < 4.8f && std::abs(drone.y - HELIPAD_2_Y) < 1.15f && length(droneVelocity) < 4.5f) {
        drone.y = HELIPAD_2_Y;
        isDroneLanded = true;
    } else if (isDroneLanded) {
        // Cất cánh khi người chơi bấm T, Space hoặc W
        if (keys['t'] || keys['T'] || keys[' '] || keys['w'] || keys['W']) {
            isDroneLanded = false;
            droneVelocity.y = 5.2f;
        }
    }

    if (isDroneLanded) {
        droneVelocity = Vec3(0.0f, 0.0f, 0.0f);
        droneRoll *= std::exp(-8.0f * dt);
        dronePitch *= std::exp(-8.0f * dt);
    }

    // Nghiêng thân Drone (Roll & Pitch)
    float rollTarget = clampf(-side * 24.0f + mouseRollImpulse, -38.0f, 38.0f);
    float pitchTarget = phi * 180.0f / PI + throttle * 6.0f;
    droneRoll += (rollTarget - droneRoll) * (1.0f - std::exp(-7.0f * dt));
    dronePitch += (pitchTarget - dronePitch) * (1.0f - std::exp(-6.0f * dt));
    mouseRollImpulse *= std::exp(-7.0f * dt);

    rotorAngle = std::fmod(rotorAngle + (isDroneLanded ? 220.0f : 1600.0f) * dt, 360.0f);

    if (invulnerableTimer > 0.0f) invulnerableTimer = std::max(0.0f, invulnerableTimer - dt);

    // Tính toán khoảng cách vật cản gần nhất và phản xạ va chạm mềm
    proximityDist = 999.0f;
    int gx0 = (int)std::floor((drone.x - 24.0f) / CITY_GRID_SPACING) - 1;
    int gx1 = (int)std::floor((drone.x + 24.0f) / CITY_GRID_SPACING) + 1;
    int gz0 = (int)std::floor((drone.z - 24.0f) / CITY_GRID_SPACING) - 1;
    int gz1 = (int)std::floor((drone.z + 24.0f) / CITY_GRID_SPACING) + 1;

    bool buildingHit = false;
    Vec3 hitNormal(0.0f, 1.0f, 0.0f);

    for (int gx = gx0; gx <= gx1 && !buildingHit; gx++) {
        for (int gz = gz0; gz <= gz1 && !buildingHit; gz++) {
            if (hasBuildingAt(gx, gz)) {
                Vec3 norm;
                float dist = 999.0f;
                Building b = getBuilding(gx, gz);
                if (checkBuildingHitAndResolve(drone, radius, b, norm, dist)) {
                    buildingHit = true;
                    hitNormal = norm;
                    drone = drone + norm * (radius - dist + 0.15f);
                }
                proximityDist = std::min(proximityDist, std::max(0.0f, dist - radius));
            }
        }
    }

    // Va chạm với khối đế thương mại tại các quận đô thị (AMENITY_CITY)
    int districtX = (int)std::floor(drone.x / (CITY_GRID_SPACING * 6.0f));
    int districtZ = (int)std::floor(drone.z / (CITY_GRID_SPACING * 6.0f));
    for (int dx = districtX - 1; dx <= districtX + 1 && !buildingHit; ++dx) {
        for (int dz = districtZ - 1; dz <= districtZ + 1 && !buildingHit; ++dz) {
            if (districtAmenity(dx, dz) == AMENITY_CITY) {
                float px = dx * (CITY_GRID_SPACING * 6.0f) + 68.0f;
                float pz = dz * (CITY_GRID_SPACING * 6.0f) + 58.0f;
                Building podiumBox = { px, pz, 48.0f, 42.0f, 7.2f, 0 };
                Vec3 norm;
                float dist = 999.0f;
                if (checkBuildingHitAndResolve(drone, radius, podiumBox, norm, dist)) {
                    buildingHit = true;
                    hitNormal = norm;
                    drone = drone + norm * (radius - dist + 0.15f);
                }
                proximityDist = std::min(proximityDist, std::max(0.0f, dist - radius));
            }
        }
    }

    // Va chạm với trung tâm thương mại và vành khán đài sân vận động.
    for (int dx = districtX - 1; dx <= districtX + 1 && !buildingHit; ++dx) {
        for (int dz = districtZ - 1; dz <= districtZ + 1 && !buildingHit; ++dz) {
            int kind = districtAmenity(dx, dz);
            if (kind == AMENITY_MALL || kind == AMENITY_SCHOOL || kind == AMENITY_HOSPITAL) {
                Vec3 norm;
                float dist = 999.0f;
                Building b = amenityCollider(dx, dz, kind);
                if (checkBuildingHitAndResolve(drone, radius, b, norm, dist)) {
                    buildingHit = true;
                    hitNormal = norm;
                    drone = drone + norm * (radius - dist + 0.15f);
                }
                proximityDist = std::min(proximityDist, std::max(0.0f, dist - radius));
            } else if (kind == AMENITY_STADIUM) {
                if (sphereStadiumRingHit(drone, radius, amenityCollider(dx, dz, kind))) {
                    buildingHit = true;
                    hitNormal = normalized(Vec3(drone.x - dx * 105.0f, 0.5f, drone.z - dz * 105.0f));
                }
            }
        }
    }
    if (buildingHit) onDroneCollision(hitNormal);

    if (cameraShakeTimer > 0.0f) cameraShakeTimer = std::max(0.0f, cameraShakeTimer - dt);
    if (collisionFlashTimer > 0.0f) collisionFlashTimer = std::max(0.0f, collisionFlashTimer - dt);

    updateCityCars(dt);
}

void resetGame() {
    isDroneLanded = false;
    drone = Vec3(0.0f, 18.0f, 0.0f);
    droneVelocity = Vec3();
    droneRoll = dronePitch = rotorAngle = mouseRollImpulse = 0.0f;
    invulnerableTimer = 0.0f;
    cameraShakeTimer = collisionFlashTimer = 0.0f;
    proximityDist = 999.0f;
    smoothCamInitialized = false;
    paused = wireframeMode = false;
    cameraMode = 1;
    parallelProjection = false;
    parallelPlane = 0;
    theta = 0.0f;
    phi = 0.05f;
    cameraR = 22.0f;
    zoomD = 1.0f;
    timeOfDay = 11.5f;
}

void timer(int) {
    int now = glutGet(GLUT_ELAPSED_TIME);
    float dt = (now - lastTick) / 1000.0f;
    lastTick = now;
    dt = clampf(dt, 0.0f, 0.05f);

    updateGame(dt);
    glutPostRedisplay();
    glutTimerFunc(16, timer, 0);
}

void reshape(int w, int h) {
    windowWidth = std::max(1, w);
    windowHeight = std::max(1, h);
    glViewport(0, 0, w, h);
}

void keyboardDown(unsigned char key, int, int) {
    keys[key] = true;
    if (key == 27) std::exit(0);
    if (key == 'r' || key == 'R') resetGame();


    if (key == 'c' || key == 'C') {
        cameraMode = (cameraMode + 1) % 3;
        parallelProjection = false;
    }

    if (key == 'p' || key == 'P') parallelProjection = !parallelProjection;
    if (key == '1') { parallelPlane = 0; parallelProjection = true; }
    if (key == '2') { parallelPlane = 1; parallelProjection = true; }
    if (key == '3') { parallelPlane = 2; parallelProjection = true; }

    if (key == 'm' || key == 'M') wireframeMode = !wireframeMode;
    if (key == ' ') paused = !paused;

    if (key == '+' || key == '=') {
        timeOfDay += 1.0f;
        if (timeOfDay >= 24.0f) timeOfDay -= 24.0f;
    }
    if (key == '-' || key == '_') {
        timeOfDay -= 1.0f;
        if (timeOfDay < 0.0f) timeOfDay += 24.0f;
    }

    if (key == 'y' || key == 'Y') timeAuto = !timeAuto;

    if (key == '[') cameraR = std::max(4.0f, cameraR - 1.5f);
    if (key == ']') cameraR = std::min(45.0f, cameraR + 1.5f);
}

void keyboardUp(unsigned char key, int, int) { keys[key] = false; }
void specialDown(int key, int, int) { if (key >= 0 && key < 256) specialKeys[key] = true; }
void specialUp(int key, int, int) { if (key >= 0 && key < 256) specialKeys[key] = false; }

void mouseButton(int button, int state, int x, int y) {
    if (state == GLUT_DOWN && button == GLUT_RIGHT_BUTTON) zoomD = clampf(zoomD * 0.88f, 0.45f, 2.5f);
    if (state == GLUT_DOWN && button == 3)                  zoomD = clampf(zoomD * 0.88f, 0.45f, 2.5f);
    if (state == GLUT_DOWN && button == 4)                  zoomD = clampf(zoomD * 1.14f, 0.45f, 2.5f);
}

void mouseLookMotion(int x, int y) {
    int centerX = windowWidth / 2, centerY = windowHeight / 2;
    int dx = x - centerX, dy = y - centerY;
    if (dx == 0 && dy == 0) return;

    theta += dx * 0.0036f;
    phi = clampf(phi - dy * 0.0032f, -1.25f, 1.25f);
    mouseRollImpulse = clampf(dx * 0.45f, -32.0f, 32.0f);
    parallelProjection = false;
    glutWarpPointer(centerX, centerY);
}

void initGL() {
    glClearColor(0.99f, 0.99f, 0.98f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);

    // Ánh sáng môi trường dịu mát tạo bóng đổ sắc nét chuẩn phong cách hình ảnh mẫu 2
    GLfloat globalAmbient[] = { 0.16f, 0.17f, 0.20f, 1.0f };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmbient);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    initCityCars();
}

} // namespace CityDroneSimulation

// =========================================================================================
// 13. HÀM MAIN
// =========================================================================================
int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(CityDroneSimulation::windowWidth, CityDroneSimulation::windowHeight);
    glutInitWindowPosition(50, 30);
    glutCreateWindow("Urban Drone Citywalk - Stylized 3D Metropolis");

    CityDroneSimulation::initGL();

    glutSetCursor(GLUT_CURSOR_NONE);
    glutWarpPointer(CityDroneSimulation::windowWidth / 2, CityDroneSimulation::windowHeight / 2);
    glutIgnoreKeyRepeat(1);

    glutDisplayFunc(CityDroneSimulation::display);
    glutReshapeFunc(CityDroneSimulation::reshape);
    glutKeyboardFunc(CityDroneSimulation::keyboardDown);
    glutKeyboardUpFunc(CityDroneSimulation::keyboardUp);
    glutSpecialFunc(CityDroneSimulation::specialDown);
    glutSpecialUpFunc(CityDroneSimulation::specialUp);
    glutMouseFunc(CityDroneSimulation::mouseButton);
    glutMotionFunc(CityDroneSimulation::mouseLookMotion);
    glutPassiveMotionFunc(CityDroneSimulation::mouseLookMotion);

    CityDroneSimulation::lastTick = glutGet(GLUT_ELAPSED_TIME);
    glutTimerFunc(16, CityDroneSimulation::timer, 0);

    glutMainLoop();
    return 0;
}
