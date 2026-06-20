#include "AUVPhysicsAPI.h"
#include <cstring>
#include <cmath>

static AUVState  g_state;
static bool      g_stateInit = false;

extern "C" void State_Reset() {
    std::memset(&g_state, 0, sizeof(AUVState));
    g_state.orientation = {0,0,0,1};
    g_stateInit = true;
}

extern "C" void State_IntegrateLinear(AUVState* s, float dt) {
    if (!s) return;
    s->position.x += s->linearVelocity.x * dt;
    s->position.y += s->linearVelocity.y * dt;
    s->position.z += s->linearVelocity.z * dt;
}

extern "C" void State_IntegrateAngular(AUVState* s, float dt) {
    if (!s) return;
    float hx = s->angularVelocity.x * dt * 0.5f;
    float hy = s->angularVelocity.y * dt * 0.5f;
    float hz = s->angularVelocity.z * dt * 0.5f;
    float qw = s->orientation.w, qx = s->orientation.x, qy = s->orientation.y, qz = s->orientation.z;
    s->orientation.w = qw - qx*hx - qy*hy - qz*hz;
    s->orientation.x = qx + qw*hx + qy*hz - qz*hy;
    s->orientation.y = qy + qw*hy - qx*hz + qz*hx;
    s->orientation.z = qz + qw*hz + qx*hy - qy*hx;
    float len = std::sqrt(s->orientation.w*s->orientation.w +
                          s->orientation.x*s->orientation.x +
                          s->orientation.y*s->orientation.y +
                          s->orientation.z*s->orientation.z);
    if (len > 1e-12f) {
        s->orientation.w /= len;
        s->orientation.x /= len;
        s->orientation.y /= len;
        s->orientation.z /= len;
    }
}
