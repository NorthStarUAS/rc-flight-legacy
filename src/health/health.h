// System health/status monitoring module

#pragma once

class health_t {
public:
    void init();
    void update();
};

extern health_t health;
