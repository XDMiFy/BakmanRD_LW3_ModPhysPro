#pragma once

// физика
constexpr double G = 980.0;                // px/с², 1 м = 100 px, ось y вниз
constexpr double RESTITUTION = 0.4;        // коэффициент восстановления e
constexpr double PROJECTILE_SPEED = 2000;  // px/с
constexpr double WALL_THICKNESS = 10;      // стенка в сцене «Стенка», px

// частоты
constexpr int HZ_SLOW = 60;   // DISCRETE и CCD; обычные тела в MULTIRATE
constexpr int HZ_FAST = 240;  // снаряды в MULTIRATE
static_assert(HZ_FAST % HZ_SLOW == 0, "HZ_FAST должна делиться на HZ_SLOW");
constexpr int RATIO = HZ_FAST / HZ_SLOW;  // тиков снарядов на один шаг обычных тел
constexpr double DT_SLOW = 1.0 / HZ_SLOW, DT_FAST = 1.0 / HZ_FAST;

// решатель контактов
constexpr int ITERS = 10;              // итерации импульсов
constexpr int POS_ITERS = 4;           // итерации выталкивания
constexpr double MU = 0.4;             // трение
constexpr double SLOP = 0.01;          // допустимое проникновение, px
constexpr double BETA = 0.8;           // доля выталкивания за итерацию
constexpr double REST_THRESHOLD = 50;  // px/с, при более медленном сближении отскока нет
constexpr double MARGIN = 1.0;         // px, медленные пары с таким зазором тоже идут в решатель

// CCD
constexpr int MAX_EVENTS = 16;        // событий на шаг
constexpr double SKIN = 1e-3;         // зазор после удара, px
constexpr double CCD_MIN_MOVE = 0.5;  // px за шаг, более медленное сближение отдаём решателю

// арена
constexpr int W = 960, H = 600;      // окно, px
constexpr int N_RANDOM_BLOCKS = 40;  // плюс 10 в пирамиде, всего 50 обычных тел
constexpr int N_PROJECTILES = 5;
constexpr unsigned SEED = 42;        // расстановка одинаковая при каждом запуске
