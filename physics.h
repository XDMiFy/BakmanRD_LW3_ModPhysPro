#pragma once
//   DISCRETE  — дискретный шаг 1/HZ_SLOW
//   CCD       — непрерывное обнаружение
//   MULTIRATE — обычные тела на HZ_SLOW, снаряды на HZ_FAST с синхронизацией по времени
#include "config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <random>
#include <vector>

// вектор
struct Vec2
{
    double x = 0, y = 0;
    double& operator[](int i) { return i ? y : x; }  // 0 — x, 1 — y
    double operator[](int i) const { return i ? y : x; }
};
inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double k) { return {a.x * k, a.y * k}; }
inline Vec2& operator+=(Vec2& a, Vec2 b) { return a = a + b; }
inline Vec2& operator-=(Vec2& a, Vec2 b) { return a = a - b; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

// тело
enum Group { STATIC, SLOW, FAST };
enum Mode { DISCRETE, CCD, MULTIRATE };
constexpr int bit(Group g) { return 1 << g; }
constexpr int DYNAMIC = bit(SLOW) | bit(FAST);

struct Body
{
    Vec2 pos, vel;       // центр, px; скорость, px/с
    Vec2 half;           // полуразмеры
    double invMass = 0;  // 1/m, где m = площадь/100; у неподвижных 0
    Group group = STATIC;
    double tLocal = 0;   // MULTIRATE: до какого момента тело уже посчитано
    Vec2 prev;           // где тело было в начале кадра, для следа
};

inline Body makeBody(Group g, Vec2 pos, Vec2 size, Vec2 vel = {})
{
    Body b;
    b.group = g;
    b.pos = b.prev = pos;
    b.vel = vel;
    b.half = size * 0.5;
    b.invMass = g == STATIC ? 0 : 100.0 / (size.x * size.y);
    return b;
}

// контакт (общий для всех режимов)
struct Contact
{
    Body *a, *b;
    Vec2 n;                             // нормаль от b к a
    double depth = 0;                   // > 0 — проникновение, <= 0 — зазор
    double target = 0, jn = 0, jt = 0;  // нужная скорость по нормали и накопленные импульсы
};

inline bool makeContact(Body& a, Body& b, double dt, Contact& c)
{
    Vec2 d = a.pos - b.pos, H = a.half + b.half, vr = a.vel - b.vel;  // H — сумма Минковского
    double px = H.x - std::abs(d.x), py = H.y - std::abs(d.y);
    if (std::max(px, py) <= 0 || std::min(px, py) <= -MARGIN)
        return false;  // далеко или касаются только углами
    int axis = px < py ? 0 : 1;
    double depth = axis ? py : px;
    if (depth <= 0 && std::abs(vr[axis]) > REST_THRESHOLD)
        return false;  // быстро сближаются, но ещё не коснулись
    double side = d[axis] - vr[axis] * dt;  // положение шагом раньше
    c = {&a, &b, {}, depth};
    c.n[axis] = side >= 0 ? 1.0 : -1.0;
    return true;
}

// Последовательные импульсы с трением, потом выталкивание.
inline void solveContacts(std::vector<Contact>& cs, double dt)
{
    for (Contact& c : cs)  // счет цели до итераций
    {
        double vn = dot(c.a->vel - c.b->vel, c.n);
        if (c.depth <= 0)
            c.target = c.depth / dt;  // зазор можно закрыть, но не быстрее чем за шаг
        else
            c.target = -vn > REST_THRESHOLD ? -RESTITUTION * vn : 0.0;  // отскок −e·v_n, медленнее порога — без него
    }
    for (int it = 0; it < ITERS; ++it)
        for (Contact& c : cs)
        {
            Body &a = *c.a, &b = *c.b;
            double k = a.invMass + b.invMass;
            double vn = dot(a.vel - b.vel, c.n);
            double jn = std::max(c.jn + (c.target - vn) / k, 0.0);  // контакт только толкает
            double j = jn - c.jn;
            c.jn = jn;
            a.vel += c.n * (j * a.invMass);
            b.vel -= c.n * (j * b.invMass);
            Vec2 t{-c.n.y, c.n.x};  // трение: |jt| <= MU·jn
            double jt = std::clamp(c.jt - dot(a.vel - b.vel, t) / k, -MU * c.jn, MU * c.jn);
            double dj = jt - c.jt;
            c.jt = jt;
            a.vel += t * (dj * a.invMass);
            b.vel -= t * (dj * b.invMass);
        }
    for (int it = 0; it < POS_ITERS; ++it)  // выталкивание, скорости не меняются
        for (Contact& c : cs)
        {
            Body &a = *c.a, &b = *c.b;
            double k = a.invMass + b.invMass;
            int ax = c.n.x != 0 ? 0 : 1;
            // глубина вдоль нормали иначе снаряд за серединой стенки уйдёт глубже
            double depth = a.half[ax] + b.half[ax] - c.n[ax] * (a.pos[ax] - b.pos[ax]);
            double corr = std::max(depth - SLOP, 0.0) * BETA / k;
            a.pos += c.n * (corr * a.invMass);
            b.pos -= c.n * (corr * b.invMass);
        }
}

// CCD для двух AABB, метод плит
struct Hit
{
    double t;  // от начала остатка шага
    Vec2 n;    // нормаль от b к a
};

inline bool sweptAABB(const Body& a, const Body& b, double tMax, Hit& hit)
{
    constexpr double INF = std::numeric_limits<double>::infinity();
    Vec2 d = a.pos - b.pos, v = a.vel - b.vel, H = a.half + b.half;
    double tIn = -INF, tOut = INF;
    int axis = -1;
    for (int i = 0; i < 2; ++i)
    {
        if (v[i] == 0)
        {
            if (std::abs(d[i]) >= H[i])
                return false;  // движется параллельно плите и снаружи
            continue;
        }
        double t1 = (-H[i] - d[i]) / v[i], t2 = (H[i] - d[i]) / v[i];
        if (t1 > t2)
            std::swap(t1, t2);
        if (t1 > tIn)
        {
            tIn = t1;
            axis = i;  // нормаль по оси, где вход случился позже
        }
        tOut = std::min(tOut, t2);
        if (tIn > tOut)
            return false;
    }
    if (axis < 0 || tIn < 0 || tIn > tMax)
        return false;  // уже пересекаются или удар после конца шага
    hit.t = tIn;
    hit.n = {};
    hit.n[axis] = v[axis] > 0 ? -1.0 : 1.0;
    return true;
}

// Удар: j = −(1+e)·v_n / (1/m_a + 1/m_b), у неподвижного 1/m = 0.
inline void respond(Body& a, Body& b, Vec2 n)
{
    double vn = dot(a.vel - b.vel, n);
    if (vn >= 0)
        return;  // уже расходятся
    double e = -vn < REST_THRESHOLD ? 0.0 : RESTITUTION;
    double j = -(1 + e) * vn / (a.invMass + b.invMass);
    a.vel += n * (j * a.invMass);
    b.vel -= n * (j * b.invMass);
}

// мир
constexpr Vec2 LAUNCHER{70, 500};                 // откуда стреляют в арене
constexpr double SHOT_ANGLES[] = {-1, -3, -5, -2, -4};  // градусы, минус — вверх
constexpr double SHOT_FIRST = 1.0, SHOT_EVERY = 0.5;    // k-й автовыстрел в 1.0 + 0.5·k с

struct World
{
    std::vector<Body> statics, slow, fast;  // три группы тел
    std::vector<Body*> all;                 // все тела в постоянном порядке
    Mode mode = DISCRETE;
    double g = G;
    bool syncSlow = true;   // MULTIRATE: подтягивать обычное тело ко времени удара
    long tick = 0;          // MULTIRATE: номер тика, время = tick / HZ_FAST
    int shots = 0;
    bool autoFire = false;  // арена: снаряды вылетают сами

    // статистика для бенча и заголовка окна
    long steps = 0, fastTicks = 0, ccdEvents = 0, ccdMaxEvents = 0, capHits = 0;
    double fastMs = 0, slowMs = 0;  // MULTIRATE: время на тики снарядов и на шаги обычных тел

    World() = default;
    World(const World&) = delete;  // all указывает внутрь своих же векторов
    World(World&&) = default;
    World& operator=(World&&) = default;

    void index()
    {
        all.clear();
        for (auto* v : {&statics, &slow, &fast})
            for (Body& b : *v)
                all.push_back(&b);
    }

    // Запустить следующий снаряд (по кругу).
    void fire()
    {
        Body& p = fast[shots % fast.size()];
        double a = SHOT_ANGLES[shots % std::size(SHOT_ANGLES)] * std::numbers::pi / 180;
        p.pos = p.prev = LAUNCHER;
        p.vel = {PROJECTILE_SPEED * std::cos(a), PROJECTILE_SPEED * std::sin(a)};
        p.tLocal = tick * DT_FAST;
        ++shots;
    }

    // Один шаг 1/HZ_SLOW в текущем режиме.
    void step()
    {
        if (autoFire && shots < int(fast.size()) && steps * DT_SLOW >= SHOT_FIRST + SHOT_EVERY * shots - 1e-9)
            fire();
        for (Body* b : all)
            b->prev = b->pos;
        ++steps;
        if (mode == DISCRETE)
            stepDiscrete(DT_SLOW);
        else if (mode == CCD)
            stepCCD(DT_SLOW);
        else
            stepMultirate();
    }

    void resolveOverlaps(int movers, double dt, double tNow = -1, int with = bit(STATIC) | DYNAMIC)
    {
        std::vector<Contact> cs;
        for (size_t i = 0; i < all.size(); ++i)
        {
            Body& a = *all[i];
            if (!(movers & bit(a.group)))
                continue;
            for (size_t j = 0; j < all.size(); ++j)
            {
                Body& b = *all[j];
                if (i == j || !(with & bit(b.group)) || ((movers & bit(b.group)) && j < i))
                    continue;  // каждую пару один раз
                if (tNow >= 0 && b.group == SLOW && b.tLocal < tNow)
                    continue;
                Contact c;
                if (makeContact(a, b, dt, c))
                    cs.push_back(c);
            }
        }
        solveContacts(cs, dt);
    }

    // DISCRETE: гравитация, сдвиг, проверка пересечений в конце шага
    void stepDiscrete(double dt)
    {
        for (Body* b : all)
            if (b->invMass > 0)
            {
                b->vel.y += g * dt;  // полунеявный Эйлер: скорость, положение
                b->pos += b->vel * dt;
            }
        resolveOverlaps(DYNAMIC, dt);
    }

    // CCD: цикл событий внутри шага
    void advanceAll(double t)
    {
        for (Body* b : all)
            if (b->invMass > 0)
                b->pos += b->vel * t;
    }

    void stepCCD(double dt)
    {
        for (Body* b : all)  // 1. гравитация сразу в скорость на начало шага
            if (b->invMass > 0)
                b->vel.y += g * dt;
        double rem = dt;
        int ev = 0;
        for (; ev < MAX_EVENTS && rem > 1e-9; ++ev)
        {
            // 2. самый ранний удар среди всех пар на остатке шага
            double tBest = rem;
            Hit best{};
            Body *A = nullptr, *B = nullptr;
            for (size_t i = 0; i < all.size(); ++i)
                for (size_t j = i + 1; j < all.size(); ++j)
                {
                    Body &a = *all[i], &b = *all[j];
                    if (a.invMass == 0 && b.invMass == 0)
                        continue;
                    Hit h;
                    if (!sweptAABB(a, b, tBest, h))
                        continue;
                    if (-dot(a.vel - b.vel, h.n) * rem < CCD_MIN_MOVE)
                        continue;
                    tBest = h.t;
                    best = h;
                    A = &a;
                    B = &b;
                }
            if (!A)
                break;  // ударов больше нет
            advanceAll(tBest);        // 3. все тела к моменту удара
            respond(*A, *B, best.n);  // 4. импульс обоим
            double k = A->invMass + B->invMass;
            A->pos += best.n * (SKIN * A->invMass / k);
            B->pos -= best.n * (SKIN * B->invMass / k);
            rem -= tBest;             // 5. считаем остаток шага, возможен ли ещё удар
        }
        ccdEvents += ev;
        ccdMaxEvents = std::max<long>(ccdMaxEvents, ev);
        if (ev == MAX_EVENTS)
            ++capHits;  // лимит кончился, остаток шага идёт без CCD
        advanceAll(rem);
        resolveOverlaps(DYNAMIC, dt);  // покоящиеся контакты и то, что swept-тест не видит
    }

    // Догнать момент t шагом переменной длины
    void advanceTo(Body& s, double t)
    {
        double d = t - s.tLocal;
        if (d <= 0)
            return;
        s.vel.y += g * d;
        s.pos += s.vel * d;
        s.tLocal = t;
    }

    // Задевает ли снаряд f обычное тело s там, где s было бы в момент t
    bool overlapsPredicted(const Body& f, const Body& s, double t) const
    {
        double d = t - s.tLocal;
        Vec2 p = s.pos + Vec2{s.vel.x, s.vel.y + g * d} * d;
        Vec2 H = f.half + s.half;
        return std::abs(f.pos.x - p.x) < H.x && std::abs(f.pos.y - p.y) < H.y;
    }

    // Аккумулятор в main выдаёт шаги 1/HZ_SLOW
    void stepMultirate()
    {
        using Clock = std::chrono::steady_clock;
        auto ms = [](auto d) { return std::chrono::duration<double, std::milli>(d).count(); };
        for (int k = 0; k < RATIO; ++k)
        {
            ++tick;
            ++fastTicks;
            double tNow = tick * DT_FAST;  // из целого счётчика
            auto t0 = Clock::now();
            // 1. снаряды: шаг 1/HZ_FAST
            for (Body& f : fast)
            {
                f.vel.y += g * DT_FAST;
                f.pos += f.vel * DT_FAST;
                f.tLocal = tNow;
            }
            // 2. обычные тела, которые снаряд задевает в момент tNow подтягиваются до tNow
            if (syncSlow)
                for (Body& f : fast)
                    for (Body& s : slow)
                        if (s.tLocal < tNow && overlapsPredicted(f, s, tNow))
                            advanceTo(s, tNow);
            // 3. контакты снарядов = оба тела уже в моменте tNow
            resolveOverlaps(bit(FAST), DT_FAST, syncSlow ? tNow : -1);
            auto t1 = Clock::now();
            fastMs += ms(t1 - t0);
            // 4. каждый RATIO тик - шаг обычных тел
            if (tick % RATIO == 0)
            {
                for (Body& s : slow)
                    advanceTo(s, tNow);
                resolveOverlaps(bit(SLOW), DT_SLOW, -1, bit(STATIC) | bit(SLOW));
                slowMs += ms(Clock::now() - t1);
            }
        }
    }
};

// сцены
enum Scene { WALL, ARENA, PYRAMID };
constexpr double WALL_X = 600, WALL_Y = 300;  // центр стенки
constexpr double FLOOR_Y = H - 40;            // верх пола арены

inline World makeWorld(Scene sc, Mode m, double speed = PROJECTILE_SPEED)
{
    World w;
    w.mode = m;
    if (sc == WALL)
    {
        w.g = 0;
        w.statics.push_back(makeBody(STATIC, {WALL_X, WALL_Y}, {WALL_THICKNESS, 200}));
        double step = speed * DT_SLOW;
        double x0 = WALL_X - step / 2 - step * std::floor((WALL_X - 150) / step);
        w.fast.push_back(makeBody(FAST, {x0, WALL_Y}, {8, 8}, {speed, 0}));
    }
    else
    {
        w.statics.push_back(makeBody(STATIC, {W / 2.0, H - 20.0}, {double(W), 40}));
        w.statics.push_back(makeBody(STATIC, {W / 2.0, 20}, {double(W), 40}));
        w.statics.push_back(makeBody(STATIC, {20, H / 2.0}, {40, double(H)}));
        w.statics.push_back(makeBody(STATIC, {W - 20.0, H / 2.0}, {40, double(H)}));
        for (int r = 0; r < 4; ++r)
            for (int i = 0; i < 4 - r; ++i)
                w.slow.push_back(makeBody(SLOW, {700 + (i - (3 - r) / 2.0) * 21, FLOOR_Y - 10 - r * 20}, {20, 20}));
        for (int i = 0; i < N_PROJECTILES; ++i)
            w.fast.push_back(makeBody(FAST, {60 + 14.0 * i, FLOOR_Y - 4}, {8, 8}));
        if (sc == ARENA)
        {
            w.autoFire = true;
            for (Vec2 p : {Vec2{220, 320}, {360, 420}, {500, 320}, {640, 420}, {780, 320}, {880, 420}})
                w.statics.push_back(makeBody(STATIC, p, {40, 40}));
            std::mt19937 rng(SEED);
            std::uniform_int_distribution<int> size(16, 24), jitter(-10, 10);
            for (int i = 0; i < N_RANDOM_BLOCKS; ++i)
            {
                double s = size(rng);
                Vec2 p{140.0 + (i % 8) * 100 + jitter(rng), 70.0 + (i / 8) * 45};
                w.slow.push_back(makeBody(SLOW, p, {s, s}));
            }
        }
    }
    w.index();
    return w;
}
