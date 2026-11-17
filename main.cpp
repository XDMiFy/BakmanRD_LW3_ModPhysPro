// Окно SDL2
// ./ModPhysPro - интерактивная сцена
// ./ModPhysPro --bench - замеры
#include "physics.h"

#include <SDL.h>

#include <cstdint>
#include <cstdio>
#include <cstring>

static const char* MODE_NAME[] = {"DISCRETE", "CCD", "MULTIRATE"};
static const char* SCENE_NAME[] = {"Стенка", "Арена", "Пирамида"};

// отрисовка
static SDL_FRect rectAt(const Body& b, Vec2 p)
{
    return {float(p.x - b.half.x), float(p.y - b.half.y), float(2 * b.half.x), float(2 * b.half.y)};
}

static void render(SDL_Renderer* r, const World& w)
{
    SDL_SetRenderDrawColor(r, 18, 18, 24, 255);
    SDL_RenderClear(r);
    for (const Body& b : w.statics)  // неподвижные серые
    {
        SDL_FRect q = rectAt(b, b.pos);
        SDL_SetRenderDrawColor(r, 110, 110, 125, 255);
        SDL_RenderFillRectF(r, &q);
    }
    for (const Body& b : w.slow)  // обычные синие
    {
        SDL_FRect q = rectAt(b, b.pos);
        SDL_SetRenderDrawColor(r, 70, 140, 220, 255);
        SDL_RenderFillRectF(r, &q);
        SDL_SetRenderDrawColor(r, 25, 55, 105, 255);
        SDL_RenderDrawRectF(r, &q);
    }
    for (const Body& b : w.fast)  // снаряды красные, жёлтым показ где был в начале кадра и путь за кадр
    {
        SDL_FRect was = rectAt(b, b.prev), now = rectAt(b, b.pos);
        SDL_SetRenderDrawColor(r, 255, 200, 0, 255);
        SDL_RenderDrawRectF(r, &was);
        SDL_RenderDrawLineF(r, float(b.prev.x), float(b.prev.y), float(b.pos.x), float(b.pos.y));
        SDL_SetRenderDrawColor(r, 240, 70, 60, 255);
        SDL_RenderFillRectF(r, &now);
    }
}

// интерактивный режим
static int runWindow()
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
        return std::printf("SDL_Init: %s\n", SDL_GetError()), 1;
    SDL_Window* win = SDL_CreateWindow("ModPhysPro", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, W, H, 0);
    SDL_Renderer* r = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    Scene scene = WALL;
    Mode mode = DISCRETE;
    bool sync = true, paused = false, running = true;
    World w = makeWorld(scene, mode);
    auto restart = [&]  // всегда с одного и того же начального состояния
    {
        w = makeWorld(scene, mode);
        w.syncSlow = sync;
    };

    const double freq = double(SDL_GetPerformanceFrequency());
    Uint64 prev = SDL_GetPerformanceCounter(), lastTitle = prev;
    double acc = 0, statMs = 0;  // накопленное время кадра; время физики для заголовка
    long statSteps = 0, statFast = 0, statEvents = 0;

    while (running)
    {
        bool stepOnce = false;
        for (SDL_Event e; SDL_PollEvent(&e);)
        {
            if (e.type == SDL_QUIT)
                running = false;
            if (e.type != SDL_KEYDOWN)
                continue;
            switch (e.key.keysym.sym)
            {
                case SDLK_1:
                case SDLK_2:
                case SDLK_3:
                    mode = Mode(e.key.keysym.sym - SDLK_1);
                    restart();
                    break;
                case SDLK_TAB:
                    scene = Scene((scene + 1) % 3);
                    restart();
                    break;
                case SDLK_r: restart(); break;
                case SDLK_SPACE: paused = !paused; break;
                case SDLK_n: stepOnce = true; break;
                case SDLK_f:
                    if (scene != WALL)
                        w.fire();
                    break;
                case SDLK_s: w.syncSlow = sync = !sync; break;
                case SDLK_ESCAPE: running = false; break;
            }
        }

        // Фиксированный шаг: время кадра копится в acc, физика получает только DT_SLOW.
        Uint64 now = SDL_GetPerformanceCounter();
        acc += std::min((now - prev) / freq, 0.25);  // не больше 0.25 с, чтобы не было «спирали смерти»
        prev = now;
        if (paused)
            acc = stepOnce ? DT_SLOW : 0;  // N на паузе — ровно один шаг
        while (acc >= DT_SLOW)
        {
            long f0 = w.fastTicks, e0 = w.ccdEvents;
            Uint64 t0 = SDL_GetPerformanceCounter();
            w.step();
            statMs += (SDL_GetPerformanceCounter() - t0) * 1000.0 / freq;
            ++statSteps;
            statFast += w.fastTicks - f0;
            statEvents += w.ccdEvents - e0;
            acc -= DT_SLOW;
        }
        render(r, w);
        SDL_RenderPresent(r);

        if ((now - lastTitle) / freq > 0.25)
        {
            char title[256];
            double n = std::max<long>(statSteps, 1);
            std::snprintf(title, sizeof title,
                          "%s %d/%d Гц | %s | %.3f мс/шаг | шагов %d Гц на кадр: %.1f | событий CCD на шаг: %.2f | "
                          "sync %s%s",
                          MODE_NAME[mode], HZ_SLOW, HZ_FAST, SCENE_NAME[scene], statMs / n, HZ_FAST, statFast / n,
                          statEvents / n, sync ? "on" : "off", paused ? " | ПАУЗА" : "");
            SDL_SetWindowTitle(win, title);
            lastTitle = now;
            statMs = 0;
            statSteps = statFast = statEvents = 0;
        }
    }
    SDL_Quit();
    return 0;
}

// бенч и тесты без окна
static uint64_t hashState(const World& w)
{
    uint64_t h = 1469598103934665603ull;
    auto mix = [&](double x)
    {
        uint64_t u;
        std::memcpy(&u, &x, 8);
        h = (h ^ u) * 1099511628211ull;
    };
    for (const auto* v : {&w.slow, &w.fast})
        for (const Body& b : *v)
        {
            mix(b.pos.x);
            mix(b.pos.y);
            mix(b.vel.x);
            mix(b.vel.y);
        }
    return h;
}

static double speedOf(const Body& b) { return std::hypot(b.vel.x, b.vel.y); }

// sweptAABB: A(0,0) 8×8 v=(100,0) и B(50,0) 10×10 → t = 0.41, n = (−1,0); если B на y=20, удара нет
static bool testSwept()
{
    Body a = makeBody(SLOW, {0, 0}, {8, 8}, {100, 0}), b = makeBody(STATIC, {50, 0}, {10, 10});
    Hit h{};
    bool ok = sweptAABB(a, b, 1, h) && std::abs(h.t - 0.41) < 1e-12 && h.n.x == -1 && h.n.y == 0;
    b.pos.y = 20;
    return ok && !sweptAABB(a, b, 1, h);
}

// 100 выстрелов в стенку
// старт сдвигается на v·Δt/100 (Δt — шаг снаряда в режиме)
// Возвращает число пропусков, оно же процент; err — худшее отклонение отскока от e·v
static int wallSeries(Mode m, double v, double& err)
{
    int miss = 0;
    err = 0;
    double dt = m == MULTIRATE ? DT_FAST : DT_SLOW;
    for (int k = 0; k < 100; ++k)
    {
        World w = makeWorld(WALL, m, v);
        w.fast[0].pos.x += k * v * dt / 100;
        for (int f = 0; f < 40; ++f)
            w.step();
        const Body& p = w.fast[0];
        if (p.pos.x > WALL_X)
            ++miss;
        else
            err = std::max(err, std::abs(-p.vel.x - RESTITUTION * v) / (RESTITUTION * v));
    }
    return miss;
}

struct SyncResult
{
    double posMean = 0, posMax = 0, velMax = 0, momMax = 0;
};
static SyncResult syncTest(Mode m, bool sync)
{
    const double m1 = 0.64, v1 = -2000, m2 = 4, v2 = 200, e = RESTITUTION, vRel = v2 - v1;
    const double v1a = ((m1 - e * m2) * v1 + (1 + e) * m2 * v2) / (m1 + m2);
    const double v2a = ((m2 - e * m1) * v2 + (1 + e) * m1 * v1) / (m1 + m2);
    const int frames = 30;
    const double T = frames * DT_SLOW;
    SyncResult res;
    for (int k = 0; k < 24; ++k)
    {
        double gap = 100 + k * vRel * DT_SLOW / 24;
        World w;
        w.mode = m;
        w.g = 0;
        w.syncSlow = sync;
        w.slow.push_back(makeBody(SLOW, {300, 300}, {20, 20}, {v2, 0}));
        w.fast.push_back(makeBody(FAST, {300 + 10 + gap + 4, 300}, {8, 8}, {v1, 0}));
        w.index();
        for (int f = 0; f < frames; ++f)
            w.step();
        double tc = gap / vRel;
        double xb = 300 + v2 * tc + v2a * (T - tc);  // x_b(T) = x_b0 + v2·t_c + v2'·(T − t_c)
        const Body &b = w.slow[0], &p = w.fast[0];
        double err = std::abs(b.pos.x - xb);
        res.posMean += err / 24;
        res.posMax = std::max(res.posMax, err);
        res.velMax = std::max({res.velMax, std::abs(b.vel.x - v2a), std::abs(p.vel.x - v1a)});
        res.momMax = std::max(res.momMax, std::abs(m1 * p.vel.x + m2 * b.vel.x - (m1 * v1 + m2 * v2)));
    }
    return res;
}

// Самое глубокое взаимное проникновение тел
static double maxPenetration(const World& w)
{
    double worst = 0;
    for (size_t i = 0; i < w.all.size(); ++i)
        for (size_t j = i + 1; j < w.all.size(); ++j)
        {
            const Body &a = *w.all[i], &b = *w.all[j];
            if (a.invMass == 0 && b.invMass == 0)
                continue;
            Vec2 d = a.pos - b.pos, Hs = a.half + b.half;
            worst = std::max(worst, std::min(Hs.x - std::abs(d.x), Hs.y - std::abs(d.y)));
        }
    return worst;
}

static bool inArena(const Body& b)
{
    return std::isfinite(b.pos.x) && std::isfinite(b.pos.y) && b.pos.x > 40 && b.pos.x < W - 40 &&
           b.pos.y > 40 && b.pos.y < H - 40;
}

static int bench()
{
    std::printf("Release-замеры; кадр = один шаг 1/%d с (в MULTIRATE — %d тиков 1/%d с)\n\n", HZ_SLOW, RATIO, HZ_FAST);
    bool sweptOk = testSwept();
    std::printf("[тест] sweptAABB: %s\n\n", sweptOk ? "OK" : "FAIL");

    std::printf("[быстрые объекты] Стенка %g px, снаряд 8 px, 100 выстрелов, e = %g\n", WALL_THICKNESS, RESTITUTION);
    std::printf("%-10s %22s %22s\n", "режим", "2000 px/с: пропуск/отскок", "5000 px/с: пропуск/отскок");
    for (Mode m : {DISCRETE, CCD, MULTIRATE})
    {
        double e1, e2;
        int m1 = wallSeries(m, 2000, e1), m2 = wallSeries(m, 5000, e2);
        std::printf("%-10s %12d%% / %.1e %13d%% / %.1e\n", MODE_NAME[m], m1, e1, m2, e2);
    }

    std::printf("\n[синхронизация] удар снаряд → блок без g, 24 фазы: ошибка x блока mean/max px, "
                "макс |Δv| px/с, макс |Δp|\n");
    auto printSync = [](const char* name, SyncResult s)
    { std::printf("%-17s %8.3f / %8.3f   %9.2e   %9.2e\n", name, s.posMean, s.posMax, s.velMax, s.momMax); };
    printSync("DISCRETE", syncTest(DISCRETE, true));
    printSync("CCD", syncTest(CCD, true));
    printSync("MULTIRATE sync", syncTest(MULTIRATE, true));
    printSync("MULTIRATE no sync", syncTest(MULTIRATE, false));

    std::printf("\n[устойчивость] пирамида 4-3-2-1 (20×20): 10 с покоя | 5 выстрелов по %g px/с, 12 с\n",
                PROJECTILE_SPEED);
    std::printf("%-10s %14s %14s %16s %14s\n", "режим", "vmax после 5с", "сдвиг верха", "блоков в арене",
                "vmax в конце");
    for (Mode m : {DISCRETE, CCD, MULTIRATE})
    {
        World w = makeWorld(PYRAMID, m);
        Vec2 top0 = w.slow[9].pos;  // верхний блок пирамиды
        double vRest = 0;
        for (int f = 0; f < 10 * HZ_SLOW; ++f)
        {
            w.step();
            if (f >= 5 * HZ_SLOW)
                for (int i = 0; i < 10; ++i)
                    vRest = std::max(vRest, speedOf(w.slow[i]));
        }
        Vec2 d = w.slow[9].pos - top0;

        World s = makeWorld(PYRAMID, m);
        double vEnd = 0;
        for (int f = 0; f < 12 * HZ_SLOW; ++f)
        {
            if (f % HZ_SLOW == 0 && f / HZ_SLOW >= 1 && f / HZ_SLOW <= 5)
                s.fire();  // выстрелы на 1–5 с
            s.step();
            if (f >= 11 * HZ_SLOW)
                for (int i = 0; i < 10; ++i)
                    vEnd = std::max(vEnd, speedOf(s.slow[i]));
        }
        int inside = 0;
        for (int i = 0; i < 10; ++i)
            inside += inArena(s.slow[i]);
        std::printf("%-10s %10.2f px/с %11.3f px %13d/10 %10.2f px/с\n", MODE_NAME[m], vRest, std::hypot(d.x, d.y),
                    inside, vEnd);
    }

    const int FRAMES = 60 * HZ_SLOW, WARMUP = 2 * HZ_SLOW, RUNS = 5;
    std::printf("\n[затраты + детерминированность] Арена: 10 статических, 50 обычных, 5 снарядов; %d кадров, "
                "прогрев %d, %d прогонов\n",
                FRAMES, WARMUP, RUNS);
    for (Mode m : {DISCRETE, CCD, MULTIRATE})
    {
        std::vector<double> means, pooled;
        uint64_t hash0 = 0;
        bool sameHash = true;
        World w;
        for (int run = 0; run < RUNS; ++run)
        {
            w = makeWorld(ARENA, m);
            double sum = 0;
            for (int f = 0; f < FRAMES; ++f)  // снаряды арены вылетают сами на 1–3 с
            {
                auto t0 = std::chrono::steady_clock::now();
                w.step();
                double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
                if (f >= WARMUP)
                {
                    pooled.push_back(ms);
                    sum += ms;
                }
            }
            means.push_back(sum / (FRAMES - WARMUP));
            uint64_t h = hashState(w);
            if (run == 0)
                hash0 = h;
            sameHash = sameHash && h == hash0;
        }
        std::sort(means.begin(), means.end());
        std::sort(pooled.begin(), pooled.end());
        std::printf("%-10s мс/кадр: среднее %.4f (медиана %d прогонов), p50 %.4f, p95 %.4f, max %.4f\n", MODE_NAME[m],
                    means[RUNS / 2], RUNS, pooled[pooled.size() / 2], pooled[pooled.size() * 95 / 100],
                    pooled.back());
        if (m == CCD)
            std::printf("%-10s событий CCD на шаг: среднее %.3f, макс %ld, срабатываний лимита %ld\n", "",
                        double(w.ccdEvents) / w.steps, w.ccdMaxEvents, w.capHits);
        if (m == MULTIRATE)
            std::printf("%-10s шагов %d Гц на кадр: %.2f; тик fast %.4f мс, шаг slow %.4f мс\n", "", HZ_FAST,
                        double(w.fastTicks) / w.steps, w.fastMs / w.fastTicks, w.slowMs / w.steps);
        int inside = 0;
        for (const Body* b : w.all)
            inside += b->group != STATIC && inArena(*b);
        std::printf("%-10s хэш %016llx, совпал в %d прогонах: %s\n", "", (unsigned long long)hash0, RUNS,
                    sameHash ? "да" : "НЕТ");
        std::printf("%-10s в конце: подвижных тел в арене %d/%zu, макс. взаимное проникновение %.2f px\n", "", inside,
                    w.slow.size() + w.fast.size(), maxPenetration(w));
    }
    return sweptOk ? 0 : 1;
}

int main(int argc, char** argv)
{
    if (argc > 1 && std::strcmp(argv[1], "--bench") == 0)
        return bench();
    return runWindow();
}
