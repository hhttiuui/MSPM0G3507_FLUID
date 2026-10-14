/*
 * fluid.c —— FLIP/PIC 流体求解器
 *
 * 逐句移植自 CH32 工程的 User/SandSim.c，与 CH32 原版的差异只有三处：
 *   1. 删掉 screen[32] 和 screen_update()（显示交给 fluid_demo.c）
 *   2. _IQ(<运行期整型表达式>) 换成 _IQI(...)，避免运行期走软件浮点
 *   3. cellParticleCountPrefix 由 uint8_t 改成 uint16_t
 *      （原版靠"粒子数 130 < 255"侥幸不溢出，粒子一多就会出错）
 *
 * 其余算法、参数、边界处理、循环顺序全部保持原样，方便出问题时逐行对照。
 */
#include <string.h>
#include "fluid.h"

#define PUSH_ITER           1U
#define GRID_ITER_DEFAULT   12U

/* ---- 把热函数放到 SRAM 执行 ----
   MSPM0 在 80MHz 下 flash 是 2 个等待周期，而 M0+ 没有指令缓存，
   每取一条指令都要多等。放到 0 等待周期的 SRAM 执行通常能明显提速，
   代价是多占几 KB SRAM（链接脚本里已有 .TI.ramfunc 段）。
   1 = 启用; 0 = 关闭（用于 A/B 对比） */
#define FLUID_HOT_RAM   1

#if FLUID_HOT_RAM
#define FLUID_FN   __attribute__((section(".TI.ramfunc")))
#else
#define FLUID_FN
#endif

/* 访问粒子位置/速度用的便捷宏 */
#define XID(n)       (2U * (n))
#define YID(n)       (2U * (n) + 1U)
#define INDEX(x, y)  ((x) * CellNumY + (y))

/* ---- 硬件开方（MATHACL SQRT）的运行时开关 ----
   在这里定义、在 fluid_iqmath.h 里 extern 声明，保证全工程只有一份。
   为什么要开关而不是写死：硬件 SQRT 的 sfactor 要按输入现算，
   万一算错，_IQsqrt 会把整个仿真算飞（现象就是屏幕全亮/乱码）。
   启动时先自检（IQ_SqrtSelfTest），不对就自动退回软件开方。
   默认 0 = 先用软件，自检通过后由自检函数置 1 —— 即使自检没被调用，
   也只会慢一点，绝不会算错。 */
#if (IQ_MPY_MODE == 2)
uint8_t g_iqMpyUseHw    = 1u;   /* 1 = 硬件乘法；0 = 自检不过，已退回软件 */
#endif

#if IQ_SQRT_MODE
uint8_t g_iqSqrtUseHw   = 0u;   /* 0 = 软件开方；1 = 硬件开方 */
uint8_t g_iqSqrtSfactor = 31u;  /* 诊断：自检点算出的 sfactor */
uint8_t g_iqSqrtIter    = 31u;  /* 诊断：NUMITER */
#endif

static _iq invertSpacing;

static _iq particlePos[NumberOfParticles * 2U];
static _iq particleVel[NumberOfParticles * 2U];

static _iq uVel[CellCount];
static _iq vVel[CellCount];
static _iq uPrev[CellCount];
static _iq vPrev[CellCount];
static _iq uWeight[CellCount];
static _iq vWeight[CellCount];
static _iq particleDensity[CellCount];
/* 固体掩码：**只取 0 或 1**（不是 Q24 的 1.0！）。
   因为只有这两种值，压力求解里"乘以掩码"可以直接用整数条件判断代替，
   省掉 4 次 Q24 乘法 —— 见 compute_grid_forces() 里的说明。 */
static uint8_t solidMask[CellCount];
static uint8_t cellType[CellCount];
static _iq particleRestDensity = 0;

static _iq minX = Spacing + ParticleRadius;
static _iq minY = Spacing + ParticleRadius;
static _iq maxX;
static _iq maxY;

#define CellPrefixCountSize (CellCount + 1U)

static uint16_t cellParticleCountPrefix[CellPrefixCountSize];
static uint8_t  particlePosId[NumberOfParticles];

static uint32_t s_frameCount;

/* 压力求解迭代次数，运行期可调（便于一次跑出"耗时 vs 迭代次数"曲线） */
static uint32_t s_gridIter = GRID_ITER_DEFAULT;

/* 时间步长，运行期可调（便于一次烧录试出最手感最好且不发散的值） */
static _iq s_dt = FLUID_DT;

/* ==================== 小工具 ==================== */

static int FLUID_FN clamp_index(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static _iq FLUID_FN clampf_local(_iq value, _iq min_value, _iq max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/* 四周一圈格子设为固体（solidMask = 0） */
static void FLUID_FN setup_solid_mask(void)
{
    for (unsigned int x = 0; x < CellNumX; x++) {
        for (unsigned int y = 0; y < CellNumY; y++) {
            uint8_t s = 1u;
            if (x == 0U || x == CellNumX - 1U || y == 0U || y == CellNumY - 1U) {
                s = 0u;
            }
            solidMask[INDEX(x, y)] = s;
        }
    }
}

/* ==================== 初始化 ==================== */

void Fluid_Init(void)
{
    memset(particleVel, 0, sizeof(particleVel));
    memset(uVel, 0, sizeof(uVel));
    memset(vVel, 0, sizeof(vVel));
    memset(uPrev, 0, sizeof(uPrev));
    memset(vPrev, 0, sizeof(vPrev));
    memset(particleDensity, 0, sizeof(particleDensity));

    particleRestDensity = _IQ(0.0f);
    s_frameCount = 0U;

    setup_solid_mask();

    const _iq h  = Spacing;
    const _iq r  = ParticleRadius;
    const _iq dx = _IQmpy(_IQ(2.0f), r);
    const _iq dy = _IQmpy(_IQ(0.86602540378f), dx);

    invertSpacing = _IQdiv(_IQ(1.0f), Spacing);
    maxX = _IQmpy(_IQI(CellNumX - 1U), Spacing) - ParticleRadius;
    maxY = _IQmpy(_IQI(CellNumY - 1U), Spacing) - ParticleRadius;

    /* 六角紧密堆积式摆放粒子 */
    unsigned int p_num = 0;
    for (unsigned int i = 0; i < CellNumX && p_num < NumberOfParticles; i++) {
        for (unsigned int j = 0; j < CellNumY && p_num < NumberOfParticles; j++) {
            _iq px = h + r + _IQmpy(dx, _IQI(i)) + ((j % 2U == 0U) ? _IQ(0.0f) : r);
            _iq py = h + r + _IQmpy(dy, _IQI(j));

            if (px > _IQmpy(_IQI(CellNumX - 1U), h) - r ||
                py > _IQmpy(_IQI(CellNumY - 1U), h) - r) {
                continue;
            }

            particlePos[XID(p_num)] = px;
            particlePos[YID(p_num)] = py;
            p_num++;
        }
    }

    for (; p_num < NumberOfParticles; p_num++) {
        particlePos[XID(p_num)] = h + r;
        particlePos[YID(p_num)] = h + r;
    }
}

/* ==================== 1. 粒子受力积分 + 墙壁碰撞 ==================== */

void FLUID_FN ParticleIntegrate(_iq xAcceleration, _iq yAcceleration)
{
    for (unsigned int i = 0; i < NumberOfParticles; i++) {
        particleVel[XID(i)] += _IQmpy(xAcceleration, s_dt);
        particleVel[YID(i)] += _IQmpy(yAcceleration, s_dt);
        particlePos[XID(i)] += _IQmpy(particleVel[XID(i)], s_dt);
        particlePos[YID(i)] += _IQmpy(particleVel[YID(i)], s_dt);

        _iq x = particlePos[XID(i)];
        _iq y = particlePos[YID(i)];

        if (x < minX) {
            x = minX;
            particleVel[XID(i)] = _IQmpy(particleVel[XID(i)], BOUNCYNESS);
        }
        if (x > maxX) {
            x = maxX;
            particleVel[XID(i)] = _IQmpy(particleVel[XID(i)], BOUNCYNESS);
        }
        if (y < minY) {
            y = minY;
            particleVel[YID(i)] = _IQmpy(particleVel[YID(i)], BOUNCYNESS);
        }
        if (y > maxY) {
            y = maxY;
            particleVel[YID(i)] = _IQmpy(particleVel[YID(i)], BOUNCYNESS);
        }

        particlePos[XID(i)] = x;
        particlePos[YID(i)] = y;
    }
}

/* ==================== 2. 粒子互斥（空间哈希 + 邻居推挤） ==================== */

void FLUID_FN PushParticlesApart(unsigned int nIters)
{
    memset(cellParticleCountPrefix, 0, sizeof(cellParticleCountPrefix));
    memset(particlePosId, 0, sizeof(particlePosId));

    /* 按格子统计粒子数 */
    for (unsigned int i = 0; i < NumberOfParticles; i++) {
        unsigned int xi = (unsigned int)clamp_index(
            _IQint(_IQmpy(particlePos[XID(i)], invertSpacing)), 0, (int)CellNumX - 1);
        unsigned int yi = (unsigned int)clamp_index(
            _IQint(_IQmpy(particlePos[YID(i)], invertSpacing)), 0, (int)CellNumY - 1);
        cellParticleCountPrefix[INDEX(xi, yi)]++;
    }

    /* 前缀和 */
    unsigned int prefix = 0;
    for (unsigned int i = 0; i < CellCount; i++) {
        prefix += cellParticleCountPrefix[i];
        cellParticleCountPrefix[i] = (uint16_t)prefix;
    }
    cellParticleCountPrefix[CellCount] = (uint16_t)prefix;

    /* 按格子重新排列粒子索引 */
    for (unsigned int i = 0; i < NumberOfParticles; i++) {
        unsigned int xi = (unsigned int)clamp_index(
            _IQint(_IQmpy(particlePos[XID(i)], invertSpacing)), 0, (int)CellNumX - 1);
        unsigned int yi = (unsigned int)clamp_index(
            _IQint(_IQmpy(particlePos[YID(i)], invertSpacing)), 0, (int)CellNumY - 1);
        unsigned int cellNr = INDEX(xi, yi);
        unsigned int gridIndex = --cellParticleCountPrefix[cellNr];
        particlePosId[gridIndex] = (uint8_t)i;
    }

    const _iq minDist  = _IQmpy(_IQ(2.0f), ParticleRadius);
    const _iq minDist2 = _IQmpy(minDist, minDist);

    for (unsigned int iter = 0; iter < nIters; iter++) {
        for (unsigned int i = 0; i < NumberOfParticles; i++) {
            _iq px = particlePos[XID(i)];
            _iq py = particlePos[YID(i)];

            int pxi = clamp_index(_IQint(_IQmpy(px, invertSpacing)), 0, (int)CellNumX - 1);
            int pyi = clamp_index(_IQint(_IQmpy(py, invertSpacing)), 0, (int)CellNumY - 1);
            int x0 = (pxi > 0) ? pxi - 1 : 0;
            int y0 = (pyi > 0) ? pyi - 1 : 0;
            int x1 = (pxi + 1 < (int)CellNumX) ? pxi + 1 : (int)CellNumX - 1;
            int y1 = (pyi + 1 < (int)CellNumY) ? pyi + 1 : (int)CellNumY - 1;

            for (int xi = x0; xi <= x1; xi++) {
                for (int yi = y0; yi <= y1; yi++) {
                    unsigned int cellNr   = INDEX((unsigned int)xi, (unsigned int)yi);
                    unsigned int firstIdx = cellParticleCountPrefix[cellNr];
                    unsigned int lastIdx  = cellParticleCountPrefix[cellNr + 1U];

                    for (unsigned int j = firstIdx; j < lastIdx; j++) {
                        unsigned int id = particlePosId[j];
                        if (id == i) {
                            continue;
                        }

                        _iq qx = particlePos[XID(id)];
                        _iq qy = particlePos[YID(id)];
                        _iq ddx = qx - px;
                        _iq ddy = qy - py;
                        _iq d2 = _IQmpy(ddx, ddx) + _IQmpy(ddy, ddy);

                        if (d2 > minDist2 || d2 == _IQ(0.0f)) {
                            continue;
                        }

                        _iq d = _IQsqrt(d2);
                        _iq s = _IQdiv(_IQmpy(_IQ(0.5f), (minDist - d)), d);
                        ddx = _IQmpy(ddx, s);
                        ddy = _IQmpy(ddy, s);

                        particlePos[XID(i)]  -= ddx;
                        particlePos[YID(i)]  -= ddy;
                        particlePos[XID(id)] += ddx;
                        particlePos[YID(id)] += ddy;
                    }
                }
            }
        }
    }

    /* 推挤完再夹一次墙 */
    for (unsigned int i = 0; i < NumberOfParticles; i++) {
        _iq x = particlePos[XID(i)];
        _iq y = particlePos[YID(i)];

        if (x < minX) {
            x = minX;
            particleVel[XID(i)] = _IQmpy(particleVel[XID(i)], BOUNCYNESS);
        }
        if (x > maxX) {
            x = maxX;
            particleVel[XID(i)] = _IQmpy(particleVel[XID(i)], BOUNCYNESS);
        }
        if (y < minY) {
            y = minY;
            particleVel[YID(i)] = _IQmpy(particleVel[YID(i)], BOUNCYNESS);
        }
        if (y > maxY) {
            y = maxY;
            particleVel[YID(i)] = _IQmpy(particleVel[YID(i)], BOUNCYNESS);
        }

        particlePos[XID(i)] = x;
        particlePos[YID(i)] = y;
    }
}

/* ==================== 3. 密度统计（双线性权重累加到格子） ==================== */

void FLUID_FN density_update(void)
{
    memset(particleDensity, 0, sizeof(particleDensity));

    const _iq h  = Spacing;
    const _iq h1 = invertSpacing;
    const _iq h2 = _IQmpy(_IQ(0.5f), h);

    for (unsigned int i = 0; i < NumberOfParticles; i++) {
        _iq x = clampf_local(particlePos[XID(i)], h, _IQmpy(_IQI(CellNumX - 1U), h));
        _iq y = clampf_local(particlePos[YID(i)], h, _IQmpy(_IQI(CellNumY - 1U), h));

        int x0 = _IQint(_IQmpy((x - h2), h1));
        int y0 = _IQint(_IQmpy((y - h2), h1));
        x0 = clamp_index(x0, 0, (int)CellNumX - 2);
        y0 = clamp_index(y0, 0, (int)CellNumY - 2);
        int x1 = x0 + 1;
        int y1 = y0 + 1;

        _iq tx = _IQmpy(((x - h2) - _IQmpy(_IQI(x0), h)), h1);
        _iq ty = _IQmpy(((y - h2) - _IQmpy(_IQI(y0), h)), h1);
        _iq sx = _IQ(1.0f) - tx;
        _iq sy = _IQ(1.0f) - ty;

        particleDensity[INDEX(x0, y0)] += _IQmpy(sx, sy);
        particleDensity[INDEX(x1, y0)] += _IQmpy(tx, sy);
        particleDensity[INDEX(x1, y1)] += _IQmpy(tx, ty);
        particleDensity[INDEX(x0, y1)] += _IQmpy(sx, ty);
    }

    /* 静止密度只在第一帧标定一次。
     *
     * ⚠ 这里的累加**必须用 64 位**：总密度 ≈ N × 2^24，而 INT32_MAX 只够撑到
     *   N ≈ 128.7。之前用 _iq 累加，N=130 只是**勉强压在悬崖边上**
     *   （实测 rest_dens = +36559675 = +2.179，还是正的，所以
     *     compute_grid_forces() 里的刚度项其实是开着的）；
     *   一加到 N=200 立刻翻成负数 → `if (particleRestDensity > 0)` 恒不成立
     *   → 刚度项被关掉 → 流体失去抗压能力，被压成一小坨：
     *        dens_max 2789 → 16697，dens_lit 71 → 40（实测）
     *
     *   sum 与 numFluid 都是"粒子数当量"，2^24 直接约掉，所以整数除的结果
     *   仍是 Q24 密度 —— 与原来的 _IQdiv(sum, _IQI(numFluid)) **完全等价**，
     *   只是不会溢出（在 N=130 下行为逐位不变，是个干净的对照）。 */
    if (particleRestDensity == _IQ(0.0f)) {
        int64_t sum = 0;
        unsigned int numFluid = 0;

        for (unsigned int i = 0; i < CellCount; i++) {
            if (cellType[i] == FLUID_CELL) {
                sum += particleDensity[i];
                numFluid++;
            }
        }

        if (numFluid > 0U) {
            particleRestDensity = (_iq)(sum / (int64_t)numFluid);
        }
    }
}

/* ==================== 4. 粒子速度 → 网格速度 ==================== */

void FLUID_FN particles_to_grid(void)
{
    memset(uVel, 0, sizeof(uVel));
    memset(vVel, 0, sizeof(vVel));
    memset(uWeight, 0, sizeof(uWeight));
    memset(vWeight, 0, sizeof(vWeight));

    /* 先按固体掩码标记类型 */
    for (unsigned int i = 0; i < CellCount; i++) {
        cellType[i] = (solidMask[i] == 0u) ? SOLID_CELL : AIR_CELL;
    }

    /* 有粒子的格子标成流体 */
    for (unsigned int i = 0; i < NumberOfParticles; i++) {
        int xi = clamp_index(_IQint(_IQmpy(particlePos[XID(i)], invertSpacing)), 0, (int)CellNumX - 1);
        int yi = clamp_index(_IQint(_IQmpy(particlePos[YID(i)], invertSpacing)), 0, (int)CellNumY - 1);
        cellType[INDEX((unsigned int)xi, (unsigned int)yi)] = FLUID_CELL;
    }

    const _iq h  = Spacing;
    const _iq h1 = invertSpacing;
    const _iq h2 = _IQmpy(_IQ(0.5f), h);

    for (int component = 0; component < 2; component++) {
        _iq dx = (component == 0) ? _IQ(0.0f) : h2;
        _iq dy = (component == 0) ? h2 : _IQ(0.0f);
        _iq *f = (component == 0) ? uVel : vVel;
        _iq *w = (component == 0) ? uWeight : vWeight;

        for (unsigned int i = 0; i < NumberOfParticles; i++) {
            _iq x = clampf_local(particlePos[XID(i)], h, _IQmpy(_IQI(CellNumX - 1U), h));
            _iq y = clampf_local(particlePos[YID(i)], h, _IQmpy(_IQI(CellNumY - 1U), h));

            int x0 = _IQint(_IQmpy((x - dx), h1));
            int y0 = _IQint(_IQmpy((y - dy), h1));
            x0 = clamp_index(x0, 0, (int)CellNumX - 2);
            y0 = clamp_index(y0, 0, (int)CellNumY - 2);
            int x1 = x0 + 1;
            int y1 = y0 + 1;

            _iq tx = _IQmpy(((x - dx) - _IQmpy(_IQI(x0), h)), h1);
            _iq ty = _IQmpy(((y - dy) - _IQmpy(_IQI(y0), h)), h1);
            _iq sx = _IQ(1.0f) - tx;
            _iq sy = _IQ(1.0f) - ty;

            _iq w0 = _IQmpy(sx, sy);
            _iq w1 = _IQmpy(tx, sy);
            _iq w2 = _IQmpy(tx, ty);
            _iq w3 = _IQmpy(sx, ty);

            _iq pv = particleVel[2U * i + (unsigned int)component];

            unsigned int nr0 = INDEX((unsigned int)x0, (unsigned int)y0);
            unsigned int nr1 = INDEX((unsigned int)x1, (unsigned int)y0);
            unsigned int nr2 = INDEX((unsigned int)x1, (unsigned int)y1);
            unsigned int nr3 = INDEX((unsigned int)x0, (unsigned int)y1);

            f[nr0] += _IQmpy(pv, w0);  w[nr0] += w0;
            f[nr1] += _IQmpy(pv, w1);  w[nr1] += w1;
            f[nr2] += _IQmpy(pv, w2);  w[nr2] += w2;
            f[nr3] += _IQmpy(pv, w3);  w[nr3] += w3;
        }

        /* 归一化 */
        for (unsigned int i = 0; i < CellCount; i++) {
            if (w[i] > _IQ(0.0f)) {
                f[i] = _IQdiv(f[i], w[i]);
            }
        }

        /* 固体格子以及其下游格子，沿用上一帧的速度（不可穿透边界） */
        for (unsigned int x = 0; x < CellNumX; x++) {
            for (unsigned int y = 0; y < CellNumY; y++) {
                unsigned int idx = INDEX(x, y);
                unsigned int solid = (cellType[idx] == SOLID_CELL) ? 1U : 0U;

                if (component == 0) {
                    unsigned int leftSolid =
                        (x > 0U && cellType[INDEX(x - 1U, y)] == SOLID_CELL) ? 1U : 0U;
                    if (solid || leftSolid) {
                        uVel[idx] = uPrev[idx];
                    }
                } else {
                    unsigned int bottomSolid =
                        (y > 0U && cellType[INDEX(x, y - 1U)] == SOLID_CELL) ? 1U : 0U;
                    if (solid || bottomSolid) {
                        vVel[idx] = vPrev[idx];
                    }
                }
            }
        }
    }
}

/* ==================== 5. 网格压力求解（把这步散度压掉） ==================== */

void FLUID_FN compute_grid_forces(unsigned int nIters)
{
    memcpy(uPrev, uVel, sizeof(uVel));
    memcpy(vPrev, vVel, sizeof(vVel));

    for (unsigned int iter = 0; iter < nIters; iter++) {
        for (unsigned int x = 1; x < CellNumX - 1U; x++) {
            for (unsigned int y = 1; y < CellNumY - 1U; y++) {
                unsigned int center = INDEX(x, y);
                if (cellType[center] != FLUID_CELL) {
                    continue;
                }

                unsigned int left   = INDEX(x - 1U, y);
                unsigned int right  = INDEX(x + 1U, y);
                unsigned int bottom = INDEX(x, y - 1U);
                unsigned int top    = INDEX(x, y + 1U);

                uint8_t sx0 = solidMask[left];
                uint8_t sx1 = solidMask[right];
                uint8_t sy0 = solidMask[bottom];
                uint8_t sy1 = solidMask[top];

                /*
                 * 【除法降级】solidMask 非 0 即 1，于是 -_IQdiv(div, s) = -(div / k)：
                 * Q24 的 2^24 被约掉了。这里用 **32 位整数除法**代替 64 位软件除法，
                 * 结果完全等价（截断方向也一致），但快很多。
                 * 这是压力求解里最热的一行：每步要执行「迭代次数 × 流体格子数」次。
                 */
                int k = (int) sx0 + (int) sx1 + (int) sy0 + (int) sy1;

                if (k == 0) {
                    continue;
                }

                _iq div = uVel[right] - uVel[center] + vVel[top] - vVel[center];

                if (particleRestDensity > _IQ(0.0f)) {
                    _iq compression = particleDensity[center] - particleRestDensity;
                    if (compression > _IQ(0.0f)) {
                        compression = _IQmpy(compression, stiffnessCoefficient);
                        div -= compression;
                    }
                }

                _iq p = -(div / k);          /* 原来的 -_IQdiv(div, s) */
                p = _IQmpy(p, overRelaxiation);

                /* 【乘法降级】_IQmpy(sx, p) 里 sx 只有 0 / 1 两种取值：
                 * 乘 1 就是原值（Q24 下 (2^24 * p) >> 24 == p，无舍入），
                 * 乘 0 就是 0。所以直接用条件判断，省掉 4 次 Q24 乘法 ——
                 * 这一步同样是「迭代次数 × 流体格子数」量级。 */
                if (sx0 != 0u) { uVel[center] -= p; }
                if (sx1 != 0u) { uVel[right]  += p; }
                if (sy0 != 0u) { vVel[center] -= p; }
                if (sy1 != 0u) { vVel[top]    += p; }
            }
        }
    }
}

/* ==================== 6. 网格速度 → 粒子速度（PIC/FLIP 混合） ==================== */

void FLUID_FN grid_to_particles(void)
{
    const _iq flipRatio = _IQ(0.9f);
    const _iq h  = Spacing;
    const _iq h1 = invertSpacing;
    const _iq h2 = _IQmpy(_IQ(0.5f), h);

    for (int component = 0; component < 2; component++) {
        _iq dx = (component == 0) ? _IQ(0.0f) : h2;
        _iq dy = (component == 0) ? h2 : _IQ(0.0f);
        _iq *f    = (component == 0) ? uVel  : vVel;
        _iq *prevF = (component == 0) ? uPrev : vPrev;
        int offset = (component == 0) ? (int)CellNumY : 1;

        for (unsigned int i = 0; i < NumberOfParticles; i++) {
            _iq x = clampf_local(particlePos[XID(i)], h, _IQmpy(_IQI(CellNumX - 1U), h));
            _iq y = clampf_local(particlePos[YID(i)], h, _IQmpy(_IQI(CellNumY - 1U), h));

            int x0 = clamp_index(_IQint(_IQmpy((x - dx), h1)), 0, (int)CellNumX - 2);
            int y0 = clamp_index(_IQint(_IQmpy((y - dy), h1)), 0, (int)CellNumY - 2);
            int x1 = x0 + 1;
            int y1 = y0 + 1;

            _iq tx = _IQmpy(((x - dx) - _IQmpy(_IQI(x0), h)), h1);
            _iq ty = _IQmpy(((y - dy) - _IQmpy(_IQI(y0), h)), h1);
            _iq sx = _IQ(1.0f) - tx;
            _iq sy = _IQ(1.0f) - ty;

            _iq d0 = _IQmpy(sx, sy);
            _iq d1 = _IQmpy(tx, sy);
            _iq d2 = _IQmpy(tx, ty);
            _iq d3 = _IQmpy(sx, ty);

            unsigned int nr0 = INDEX((unsigned int)x0, (unsigned int)y0);
            unsigned int nr1 = INDEX((unsigned int)x1, (unsigned int)y0);
            unsigned int nr2 = INDEX((unsigned int)x1, (unsigned int)y1);
            unsigned int nr3 = INDEX((unsigned int)x0, (unsigned int)y1);

            /* 只有流体格子、或"有效侧"的相邻格子才参与插值 */
            _iq valid0 = (cellType[nr0] != AIR_CELL ||
                          ((int)nr0 - offset >= 0 && cellType[(unsigned int)((int)nr0 - offset)] != AIR_CELL))
                         ? _IQ(1.0f) : _IQ(0.0f);
            _iq valid1 = (cellType[nr1] != AIR_CELL ||
                          ((int)nr1 - offset >= 0 && cellType[(unsigned int)((int)nr1 - offset)] != AIR_CELL))
                         ? _IQ(1.0f) : _IQ(0.0f);
            _iq valid2 = (cellType[nr2] != AIR_CELL ||
                          ((int)nr2 - offset >= 0 && cellType[(unsigned int)((int)nr2 - offset)] != AIR_CELL))
                         ? _IQ(1.0f) : _IQ(0.0f);
            _iq valid3 = (cellType[nr3] != AIR_CELL ||
                          ((int)nr3 - offset >= 0 && cellType[(unsigned int)((int)nr3 - offset)] != AIR_CELL))
                         ? _IQ(1.0f) : _IQ(0.0f);

            _iq d = _IQmpy(valid0, d0) + _IQmpy(valid1, d1)
                  + _IQmpy(valid2, d2) + _IQmpy(valid3, d3);
            if (d <= _IQ(0.0f)) {
                continue;
            }

            _iq picV = _IQdiv(
                _IQmpy(valid0, _IQmpy(d0, f[nr0])) + _IQmpy(valid1, _IQmpy(d1, f[nr1])) +
                _IQmpy(valid2, _IQmpy(d2, f[nr2])) + _IQmpy(valid3, _IQmpy(d3, f[nr3])), d);

            _iq corr = _IQdiv(
                _IQmpy(valid0, _IQmpy(d0, (f[nr0] - prevF[nr0]))) +
                _IQmpy(valid1, _IQmpy(d1, (f[nr1] - prevF[nr1]))) +
                _IQmpy(valid2, _IQmpy(d2, (f[nr2] - prevF[nr2]))) +
                _IQmpy(valid3, _IQmpy(d3, (f[nr3] - prevF[nr3]))), d);

            _iq oldV  = particleVel[2U * i + (unsigned int)component];
            _iq flipV = oldV + corr;

            particleVel[2U * i + (unsigned int)component] =
                _IQmpy((_IQ(1.0f) - flipRatio), picV) + _IQmpy(flipRatio, flipV);
        }
    }
}

/* ==================== 对外接口 ==================== */

/* 一个完整时间步，顺序与原版 main() 里的调用序列一致 */
void Fluid_Step(_iq xAcceleration, _iq yAcceleration)
{
    ParticleIntegrate(xAcceleration, yAcceleration);
    PushParticlesApart(PUSH_ITER);
    particles_to_grid();
    density_update();
    compute_grid_forces(s_gridIter);
    grid_to_particles();

    s_frameCount++;
}

void Fluid_SetGridIter(uint32_t nIters)
{
    if (nIters > 32U) {
        nIters = 32U;
    }
    s_gridIter = nIters;
}

void Fluid_SetDt(_iq dt)
{
    /* 简单限幅，避免传入 0 或负值把积分搞坏 */
    if (dt < _IQ(0.001f)) {
        dt = _IQ(0.001f);
    }
    if (dt > _IQ(0.100f)) {
        dt = _IQ(0.100f);
    }
    s_dt = dt;
}

const uint8_t *Fluid_GetCellType(void)
{
    return cellType;
}

/* 取内部 particleDensity[] 数组首地址，长度 CellCount（显示层只读使用）。
 *
 * 用途：显示层用它做"密度阈值渲染"，替代原来的点采样 cellType[]。
 *
 * 为什么密度场不会闪：
 *   density_update() 里每个粒子按**双线性权重**贡献给最近的 4 个格子
 *   （sx/sy/tx/ty 连续变化，四者之和恒为 1）。粒子在格子边界微抖时，
 *   相邻两格的密度是"此消彼长"的连续量，两格都保持 > 0；
 *   而 cellType[] 是点采样——粒子中心跨过边界，格子状态**瞬间翻转**。
 *   所以用密度判断"有没有液体"比用 cellType 平滑得多。
 *
 * ⚠ 有效时机：Fluid_Step() 返回后读到的就是本步的密度。
 *   （density_update() 在 grid_to_particles() 之前跑，而后者只改粒子速度，
 *     不动 particleDensity，所以整步结束后数据仍是最新的。） */
const _iq *Fluid_GetParticleDensity(void)
{
    return particleDensity;
}

/* 静止密度（第一帧标定的"典型液体密度"）。
 * 显示层可以用它当阈值的参考基准，这样换网格尺寸/粒子数后阈值能自适应。
 * 返回 0 表示还没标定（第一帧之前）。 */
_iq Fluid_GetRestDensity(void)
{
    return particleRestDensity;
}

uint32_t Fluid_GetFrameCount(void)
{
    return s_frameCount;
}
