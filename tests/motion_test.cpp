#include "motion.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
using namespace navbot;
class Fake final : public Motor
{
public:
    Fake() : Motor({WheelPosition::FrontLeft, 'C', "test", 12000, 10, 500, 4, 32, false}) {}
    uint32_t count{};
    int output{};

private:
    bool hardwareInit() override { return true; }
    bool hardwareWrite(int32_t d) override
    {
        output = d;
        return true;
    }
    void hardwareCoast() override { output = 0; }
    bool hardwareBrake() override { return true; }
    uint32_t hardwareCounter() const override { return count; }
};
struct Rig
{
    Fake a, b, c, d;
    std::array<Motor *, 4> motors{&a, &b, &c, &d};
    Motion motion{motors};
    uint32_t now{};
    Rig()
    {
        for (auto *m : motors)
            assert(m->initialize(0));
        tick();
    }
    void tick(uint32_t elapsed = 10)
    {
        now += elapsed;
        for (auto *m : motors)
            assert(m->updateFeedback(now));
        motion.update(now);
    }
};
int main()
{
    Rig r;
    float v[4] = {6, 6, 6, 6};
    int8_t signs[4] = {1, 1, -1, -1};
    assert(!r.motion.wheels(v, 200, 1, r.now));
    assert(!r.motion.body(.1f, 0, 200, 1, r.now));
    assert(r.motion.configure(.0425f, .228f, signs));
    assert(r.motion.enable(r.now));
    assert(!r.motion.configure(.0425f, .228f, signs));
    float bad[4] = {6, 6, std::numeric_limits<float>::quiet_NaN(), 6};
    assert(!r.motion.wheels(bad, 200, 1, r.now));
    assert(!r.motion.wheels(v, 251, 1, r.now));
    assert(r.motion.body(.1f, .2f, 200, 2, r.now));
    auto s = r.motion.snapshot();
    assert(std::fabs(s.target[0] - (.1f - .0228f) / .0425f) < .001f);
    assert(s.target[0] == s.target[1] && s.target[2] == s.target[3]);
    for (int i = 0; i < 19; ++i)
    {
        r.tick();
        assert(r.a.output >= 0 && r.c.output <= 0);
        assert(std::abs(r.a.output) <= 200);
    }
    r.tick();
    assert(r.motion.snapshot().state == MotionState::Timeout && r.a.output == 0 && r.c.output == 0);
    assert(!r.motion.enable(r.now));
    r.motion.stop();
    assert(r.motion.enable(r.now));
    assert(r.motion.wheels(v, 200, 3, r.now));
    // Sustained saturation must remain bounded. Refresh every 100ms.
    for (int i = 0; i < 200; ++i)
    {
        if (i % 10 == 0)
            assert(r.motion.wheels(v, 200, 4 + i, r.now));
        r.a.count += 20;
        r.b.count += 20;
        r.c.count -= 20;
        r.d.count -= 20;
        r.tick();
        assert(std::abs(r.a.output) <= 200);
    }
    float zero[4]{};
    assert(r.motion.wheels(zero, 200, 300, r.now));
    for (int i = 0; i < 40; ++i)
    {
        if (i % 10 == 0)
            assert(r.motion.wheels(zero, 200, 301 + i, r.now));
        r.tick();
    }
    assert(r.a.output == 0 && r.motion.snapshot().ramp[0] == 0);
    // Timeout before tick cannot be revived by an arriving frame.
    assert(!r.motion.wheels(v, 200, 400, r.now + 250));
    r.motion.stop();
    r.tick();
    assert(r.motion.enable(r.now));
    r.motion.update(r.now + 21);
    assert(r.motion.snapshot().state == MotionState::Fault && r.a.output == 0);
    r.motion.stop();
    assert(!r.motion.enable(r.now));
    // A negative target ramps through zero before opposite PWM.
    Rig q;
    assert(q.motion.enable(q.now));
    assert(q.motion.wheels(v, 200, 1, q.now));
    for (int i = 0; i < 10; ++i)
        q.tick();
    float neg[4] = {-6, -6, -6, -6};
    assert(q.motion.wheels(neg, 200, 2, q.now));
    q.tick();
    assert(q.a.output >= 0);
    // A stationary encoder cannot cause an indefinitely renewed startup boost.
    Rig blocked;
    assert(blocked.motion.enable(blocked.now));
    for (int i = 0; i < 31; ++i)
    {
        if (i % 10 == 0)
            assert(blocked.motion.wheels(v, 200, 100 + i, blocked.now));
        blocked.tick();
    }
    assert(blocked.motion.snapshot().state == MotionState::Fault);
    assert(blocked.motion.snapshot().fault == 4 && blocked.a.output == 0);
    blocked.motion.stop();
    assert(!blocked.motion.enable(blocked.now));
    // A single encoder blip must not end startup or defeat its deadline.
    Rig glitch;
    assert(glitch.motion.enable(glitch.now));
    float low[4] = {1.6f, 1.6f, 1.6f, 1.6f};
    for (int i = 0; i < 31; ++i)
    {
        if (i % 10 == 0)
            assert(glitch.motion.wheels(low, 200, 200 + i, glitch.now));
        if (i == 4)
        {
            glitch.a.count += 40;
            glitch.b.count += 40;
            glitch.c.count += 40;
            glitch.d.count += 40;
        }
        glitch.tick();
    }
    assert(glitch.motion.snapshot().fault == 4 && glitch.a.output == 0);
    // No hidden integral accumulation during a long breakaway interval.
    Rig release;
    assert(release.motion.enable(release.now));
    for (int i = 0; i < 31; ++i)
    {
        if (i % 10 == 0)
            assert(release.motion.wheels(low, 200, 300 + i, release.now));
        if (i >= 28)
        {
            release.a.count += 51;
            release.b.count += 51;
            release.c.count += 51;
            release.d.count += 51;
        }
        release.tick();
    }
    assert(release.motion.snapshot().state == MotionState::Enabled);
    assert(release.a.output >= 40 && release.a.output <= 75);
    // Recovery is bounded across refreshed commands, never unlimited hammering.
    Rig recovery;
    assert(recovery.motion.enable(recovery.now));
    uint32_t recovery_seq = 500;
    auto recovery_tick = [&](bool rotating)
    {
        assert(recovery.motion.wheels(low, 200, recovery_seq++, recovery.now));
        if (rotating)
        {
            recovery.a.count += 51;
            recovery.b.count += 51;
            recovery.c.count += 51;
            recovery.d.count += 51;
        }
        recovery.tick();
    };
    for (int i = 0; i < 20; ++i)
        recovery_tick(true);
    for (int cycle = 0; cycle < 3; ++cycle)
    {
        for (int i = 0; i < 3; ++i)
            recovery_tick(false);
        for (int i = 0; i < 5; ++i)
            recovery_tick(true);
        assert(recovery.motion.snapshot().recoveries[0] == unsigned(cycle + 1));
    }
    for (int i = 0; i < 3; ++i)
        recovery_tick(false);
    assert(recovery.motion.snapshot().fault == 5 && recovery.a.output == 0);
    // The production 2ms period preserves deadlines and startup slew limits.
    Rig fast;
    assert(fast.motion.enable(fast.now));
    int previous = 0;
    for (int i = 0; i < 151; ++i)
    {
        if (i % 10 == 0)
            assert(fast.motion.wheels(low, 200, 700 + i, fast.now));
        fast.tick(2);
        assert(std::abs(fast.a.output) <= 200);
        assert(std::abs(fast.a.output) - previous <= 2);
        previous = std::abs(fast.a.output);
        if (i < 150)
            assert(fast.motion.snapshot().state == MotionState::Enabled);
    }
    assert(fast.motion.snapshot().fault == 4 && fast.a.output == 0);
    // After startup, a speed drop must receive immediate PI correction rather
    // than the startup slew cap; otherwise asymmetric delay induces stick-slip.
    Rig correction;
    assert(correction.motion.enable(correction.now));
    for (int i = 0; i < 80; ++i)
    {
        if (i % 10 == 0)
            assert(correction.motion.wheels(low, 200, 900 + i, correction.now));
        correction.a.count += 10;
        correction.b.count += 10;
        correction.c.count += 10;
        correction.d.count += 10;
        correction.tick(2);
    }
    int before_drop = correction.a.output;
    correction.tick(2);
    assert(correction.a.output > before_drop + 2);
    assert(correction.motion.snapshot().state == MotionState::Enabled);
    // Invalid configuration does not enable geometry.
    Rig x;
    signs[0] = 0;
    assert(!x.motion.configure(.0425f, .228f, signs));
    assert(!x.motion.snapshot().configured);
    puts("motion: kinematics, limits, PI saturation, ramp/reversal, stale command, timeout, feedback fault passed");
}
