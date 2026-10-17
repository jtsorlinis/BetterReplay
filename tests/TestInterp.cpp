#include <cmath>
#include <vector>

#include "Interp.h"
#include "TestHarness.h"

using namespace br;

TEST(Interp_Wrap) {
    CHECK_NEAR(wrap180(190.0), -170.0, 1e-12);
    CHECK_NEAR(wrap180(-180.0), 180.0, 1e-12);
    CHECK_NEAR(wrap180(540.0), 180.0, 1e-12);
    CHECK_NEAR(wrap360(-10.0), 350.0, 1e-12);
    CHECK_NEAR(wrap360(720.0), 0.0, 1e-12);
}

TEST(Interp_LinearAndStep) {
    CHECK_NEAR(interpolate(Interp::Linear, 10.0, 20.0, 0.25), 12.5, 1e-12);
    CHECK_NEAR(interpolate(Interp::Step, 0.0, 1.0, 0.99), 0.0, 1e-12);
    CHECK_NEAR(interpolate(Interp::Step, 1.0, 0.0, 0.0), 1.0, 1e-12);
}

TEST(Interp_HeadingTakesShortestPathThroughNorth) {
    CHECK_NEAR(lerpAngle360(350.0, 10.0, 0.5), 0.0, 1e-9);
    CHECK_NEAR(lerpAngle360(350.0, 10.0, 0.25), 355.0, 1e-9);
    CHECK_NEAR(lerpAngle360(10.0, 350.0, 0.75), 355.0, 1e-9);
    CHECK_NEAR(lerpAngle360(90.0, 270.0 - 1e-9, 0.5), 180.0, 1e-6);  // not the long way via 0
}

TEST(Interp_BankAndLongitudeWrapAt180) {
    CHECK_NEAR(lerpAngle180(170.0, -170.0, 0.5), 180.0, 1e-9);
    CHECK_NEAR(lerpAngle180(-179.0, 179.0, 0.25), -179.5, 1e-9);
    CHECK_NEAR(lerpAngle180(-10.0, 20.0, 0.5), 5.0, 1e-9);
}

TEST(Interp_NaNFallsBackToOtherSide) {
    CHECK_NEAR(interpolate(Interp::Linear, NAN, 4.0, 0.3), 4.0, 0.0);
    CHECK_NEAR(interpolate(Interp::Linear, 3.0, NAN, 0.3), 3.0, 0.0);
}

static Frame makeFrame(double t, double lat, double hdg, double gear, double light) {
    Frame f;
    for (double& v : f.v) v = 0.0;
    f.t = t;
    f.v[kLat] = lat;
    f.v[kHeading] = hdg;
    f.v[kGearCenter] = gear;
    f.v[kLightLanding] = light;
    return f;
}

TEST(Interp_SampleFramesUsesChannelInterpKinds) {
    std::vector<Frame> frames = {makeFrame(0.0, 10.0, 358.0, 0.0, 0.0), makeFrame(1.0, 11.0, 2.0, 1.0, 1.0),
                                 makeFrame(3.0, 13.0, 6.0, 1.0, 1.0)};
    Frame out;
    size_t hint = 0;
    sampleFrames(frames.data(), frames.size(), 0.5, out, &hint);
    CHECK_NEAR(out.t, 0.5, 1e-12);
    CHECK_NEAR(out.v[kLat], 10.5, 1e-12);
    CHECK_NEAR(out.v[kHeading], 0.0, 1e-9);     // Angle360 through north
    CHECK_NEAR(out.v[kGearCenter], 0.5, 1e-12);  // Linear
    CHECK_NEAR(out.v[kLightLanding], 0.0, 0.0);  // Step holds the earlier value
    CHECK_EQ(hint, 0u);

    sampleFrames(frames.data(), frames.size(), 2.0, out, &hint);  // uneven spacing
    CHECK_NEAR(out.v[kLat], 12.0, 1e-12);
    CHECK_NEAR(out.v[kHeading], 4.0, 1e-9);
    CHECK_EQ(hint, 1u);
}

TEST(Interp_SampleFramesClampsAtEnds) {
    std::vector<Frame> frames = {makeFrame(0.0, 1.0, 0, 0, 0), makeFrame(1.0, 2.0, 0, 0, 0)};
    Frame out;
    sampleFrames(frames.data(), frames.size(), -5.0, out, nullptr);
    CHECK_NEAR(out.v[kLat], 1.0, 0.0);
    sampleFrames(frames.data(), frames.size(), 99.0, out, nullptr);
    CHECK_NEAR(out.v[kLat], 2.0, 0.0);
}

TEST(Interp_SampleFramesBadHintStillCorrect) {
    std::vector<Frame> frames;
    for (int i = 0; i < 100; ++i) frames.push_back(makeFrame(i * 0.1, i, 0, 0, 0));
    Frame out;
    size_t hint = 90;  // far from the target segment
    sampleFrames(frames.data(), frames.size(), 1.05, out, &hint);
    CHECK_NEAR(out.v[kLat], 10.5, 1e-9);
    CHECK_EQ(hint, 10u);
}
