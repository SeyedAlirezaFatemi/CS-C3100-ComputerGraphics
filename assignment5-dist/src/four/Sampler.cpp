#include "Sampler.h"

#include <cassert>

Sampler::Sampler(int nSamples) : m_nSamples(nSamples) {}

Sampler *Sampler::constructSampler(Args::SamplePatternType t, int numsamples) {
    if (t == Args::Pattern_Uniform) {
        return new UniformSampler(numsamples);
    } else if (t == Args::Pattern_Regular) {
        return new RegularSampler(numsamples);
    } else if (t == Args::Pattern_Jittered) {
        return new JitteredSampler(numsamples);
    } else {
        assert(false && "Bad sampler type");
        return nullptr;
    }
}

UniformSampler::UniformSampler(int nSamples) : Sampler(nSamples) {}

Vec2f UniformSampler::getSamplePosition(int i) {
    // make the compiler shut up by referencing the variable
    (void) i;
    // YOUR CODE HERE (R9)
    // Return a uniformly distributed random 2-vector within the unit square [0,1]^2
    return this->m_generator.getVec2f(0, 1);
}

RegularSampler::RegularSampler(int nSamples) : Sampler(nSamples) {
    // test that it's a perfect square
    int dim = (int) sqrtf(float(nSamples));
    assert(nSamples == dim * dim);
    m_dim = dim;
}

Vec2f RegularSampler::getSamplePosition(int n) {
    // YOUR CODE HERE (R9)
    // Return a sample through the center of the Nth subpixel.
    // The starter code only supports one sample per pixel.
    int col = n % this->m_dim;
    int row = static_cast<int>(n / this->m_dim);
    return Vec2f(1.0f / (2 * this->m_dim) + static_cast<float>(col) * 1.0f / (this->m_dim), 1.0f / (2 * this->m_dim) + static_cast<float>(row) * 1.0f / (this->m_dim));
}

JitteredSampler::JitteredSampler(int nSamples) : Sampler(nSamples) {
    // test that it's a perfect square
    int dim = (int) sqrtf(float(nSamples));
    assert(nSamples == dim * dim);
    m_dim = dim;
}

Vec2f JitteredSampler::getSamplePosition(int n) {
    // YOUR CODE HERE (R9)
    // Return a randomly generated sample through Nth subpixel.
    const auto range = 1.0f / (2 * this->m_dim);
    int col = n % this->m_dim;
    int row = static_cast<int>(n / this->m_dim);
    return Vec2f(1.0f / (2 * this->m_dim) + static_cast<float>(col) * 1.0f / (this->m_dim), 1.0f / (2 * this->m_dim) + static_cast<float>(row) * 1.0f / (this->m_dim)) + this->m_generator.getVec2f(-range, range);
}
