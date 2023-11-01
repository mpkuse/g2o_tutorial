// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>

#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/sampler.h>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/auto_differentiation.h>

using namespace std;
using namespace g2o;

// we use the 2D and 3D SLAM types here
// G2O_USE_TYPE_GROUP(slam2d);
// G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(dense);

/**
 * \brief a circle located at x,y with radius r
 */
class VertexCircle : public g2o::BaseVertex<3 /* minimal dimension*/, Eigen::Vector3d /* data type of the vertex */>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCircle() {}

    bool read(std::istream & /*is*/) override { return false; }
    bool write(std::ostream & /*os*/) const override { return false; }

    void setToOriginImpl() override
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    }

    void oplusImpl(const double *update) override
    {
        Eigen::Vector3d::ConstMapType v(update);
        _estimate += v;
    }
};

/**
 * \brief measurement for a point on the circle
 *
 * Here the measurement is the point which is on the circle.
 * The error function computes the distance of the point to
 * the center minus the radius of the circle.
 */
class EdgePointOnCircle : public g2o::BaseUnaryEdge<1 /*errorvec dim*/, Eigen::Vector2d /*measurement*/, VertexCircle /*associated vertex type*/>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePointOnCircle() {}
    bool read(std::istream & /*is*/) override { return false; }
    bool write(std::ostream & /*os*/) const override { return false; }

    template <typename T>
    bool operator()(const T *circle, T *error) const
    {
        typename g2o::VectorN<2, T>::ConstMapType center(circle);
        const T &radius = circle[2];

        error[0] = (measurement().cast<T>() - center).norm() - radius;
        return true;
    }

    G2O_MAKE_AUTO_AD_FUNCTIONS // use autodiff
};

int main(int argc, char **argv)
{
    // Command line parsing
    int maxIterations = 5;
    CommandArgs arg;
    arg.param("i", maxIterations, 10,
              "perform n iterations, if negative consider the gain");
    arg.parseArgs(argc, argv);

    constexpr uint32_t numPoints = 10;
    Eigen::Vector2d center(4.0, 2.0);
    double radius = 2.0;
    Eigen::Vector2d *points = new Eigen::Vector2d[numPoints];
    std::cout << "Center: " << center.transpose() << std::endl;
    g2o::Sampler::seedRand();
    for (int i = 0; i < numPoints; ++i)
    {
        double r = g2o::Sampler::gaussRand(radius, 0.05);
        double angle = g2o::Sampler::uniformRand(0.0, 2.0 * M_PI);
        points[i].x() = center.x() + r * cos(angle);
        points[i].y() = center.y() + r * sin(angle);

        std::cout << "#" << i << ": " << points[i].transpose() << std::endl;
    }

    // create the optimizer to load the data and carry out the optimization
    SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    // allocate the solver
    g2o::OptimizationAlgorithmProperty solverProperty;
    optimizer.setAlgorithm(
        g2o::OptimizationAlgorithmFactory::instance()->construct("lm_dense",
                                                                 solverProperty));

    // 1. add the circle vertex
    VertexCircle *circle = new VertexCircle();
    circle->setId(0);
    circle->setEstimate(
        Eigen::Vector3d(3.0, 3.0, 3.0)); // some initial value for the circle
    optimizer.addVertex(circle);

    // 2. add edge for every points we measured
    for (int i = 0; i < numPoints; ++i)
    {
        EdgePointOnCircle *e = new EdgePointOnCircle;
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        e->setVertex(0, circle);
        e->setMeasurement(points[i]);
        optimizer.addEdge(e);
    }

    // 3. perform the optimization
    optimizer.initializeOptimization();
    optimizer.setVerbose(false);
    optimizer.optimize(5);

    std::cout << "center of the circle " << circle->estimate().head<2>().transpose()
              << endl;
    std::cout << "radius of the cirlce " << circle->estimate()(2) << endl;

    delete[] points;

    return 0;
}