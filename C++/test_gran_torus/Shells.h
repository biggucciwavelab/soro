#include <cmath>
#include <list>
#include <random>
#include <utility>
#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector.h"


namespace chrono {
    namespace chrono::granular {

        /// Poisson Disk sampler for sampling a 3D box in layers.
        /// The computational efficiency of PD sampling degrades as points are added, especially for large volumes.
        /// This class provides an alternative sampling method where PD sampling is done in 2D layers, separated by a specified
        /// distance (padding_factor * diam). This significantly improves computational efficiency of the sampling but at the
        /// cost of discarding the PD uniform distribution properties in the direction orthogonal to the layers.
        template <typename T>
        std::vector<ChVector<T>> Torus_Shell(ChVector<T> center,       ///< Center of axis-aligned box to fill
            ChVector<T> hdims,        ///< Half-dimensions along the x, y, and z axes
            T diam,                   ///< Particle diameter
            T padding_factor = 1.02,  ///< Multiplier on particle diameter for spacing
            bool verbose = false      ///< Output progress during generation
        ) 
        {
            T fill_bottom = center.z() - hdims.z();
            T fill_top = center.z() + hdims.z();

            // set center to bottom
            center.z() = fill_bottom;
            // 2D layer
            hdims.z() = 0;

            chrono::utils::PDSampler<T> sampler(diam * padding_factor);
            std::vector<ChVector<T>> points_full;
            while (center.z() < fill_top) {
                if (verbose) {
                    std::cout << "Create layer at " << center.z() << std::endl;
                }
                auto points = sampler.SampleBox(center, hdims);
                points_full.insert(points_full.end(), points.begin(), points.end());
                center.z() += diam * padding_factor;
            }

            for (double v = 0; v < CH_C_2PI; v = v + 2 * arctan((0.5 * diam) / a)) {
                for (double u = 0; u < CH_C_2PI; u = u + 2 * arctan((0.5 * diam) / (c + a * np.cos(v)))) {
                    // Point on torus
                    double x = cos(u) * (c + a * cos(v));
                    double y = a * sin(v) + init_height;
                    double z = sin(u) * (c + a * cos(v));
                    ChVector<double> location(x, y, z);
                }
            }

            return points_full;
        }
    }
}