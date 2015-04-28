#include "TriMesh.h"
#include <roboptim/capsule/util.hh>
#include <roboptim/capsule/fitter.hh>

int main(int argc, char** argv)
{

    trimesh::TriMesh *mesh;
    {
        using namespace trimesh;
        mesh = TriMesh::read(argv[1]);
        TriMesh::set_verbose(0);
    }

    int vertices_n;
    if(mesh != NULL)
    {
        vertices_n = mesh->vertices.size();
        {
            using namespace roboptim::capsule;

            // Build a cubic polyhedron centered in (0,0,0).
            polyhedron_t polyhedron;
            value_type halfLength = 0.5;

            for (int i = 0; i < vertices_n; i++) {
                polyhedron.push_back (point_t (mesh->vertices[i][0],
                                               mesh->vertices[i][1],
                                               mesh->vertices[i][2]));
            }

            polyhedrons_t polyhedrons;
            polyhedrons.push_back (polyhedron);

            // Define initial capsule parameters. The segment must be inside the
            // polyhedron, and the capsule must contain the polyhedron.
            //
            //
            // To do so, compute initial guess by finding a bounding capsule
            // (not the minimum one).
            //
            // If needed, the convex hull of the polyhedron can be first computed
            // to reduce the number of constraints and accelerate the optimization
            // phase.
            polyhedrons_t convexPolyhedrons;
            computeConvexPolyhedron (polyhedrons, convexPolyhedrons);

            // Create fitter. It is used to find the best fitting capsule on the
            // polyhedron vector.
            Fitter fitter_cube (convexPolyhedrons);

            point_t endPoint1 = point_t (0., 0., 0.);
            point_t endPoint2 = point_t (0., 0., 0.);
            value_type radius = 0.;
            computeBoundingCapsulePolyhedron (convexPolyhedrons,
                              endPoint1, endPoint2, radius);

            argument_t initParam (7);
            convertCapsuleToSolverParam (initParam, endPoint1, endPoint2, radius);

            // Compute best fitting capsule.
            fitter_cube.computeBestFitCapsule (initParam);
            argument_t solutionParam = fitter_cube.solutionParam ();
            std::cout << fitter_cube << std::endl;
        }
    }

}
