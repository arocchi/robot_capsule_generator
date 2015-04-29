/*
 * Copyright (C) 2015 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include "TriMesh.h"
#include <roboptim/capsule/util.hh>
#include <roboptim/capsule/fitter.hh>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

int main(int argc, char** argv)
{

    std::string appName = boost::filesystem::basename(argv[0]);
    std::string mesh_path;
    std::vector<double> scaling(3,1.0);

    // Define and parse the program options
    namespace po = boost::program_options;
    po::positional_options_description p;
    p.add("mesh_path",
           1);
    po::options_description desc("Options");
    desc.add_options()
      ("help", "this help message")
      ("mesh_path",
       po::value<std::string>(&mesh_path)->required(),
       "path of mesh file to load")
      ("scaling",
       po::value< std::vector<double> >(&scaling)->multitoken());

    po::variables_map vm;
    try
    {
      po::store(po::command_line_parser(argc, argv).
                options(desc).positional(p).run(),
                vm); // can throw

      /** --help option
       */
      if ( vm.count("help")  )
      {
        std::cout << "moveit_compute_default_collision, a command line tool from the moveit setup assistant" << std::endl
                  << "USAGE: moveit_compute_default_collision --urdf_path [urdf file path] --srdf_path [srdf file path] "
                  << std::endl << std::endl;
        std::cout << desc << std::endl;

        return 0;
      }

      if ( vm.count("scaling")  )
      {
          scaling = vm["scaling"].as<std::vector<double> >();
          if(scaling.size() != 3)
          {
              std::cout << "Error: scaling should be a vector of 3 elements" << std::endl;
              return 0;
          }

          for(unsigned int i = 0; i < scaling.size(); ++i)
          {
              if(scaling[i] < 0.0)
              {
                  std::cout << "Error: scaling[" << i << "] is lower than 0" << std::endl;
                  return 0;
              } else if(scaling[i] == 0.0)
              {
                  std::cout << "Error: scaling[" << i << "] is equal to 0" << std::endl;
                  return 0;
              }
          }
      }

      if (!vm.count("mesh_path"))
      {

          std::cout << "Error: a proper mesh math has not been specified" << std::endl;
          return 0;
      }

      po::notify(vm); // throws on error, so do after help in case
                      // there are any problems
    }
    catch(po::error& e)
    {
      std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
      std::cerr << desc << std::endl;
      return 1;
    }

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
                polyhedron.push_back (point_t (mesh->vertices[i][0]*scaling[0],
                                               mesh->vertices[i][1]*scaling[1],
                                               mesh->vertices[i][2]*scaling[2]));
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
            std::cout << solutionParam << std::endl;
        }
    }

}
