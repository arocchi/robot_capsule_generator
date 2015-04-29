#!/usr/bin/env python

# Copyright (C) 2011 CNRS, 2015 the Walkman Consortium
#
# Authors: Antonio El Khoury, Alessio Rocchi
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import argparse

from urdf_parser_py.urdf import *
import PyKDL
import math
import rospkg
import subprocess
import os


class CapsuleGenerator(object):
    def __init__(self, link):

        self.p = PyKDL.Vector(0,0,0)
        self.radius = 0
        self.length = 0
        self.rot = PyKDL.Rotation()
        self._link = None
        self.mesh_path = ''
        self.capsule_path = ''

        if link.collision:
            self._link = link
            if type(link.collision.geometry) is Mesh:
                mesh_filename = link.collision.geometry.filename
                is_package = mesh_filename.find('package://')
                if is_package != -1:
                    rospack = rospkg.RosPack()
                    package_len = mesh_filename.find('/',10)
                    package_name = mesh_filename[10:package_len]
                    ros_path = rospack.get_path(package_name)
                    mesh_filename = ros_path+'/'+mesh_filename[package_len+1:]
                self.mesh_path = mesh_filename
                self.capsule_path = os.path.splitext(mesh_filename)[0]+'.capsule'

                mesh_exists = os.path.isfile(self.mesh_path)
                assert mesh_exists

                print("Creating capsule data for link %s\n"%self._link.name)
                self.__load_or_compute()
            else:
                raise Exception('Specified link\'s collision object is not a mesh')

        else:
            raise Exception('Specified link does not have a collision object')

    def __load_or_compute(self):
        capsule_exists = os.path.isfile(self.capsule_path)
        capsule_is_updated = False
        if capsule_exists:
            capsule_is_updated = os.path.getmtime(self.capsule_path) >= os.path.getmtime(self.mesh_path)
        if capsule_exists and capsule_is_updated:
            print("Loading capsule from capsule file %s\n"%self.capsule_path)
            self.load()
        else:
            print("Computing capsule from mesh file %s\n"%self.mesh_path)
            self.compute()
            self.save()

    def compute(self):
        self.__params_to_cylinder(self.__compute_params())

    def load(self):
        capsule_file = file(self.capsule_path,'r')

        if type(capsule_file) is file and not capsule_file.closed:
            capsule_urdf = URDF.from_xml_string(capsule_file.read())
            capsule_link = capsule_urdf.links[0]
            if capsule_link.collision:
                collision = capsule_link.collision
                self.p = PyKDL.Vector(collision.origin.position[0],
                                      collision.origin.position[1],
                                      collision.origin.position[2])

                cylinder = collision.geometry
                self.radius = cylinder.radius
                self.length = cylinder.length
                if type(cylinder) is not Cylinder:
                    raise Exception('error loading capsule from urdf')
                self.rot = PyKDL.Rotation.RPY(collision.origin.rotation[0],
                                              collision.origin.rotation[1],
                                              collision.origin.rotation[2])

            else:
                raise Exception('no collision information found on urdf')

    def save(self):
        robot = Robot(self._link.name)
        link = Link(name=self._link.name)
        collision = Collision(origin=self._link.collision.origin)
        cylinder = Cylinder(self.radius, self.length)

        collision.geometry = cylinder

        mesh_rotation = PyKDL.Rotation().RPY(collision.origin.rotation[0],
                                             collision.origin.rotation[1],
                                             collision.origin.rotation[2],)

        """ we transform the capsule center in parent coordinates """
        offset = mesh_rotation*self.p
        collision.origin.position[0] += offset.x()
        collision.origin.position[1] += offset.y()
        collision.origin.position[2] += offset.z()

        capsule_rotation = mesh_rotation*self.rot
        (R,P,Y) = capsule_rotation.GetRPY()
        collision.origin.rotation[0] = float (R)
        collision.origin.rotation[1] = float (P)
        collision.origin.rotation[2] = float (Y)
        link.collision = collision

        robot.add_link(link)

        capsule_file = file(self.capsule_path,'w')
        capsule_file.write(robot.to_xml_string())
        capsule_file.close()

    def __compute_params(self):
        if self._link.collision.geometry.scale:
            p = subprocess.Popen(["robot_capsule_generator",self.mesh_path], stdout=subprocess.PIPE)
        else:
            scale = self._link.collision.geometry.scale
            p = subprocess.Popen(["robot_capsule_generator",
                                  self.mesh_path,
                                  "--scale",
                                  scale[0],
                                  scale[1],
                                  scale[2]],stdout=subprocess.PIPE)
        stdout,stderr = p.communicate()
        params = stdout.rstrip()

        cylinder_para = params.split('\n')
        cylinder_para = cylinder_para[1:]
        return cylinder_para

    def __params_to_cylinder(self, cylinder_params):
        ep1 = PyKDL.Vector(float(cylinder_params[0]),
                           float(cylinder_params[1]),
                           float(cylinder_params[2]))
        ep2 = PyKDL.Vector(float(cylinder_params[3]),
                           float(cylinder_params[4]),
                           float(cylinder_params[5]))

        self.radius = float(cylinder_params[6])
        self.length = (ep2-ep1).Norm()

        self.p = .5*(ep2+ep1)

        e3 = PyKDL.Vector(0,
                          0,
                          1)
        r = e3*self.p
        s_theta = r.Normalize()
        c_theta = PyKDL.dot(e3,self.p)

        theta = math.atan2(s_theta,c_theta)

        self.rot = PyKDL.Rotation().Rot(r,theta)



if __name__ == '__main__':
    """
    robot_capsule_urdf is a python script that will parse a robot urdf containing
    mesh collision information, and generate an urdf just with collision data where
    links are instead approximated with capsules.
    The capsule is then converted into a cylinder in the urdf, where the position
    of the cylinder will be the mean of the two endpoints plus the original mesh
    offset, and the orientation will be computed by multiplyng the original mesh
    rotation matrix with the rotation obtained from the two endpoints ep2 and ep1
    by computing the cross product between e3 and v (in order to align the local
    z axis of the capsule frame to the actual segment connecting the endpoints)

    e3 = [0,0,1]
    v=.5*(ep2+ep1)
    a=e3 x v

    normalizing it in a_norm, computing

    s_theta = a.norm()
    c_theta= dot(e1,a)

    Finally we will be able to obtain a rotation matrix from an axis angle rotation
    representation where the angle theta and the axis r will be

    theta = atan2(s_theta,c_theta)
    r = a_norm
    """
    parser = argparse.ArgumentParser(usage='robot_capsule_urdf <options> urdf_file capsule_param\nLoad an URDF file')
    parser.add_argument('urdf_file', type=argparse.FileType('r'), nargs='?',
                        default=None, help='Urdf file. Use - for stdin')
    parser.add_argument('-o', '--output', help='Dump file to urdf', action='store_true')
    args = parser.parse_args()
    # Extract robot name and directory

    if args.urdf_file is None:
        print("Error! no urdf_file provided")
    else:
        robot = URDF.from_xml_string(args.urdf_file.read())
    print
    links = robot.links

    for link in links:
        if link.collision:
            if type(link.collision.geometry) is Mesh:
                c = CapsuleGenerator(link)

                cylinder = Cylinder (c.radius, c.length)

                link.collision.geometry = cylinder
                link.collision.origin.position[0] = c.p.x()
                link.collision.origin.position[1] = c.p.y()
                link.collision.origin.position[2] = c.p.z()
                (R,P,Y) = c.rot.GetRPY()
                link.collision.origin.rotation[0] = R
                link.collision.origin.rotation[1] = P
                link.collision.origin.rotation[2] = Y

    robot.links = links
    urdf = robot.to_xml_string()
    if args.output:
        new_urdf_filename = os.path.splitext(args.urdf_file.name)[0]+'_capsules.urdf'
        file(new_urdf_filename,'w').write(urdf)
    else:
        print(urdf)
