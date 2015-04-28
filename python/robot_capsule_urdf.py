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
parser.add_argument('-o', '--output', type=argparse.FileType('w'),
                    default=None, help='Dump file to XML')
args = parser.parse_args()
# Extract robot name and directory

if args.urdf_file is None:
    print "Error! no urdf_file provided"
else:
    robot = URDF.from_xml_string(args.urdf_file.read())

links = robot.links
rospack = rospkg.RosPack()

for link in links:
    if link.collision:
        if type(link.collision.geometry) is Mesh:
            collision_file = link.collision.geometry.filename
            
            is_package = collision_file.find('package://')
            if is_package != -1:
                package_len = collision_file.find('/',10)
                package_name = collision_file[10:package_len]
                ros_path = rospack.get_path(package_name)
                collision_file = ros_path+'/'+collision_file[package_len+1:]
            
            p = subprocess.Popen(["robot_capsule_generator",collision_file],stdout=subprocess.PIPE)
            stdout,stderr = p.communicate()
            params = stdout.rstrip()
            
            cylinder_para = params.split('\n')
            cylinder_para = cylinder_para[1:]
            
            ep1 = PyKDL.Vector(float(cylinder_para[0]),
                               float(cylinder_para[1]),
                               float(cylinder_para[2]))
            ep2 = PyKDL.Vector(float(cylinder_para[3]),
                               float(cylinder_para[4]),
                               float(cylinder_para[5]))

            radius = float(cylinder_para[6])
            length = (ep2-ep1).Norm()

            p = .5*(ep2+ep1)
            x = p.x()
            y = p.y()
            z = p.z()

            e1 = PyKDL.Vector(0,
                              0,
                              1)
            r = e1*p
            s_theta = r.Normalize()
            c_theta = PyKDL.dot(e1,p)
                
            theta = math.atan2(s_theta,c_theta)

            (R,P,Y) = PyKDL.Rotation().Rot(r,theta).GetRPY()

            cylinder = Cylinder (float (radius), float (length))
            
            link.collision.geometry = cylinder
            link.collision.origin.position[0] = float (x)
            link.collision.origin.position[1] = float (y)
            link.collision.origin.position[2] = float (z)
            link.collision.origin.rotation[0] = float (R)
            link.collision.origin.rotation[1] = float (P)
            link.collision.origin.rotation[2] = float (Y)

robot.links = links
urdf = robot.to_xml_string ()
print (urdf)
