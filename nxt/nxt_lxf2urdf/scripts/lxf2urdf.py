#! /usr/bin/env python

"""
usage: %(progname)s file.lxf file.ldr
"""

import os, sys, getopt
import zipfile
import xml.dom.minidom
import numpy
from numpy.linalg import inv

import transformations as TF
rigid_tree=[]
brick_tree=[]

def parseInts(s):
  if s:
    return [int(x) for x in s.split(',')]
  else:
    return None

def parseFloats(s):
  if s:
    return [float(x) for x in s.split(',')]
  else:
    return None

motor_template = """
    <link name="ref_%(refID)s_link">
      <inertial>
        <mass value="0.010000" />
        <!-- center of mass (com) is defined w.r.t. link local coordinate system -->
        <origin xyz="0.0 0.0 0" />
        <inertia  ixx="0.01" ixy="0.0"  ixz="0.0"  iyy="0.01"  iyz="0.0"  izz="0.01" />
      </inertial>
      <visual>
        <!-- visual origin is defined w.r.t. link local coordinate system -->
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
          <mesh filename="%(mesh)s" scale="%(m_scale)s %(m_scale)s %(m_scale)s"/>
        </geometry>
      </visual>
      <collision>
        <!-- collision origin is defined w.r.t. link local coordinate system -->
        <origin xyz="%(bound_x)s %(bound_y)s %(bound_z)s" rpy="%(bound_roll)s %(bound_pitch)s %(bound_yaw)s" />
        <geometry>
          <box size="%(dim_x)s %(dim_y)s %(dim_z)s"/>
        </geometry>
      </collision>
    </link>

    <link name="ref_%(refID)s_link_hub">
      <inertial>
        <mass value="0.010000" />
        <!-- center of mass (com) is defined w.r.t. link local coordinate system -->
        <origin xyz="0.0 0.0 0" />
        <inertia  ixx="0.01" ixy="0.0"  ixz="0.0"  iyy="0.01"  iyz="0.0"  izz="0.01" />
      </inertial>
      <visual>
        <!-- visual origin is defined w.r.t. link local coordinate system -->
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
          <mesh filename="package://nxt_description/meshes/parts/servo_hubs.dae" scale="%(m_scale)s %(m_scale)s %(m_scale)s"/>
        </geometry>
      </visual>
      <collision>
        <!-- collision origin is defined w.r.t. link local coordinate system -->
        <origin xyz="%(bound_x)s %(bound_y)s %(bound_z)s" rpy="%(bound_roll)s %(bound_pitch)s %(bound_yaw)s" />
        <geometry>
          <box size="%(dim_x)s %(dim_y)s %(dim_z)s"/>
        </geometry>
      </collision>
    </link>

    <!--THIS IS THE MOTOR JOINT RENAME AND FIX BY 180 WHEN NEEDED-->
    <joint name="mot_%(refID)s_joint" type="continuous">
      <parent link="ref_%(refID)s_link"/>
      <child link="ref_%(refID)s_link_hub"/>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <axis xyz="1 0 0" />
    </joint>

"""

link_template = """
    <link name="ref_%(refID)s_link">
      <inertial>
        <mass value="0.010000" />
        <!-- center of mass (com) is defined w.r.t. link local coordinate system -->
        <origin xyz="0.0 0.0 0" />
        <inertia  ixx="0.01" ixy="0.0"  ixz="0.0"  iyy="0.01"  iyz="0.0"  izz="0.01" />
      </inertial>
      <visual>
        <!-- visual origin is defined w.r.t. link local coordinate system -->
        <origin xyz="0 0 0" rpy="%(rot_x)s %(rot_y)s %(rot_z)s" />
        <geometry>
          <mesh filename="%(mesh)s" scale="%(m_scale)s %(m_scale)s %(m_scale)s"/>
        </geometry>
      </visual>
      <collision>
        <!-- collision origin is defined w.r.t. link local coordinate system -->
        <origin xyz="%(bound_x)s %(bound_y)s %(bound_z)s" rpy="%(bound_roll)s %(bound_pitch)s %(bound_yaw)s" />
        <geometry>
          <box size="%(dim_x)s %(dim_y)s %(dim_z)s"/>
        </geometry>
      </collision>
    </link>
"""

joint_template = """
    <joint name="ref_%(refID)s_joint" type="%(joint_type)s">
      <parent link="%(parent_link)s"/>
      <child link="%(child_link)s"/>
      <origin xyz="%(origin_x)s %(origin_y)s %(origin_z)s" rpy="%(origin_roll)s %(origin_pitch)s %(origin_yaw)s" />
      <axis xyz="%(axis_x)s %(axis_y)s %(axis_z)s" />
    </joint>
"""

ultrasonic_link = """
  <!--THIS IS THE ULTRASONIC LINK RENAME ME-->
  <link name="ref_%(refID)s_link">
    <inertial>
      <mass value="0.023900" />
      <!-- center of mass (com) is defined w.r.t. link local coordinate system -->
      <origin xyz="0 0 0" />
      <inertia  ixx="0.01" ixy="0.0"  ixz="0.0"  iyy="0.01"  iyz="0.0"  izz="0.01" />
    </inertial>
    <visual>
      <!-- visual origin is defined w.r.t. link local coordinate system -->
      <origin xyz=" -0.026 0 -0.018" rpy="0 0 1.57079633" />
      <geometry>
        <mesh filename="%(mesh)s" scale="%(m_scale)s %(m_scale)s %(m_scale)s"/>
      </geometry>
    </visual>
      <collision>
        <!-- collision origin is defined w.r.t. link local coordinate system -->
        <origin xyz="%(bound_x)s %(bound_y)s %(bound_z)s" rpy="%(bound_roll)s %(bound_pitch)s %(bound_yaw)s" />
        <geometry>
          <box size="%(dim_x)s %(dim_y)s %(dim_z)s"/>
        </geometry>
      </collision>
  </link>
"""

generic_sensor_link = """
  <!--THIS IS THE SENSOR LINK RENAME ME-->
  <link name="ref_%(refID)s_link">
    <inertial>
      <mass value="0.023900" />
      <!-- center of mass (com) is defined w.r.t. link local coordinate system -->
      <origin xyz="0 0 0" />
      <inertia  ixx="0.01" ixy="0.0"  ixz="0.0"  iyy="0.01"  iyz="0.0"  izz="0.01" />
    </inertial>
    <visual>
      <!-- visual origin is defined w.r.t. link local coordinate system -->
      <origin xyz=" -0.013 0 -0.018" rpy="0 0 1.57079633" />
      <geometry>
        <mesh filename="%(mesh)s" scale="%(m_scale)s %(m_scale)s %(m_scale)s"/>
      </geometry>
    </visual>
      <collision>
        <!-- collision origin is defined w.r.t. link local coordinate system -->
        <origin xyz="%(bound_x)s %(bound_y)s %(bound_z)s" rpy="%(bound_roll)s %(bound_pitch)s %(bound_yaw)s" />
        <geometry>
          <box size="%(dim_x)s %(dim_y)s %(dim_z)s"/>
        </geometry>
      </collision>
  </link>
"""

def parseLXFML(handle, lxffile,ldrfile):
  #begin urdf

  print "<!--this file was autogenerated from %s -->" % lxffile
  ldr_file = open('%s' % ldrfile,'r')
  print "<robot name=nxt_%s>" % lxffile.strip('.lxf').rsplit("/")[0]
  lxf_doc = xml.dom.minidom.parse(handle)

  bricks = {}
  parts = {}
  bones = {}

  for brick in lxf_doc.getElementsByTagName('Brick'):
    b = {}
    b['refID']= int(brick.getAttribute('refID'))
    b['parts'] = []
    b['designID'] = int(brick.getAttribute('designID'))
    b['handled']=False
    i = parseInts(brick.getAttribute('itemNos'))
    if i:
      b['itemNos'] = i
    bricks[int(brick.getAttribute('refID'))] = b

    for part in brick.getElementsByTagName('Part'):
      p = {}
      p['bones'] = []
      p['designID'] = part.getAttribute('designID')
      p['materials'] = parseInts(part.getAttribute('materials'))
      parts[int(part.getAttribute('refID'))] = p
      b['parts'].append(p)
      p['parent'] = b

      for bone in part.getElementsByTagName('Bone'):
        d = {}
        d['transformation'] = parseFloats(bone.getAttribute('transformation'))
        bones[int(bone.getAttribute('refID'))] = d
        p['bones'].append(d)
        d['parent'] = p

  rigids = {}
  for rigid in lxf_doc.getElementsByTagName('Rigid'):
    r = {}
    r['refID'] = int(rigid.getAttribute('refID'))
    r['transformation'] = parseFloats(rigid.getAttribute('transformation'))
    r['boneRefs'] = parseInts(rigid.getAttribute('boneRefs'))
    r['handled'] = False
    rigids[r['refID']] = r

  joints = []
  for joint in lxf_doc.getElementsByTagName('Joint'):
    j = []
    for rigid_ref in joint.getElementsByTagName('RigidRef'):
      r = {}
      r['rigidRef'] = int(rigid_ref.getAttribute('rigidRef'))
      r['a'] = parseFloats(rigid_ref.getAttribute('a'))
      r['z'] = parseFloats(rigid_ref.getAttribute('z'))
      r['t'] = parseFloats(rigid_ref.getAttribute('t'))
      j.append(r)
    joints.append(j)


  ldr_trans={}
  count=0
  for line in ldr_file:
    if line.startswith('1'):
      ldr= [x for x in line.split(' ')]
      l={}
      l['ldraw'] = ldr[14].strip('.dat\r\n')
      l['transformation'] = [float(ldr[2]), float(ldr[3]), float(ldr[4]), float(ldr[5]), float(ldr[6]), float(ldr[7]), float(ldr[8]), float(ldr[9]), float(ldr[10]), float(ldr[11]), float(ldr[12]), float(ldr[13])]
      ldr_trans[count]=l
      count=count+1

  for refID in sorted(bricks.keys()):
    designID = ldr_trans[refID]['ldraw']
    ldrID =ldr_trans[refID]['ldraw']
    rot_x=rot_y=rot_z=0
    #these parts with odd bends need a 180 rotation
    if ldrID == '6629' or  ldrID == '32348' or ldrID == '32140' or ldrID == '32526':
      rot_y = 3.14159
    #scaling factor from the .dat models to meters
    scale =0.0004
    d = {
      'refID' : refID,
      'mesh' : "package://nxt_description/meshes/parts/%s.dae" % designID,
      'bound_x' : 0,
      'bound_y' : 0,
      'bound_z' : 0,
      'm_scale' :' %s' % str(scale),
      'bound_roll' : 0,
      'bound_pitch' : 0,
      'bound_yaw' : 0,
      'dim_x' : 0,
      'dim_y' : 0,
      'dim_z' : 0,
      'rot_x': '%s' % str(rot_x),
      'rot_y': '%s' % str(rot_y),
      'rot_z': '%s' % str(rot_z),
    }
    #special templates for sensors and motors
    if ldr_trans[refID]['ldraw'] == '53787':
      print motor_template % d
    elif ldr_trans[refID]['ldraw'] == '53792':
      print ultrasonic_link % d
    elif ldr_trans[refID]['ldraw'] == '1044' or ldr_trans[refID]['ldraw'] == '64892' or  ldr_trans[refID]['ldraw'] == '53793':
      print generic_sensor_link % d
    else:
      print link_template % d
  #create tree with no loops 
  create_rigid_tree(0, joints, rigids)
  create_brick_tree(rigids, bricks, bones)
  #for refID, joint_list in enumerate(joints):
  for refID, branch in enumerate(brick_tree):
    joint_type = "fixed"
    child_refID = branch[1]
    parent_refID = branch[0] 
    #all units are in CM
    #the models are in LDU which are 0.4mm
    #let's compute some transforms
    world_to_c = homogeneous_matrix(ldr_trans[child_refID]['transformation'])
    world_to_p = homogeneous_matrix(ldr_trans[parent_refID]['transformation'])
    #now let's get the stuff for the URDF
    p_to_c = world_to_p.I*world_to_c
    rpy = TF.euler_from_matrix(p_to_c, 'sxzy')
      
    d = {
    'refID' : refID,
    'joint_type' : joint_type,
    'parent_link' : 'ref_%s_link' % parent_refID,
    'child_link' : 'ref_%s_link' % child_refID,
    #a straight transform doesn't seem to work properly (sign changes as needed)
    'origin_x' : '%s' % str(-1*float(p_to_c[0,3])*scale),
    'origin_y' : '%s'% str(1*float(p_to_c[2,3])*scale),
    'origin_z' : '%s' % str(-1*float(p_to_c[1,3])*scale),
    'origin_roll' : '%s' % str(1*rpy[0]),
    'origin_pitch' : '%s' % str(-1*rpy[1]),
    'origin_yaw' : '%s' % str(1*rpy[2]),
    'axis_x' : 0, 'axis_y' : 0, 'axis_z' : 0,
    }
    print joint_template % d
  #end urdf 
  print "</robot>"

def create_rigid_tree(refID, joints, rigids):
  if not rigids[refID]['handled']:
    rigids[refID]['handled']=True
    parent =refID
    for joint_list in joints:
      if joint_list[0]['rigidRef'] == parent:
        if not rigids[joint_list[1]['rigidRef']]['handled']:
          rigid_tree.append([parent, joint_list[1]['rigidRef']])
          create_rigid_tree(joint_list[1]['rigidRef'], joints, rigids)
      if joint_list[1]['rigidRef'] == parent:
        if not rigids[joint_list[0]['rigidRef']]['handled']:
          rigid_tree.append([parent, joint_list[0]['rigidRef']])
          create_rigid_tree(joint_list[0]['rigidRef'], joints, rigids)

def create_brick_tree(rigids, bricks, bones):
  for branch in rigid_tree:
    parent_brick = bones[rigids[branch[0]]['boneRefs'][0]]['parent']['parent']['refID']
    for bone in rigids[branch[1]]['boneRefs']:
      child_brick = bones[bone]['parent']['parent']['refID']
      if not bricks[child_brick]['handled']:
        brick_tree.append([parent_brick,child_brick])
        bricks[child_brick]['handled']=True

def homogeneous_matrix(transform):
  tmp = numpy.ones((4,4))
  tmp[:3,:3] = (numpy.array(transform[3:]).reshape(3,3))
  tmp[0,3] = numpy.transpose(numpy.array(transform[0]))
  tmp[1,3] = numpy.transpose(numpy.array(transform[1]))
  tmp[2,3] = numpy.transpose(numpy.array(transform[2]))
  tmp[3,:3] = numpy.zeros((1,3))
  return numpy.matrix(tmp)

def handleLXF(lxffile,ldrfile):
  z = zipfile.ZipFile(lxffile)
  for item in z.namelist():
    if item.endswith('.LXFML'):
      f = z.open(item)
      parseLXFML(f,lxffile,ldrfile)

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "debug"])

  if len(args) == 0:
    usage(progname)
    return
  
  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      debugfull()
  try:
     handleLXF(args[0], args[1])
  except Exception, reason:
     print >> sys.stderr, "Unable to handle '%s': %s" % (args[0], reason)

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
