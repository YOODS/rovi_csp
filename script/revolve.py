#!/usr/bin/python

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import sys
from std_msgs.msg import Bool
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from rovi_utils import tflib
from scipy.spatial.transform import Rotation as R

def lookup(a,b):
  try:
    sys.stdout.write("//lookup "+a+" "+b+"\n")
    sys.stdout.flush()
    aTb=tfBuffer.lookup_transform(a,b,rospy.Time(0))
  except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
    sys.stdout.write("//lookup exception\n")
    sys.stdout.flush()
    return None
  return aTb

def cb_do(msg):
  if msg.data is False:
    pub_done.publish(mFalse)
    return
  stj=lookup("camera/master0","camera/master0/journal")
  stj.header.frame_id="camera/capture0/solve0"
  stj.child_frame_id="camera/capture0/solve0/journal"
  cts=lookup("camera/capture0","camera/capture0/solve0")
  cTs=tflib.toRT(cts.transform)
  bts=lookup("base","camera/capture0/solve0")
  bTs=tflib.toRT(bts.transform)
  sTj=tflib.toRT(stj.transform)
  jTs=sTj.I
  jTc=jTs.dot(cTs.I)
  jTb=jTs.dot(bTs.I)
  bsz=np.ravel(jTs[:3,2])
  bcz=np.ravel(jTc[:3,2])
  bbz=np.ravel(jTb[:3,2])
  bsz[0]=0
  bcz[0]=0
  bbz[0]=0
  bsz=bsz/np.linalg.norm(bsz)
  bcz=bcz/np.linalg.norm(bcz)
  bbz=-bbz/np.linalg.norm(bbz)
#  rot=np.cross(bsz,bcz)
  rot=np.cross(bsz,bbz)
  lrot=np.linalg.norm(rot)
  arot=np.arcsin(lrot);
  if lrot>0:
    rot=rot/lrot*arot
  r=R.from_rotvec(rot)
  RT=np.eye(4)
  RT[:3,:3]=r.as_matrix()
  jTr=RT.dot(jTs)
  jtr=TransformStamped()
  jtr.header.frame_id="camera/capture0/solve0/journal"
  jtr.child_frame_id="camera/capture0/solve0/revolve"
  jtr.header.stamp=rospy.Time.now()
  jtr.transform=tflib.fromRT(jTr)
  broadcaster.sendTransform([stj,jtr])
  rospy.Timer(rospy.Duration(0.1),lambda event: pub_done.publish(mTrue),oneshot=True)

########################################################
rospy.init_node("revolver",anonymous=True)
#Config.update(parse_argv(sys.argv))
#try:
#  Config.update(rospy.get_param("~config"))
#except Exception as e:
#  print("get_param exception:",e.args)
#try:
#  Param.update(rospy.get_param("~param"))
#except Exception as e:

###Topics Service
rospy.Subscriber("~do",Bool,cb_do)
pub_done=rospy.Publisher("~done",Bool,queue_size=1)

###Globals
mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
