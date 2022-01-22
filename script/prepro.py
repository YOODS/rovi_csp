#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import open3d as o3d
import os
import sys
import subprocess
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from rovi_utils import tflib

Param={
  "box0_width":50,
  "box0_depth":30,
  "box0_points":1000,
}
Config={
  "user_frame_id":"bucket",
  "bound0_frame_ids":["bucket_edge0","bucket_edge1"],
  "bound1_frame_ids":["bucket_edge2","bucket_edge3"]
}

def cb_redraw(msg):
  pub_ps.publish(np2F(Scene))

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

def pTr(RT,pc):
  return np.dot(RT[:3],np.vstack((pc.T,np.ones((1,len(pc)))))).T

def pCountYZ(pc,y0,y1,dy,z0,z1,dz):
  pitch=5
  count=0
  for y in np.arange(y0,y1,pitch):
    pcy=pc[ np.ravel(np.abs(pc[:,1]-y)<dy) ]
    for z in np.arange(z0,z1,pitch):
      n=len(pcy[ np.ravel(np.abs(pcy[:,2]-z)<dz) ])
      if count<n: count=n
  return count

def pCrop(ids,pc,wid,dep):
  bnd0=getRT(Config["user_frame_id"],ids[0])
  bnd1=getRT(Config["user_frame_id"],ids[1])
  xm=(bnd0[0,3]+bnd1[0,3])/2 #x center of bounding box
  xh=np.abs(bnd0[0,3]-xm)    #x width of bounding box
  ym=(bnd0[1,3]+bnd1[1,3])/2
  yh=np.abs(bnd0[1,3]-ym)
  print("prepro crop",ym,yh)
  pc=pc[ np.ravel( np.abs(pc[:,0]-xm)<xh ) & np.ravel( np.abs(pc[:,1]-ym)<yh ) ]
  cn=pCountYZ(pc,ym-yh,ym+yh,wid/2,0,1000,dep/2)
  return pc,cn

def pCropX(ids,pc):
  bnd0=getRT(Config["user_frame_id"],ids[0])
  bnd1=getRT(Config["user_frame_id"],ids[1])
  xm=(bnd0[0,3]+bnd1[0,3])/2 #x center of bounding box
  xh=np.abs(bnd0[0,3]-xm)    #x width of bounding box
  pc=pc[ np.ravel( np.abs(pc[:,0]-xm)<xh ) ]
  return pc

def cb_ps(msg):
  global Scene
  Scene=np.reshape(msg.data,(-1,3))
  pub_ps.publish(np2F(Scene))
  return

def cb_solve(msg):
  global Scene
  try:
    Param.update(rospy.get_param("/prepro"))
  except Exception as e:
    print("get_param exception:",e.args)
  print("prepro param",Param)
  report=String()
  cTu=getRT("camera/capture0",Config["user_frame_id"])
  uTc=np.linalg.inv(cTu)
  uscn=pTr(uTc,Scene)
  print("Scene",len(Scene))
  if Param["crop_edge"]:
    uscn0,cnt0=pCrop(Config["bound0_frame_ids"],uscn,Param["box0_width"],Param["box0_depth"])
    print("count0",len(uscn0),cnt0)
    uscn1,_=pCrop(Config["bound1_frame_ids"],uscn,0,0)
    Scene=pTr(cTu,np.vstack((uscn0,uscn1)))
    if cnt0>Param["box0_points"]:
      report.data='{"volume":('+str(cnt0)+',0)}'
      rospy.Timer(rospy.Duration(0.1),lambda ev: pub_thru.publish(mTrue),oneshot=True)
    else:
      report.data='{"volume":('+str(cnt0)+',201)}'
      rospy.Timer(rospy.Duration(0.1),lambda ev: pub_cut.publish(mFalse),oneshot=True)
  else:
    uscn0=pCropX(Config["bound0_frame_ids"],uscn)
    uscn1=pCropX(Config["bound1_frame_ids"],uscn)
    Scene=pTr(cTu,np.vstack((uscn0,uscn1)))
    report.data='{"volume":('+str(len(Scene))+',0)}'
    rospy.Timer(rospy.Duration(0.1),lambda ev: pub_thru.publish(mTrue),oneshot=True)
  print("Scene",len(Scene))
  cb_redraw(0)
  pub_str.publish(report)

########################################################
rospy.init_node("prepro",anonymous=True)
thispath=subprocess.getoutput("rospack find rovi_sim")
###Load params
try:
  Config.update(rospy.get_param("/config/prepro"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("~in/floats",numpy_msg(Floats),cb_ps)
rospy.Subscriber("/request/solve",Bool,cb_solve)
rospy.Subscriber("/request/redraw",Bool,cb_redraw)
pub_ps=rospy.Publisher("~out/floats",numpy_msg(Floats),queue_size=1)
pub_str=rospy.Publisher("/report",String,queue_size=1)
pub_thru=rospy.Publisher("~passthru",Bool,queue_size=1)
pub_cut=rospy.Publisher("~shortcut",Bool,queue_size=1)
###Bool message
mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False
###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

#if __name__=="__main__":
#

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
