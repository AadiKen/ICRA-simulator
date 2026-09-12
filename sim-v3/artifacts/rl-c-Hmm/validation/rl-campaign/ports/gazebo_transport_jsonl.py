#!/usr/bin/env python3
"""Persistent in-container Gazebo Transport facade for control and thrust."""
import json,sys
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean

node=Node();publishers={}
for line in sys.stdin:
 try:
  request=json.loads(line);op=request.get("op")
  if op=="publish":
   topic=request["topic"];publisher=publishers.get(topic)
   if publisher is None:publisher=node.advertise(topic,Double);publishers[topic]=publisher
   message=Double();message.data=float(request["value"]);ok=publisher.publish(message);response={"ok":bool(ok)}
  elif op=="control":
   message=WorldControl();message.pause=True
   if "run_to_sim_time_s" in request:
    message.pause=False
    target=float(request["run_to_sim_time_s"]);seconds=int(target);nanoseconds=round((target-seconds)*1e9)
    if nanoseconds>=1_000_000_000:seconds+=1;nanoseconds-=1_000_000_000
    message.run_to_sim_time.sec=seconds;message.run_to_sim_time.nsec=nanoseconds
   else:message.multi_step=int(request.get("multi_step",0))
   executed,result=node.request(request["service"],message,WorldControl,Boolean,10000);response={"ok":bool(executed and result.data)}
  elif op=="close":response={"ok":True,"closed":True}
  else:raise ValueError("unsupported transport operation")
 except Exception as error:response={"ok":False,"error":f"{type(error).__name__}: {error}"}
 print(json.dumps(response,separators=(",",":")),flush=True)
 if response.get("closed"):break
