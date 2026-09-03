import assert from "node:assert/strict";
import test from "node:test";
import {dampingWrench3} from "../../../packages/core/src/damping.ts";

const near=(a:number,b:number,tol=1e-12)=>assert.ok(Math.abs(a-b)<=tol,`${a} != ${b}`);
const bodyNed=(N:number,E:number,yaw:number)=>({u:N*Math.cos(yaw)+E*Math.sin(yaw),v:-N*Math.sin(yaw)+E*Math.cos(yaw)});
const bodyFlu=(E:number,N:number,yawEnu:number)=>({x:E*Math.cos(yawEnu)+N*Math.sin(yawEnu),y:-E*Math.sin(yawEnu)+N*Math.cos(yawEnu)});
const nodeToWorld=(fx:number,fy:number,yaw:number)=>({N:fx*Math.cos(yaw)-fy*Math.sin(yaw),E:fx*Math.sin(yaw)+fy*Math.cos(yaw)});
const fluToWorld=(fx:number,fy:number,yawEnu:number)=>({E:fx*Math.cos(yawEnu)-fy*Math.sin(yawEnu),N:fx*Math.sin(yawEnu)+fy*Math.cos(yawEnu)});

test("prescribed moving-frame current and wind transforms match Node",()=>{
  const cases=[
    {yaw:0,flow:{N:2,E:.5},velocity:{N:.4,E:-.2}},
    {yaw:Math.PI/2,flow:{N:-.3,E:1.7},velocity:{N:.8,E:.1}},
    {yaw:-Math.PI/4,flow:{N:1.2,E:-.9},velocity:{N:-.2,E:.6}}
  ];
  for(const c of cases){
    const yawEnu=Math.PI/2-c.yaw;
    const nodeFlow=bodyNed(c.flow.N,c.flow.E,c.yaw),nodeVelocity=bodyNed(c.velocity.N,c.velocity.E,c.yaw);
    const fluFlow=bodyFlu(c.flow.E,c.flow.N,yawEnu),fluVelocity=bodyFlu(c.velocity.E,c.velocity.N,yawEnu);
    near(fluFlow.x,nodeFlow.u);near(fluFlow.y,-nodeFlow.v);near(fluVelocity.x,nodeVelocity.u);near(fluVelocity.y,-nodeVelocity.v);
    const relativeNode={u:nodeFlow.u-nodeVelocity.u,v:nodeFlow.v-nodeVelocity.v};
    const speed=Math.hypot(relativeNode.u,relativeNode.v),scale=.5*1.225*1.1*speed;
    const nodeForce={x:scale*.3822*relativeNode.u,y:scale*.7686*relativeNode.v};
    const vrxForce={x:nodeForce.x,y:-nodeForce.y};
    const nw=nodeToWorld(nodeForce.x,nodeForce.y,c.yaw),vw=fluToWorld(vrxForce.x,vrxForce.y,yawEnu);
    near(vw.N,nw.N);near(vw.E,nw.E);
  }
});

test("stock USVWind world-axis drag is not a Surveyor body-profile serialization",()=>{
  const yaw=Math.PI/3,yawEnu=Math.PI/2-yaw,flow={N:1.1,E:2.0},velocity={N:.4,E:-.3};
  const rel={N:flow.N-velocity.N,E:flow.E-velocity.E},body=bodyNed(rel.N,rel.E,yaw),speed=Math.hypot(body.u,body.v),scale=.5*1.225*1.1*speed;
  const expected=nodeToWorld(scale*.3822*body.u,scale*.7686*body.v,yaw);
  const stock={E:.25754575*rel.E*Math.abs(rel.E),N:.51783225*rel.N*Math.abs(rel.N)};
  assert.ok(Math.hypot(expected.E-stock.E,expected.N-stock.N)>.1);
});

test("prescribed current forces match before integration or hull dynamics",()=>{
  const params={damping:{linear:{Xu:6,Yv:18,Nr:8},quadratic:{Xuu:18,Yvv:60,Nrr:12}}};
  const states=[
    {velocity:[0,0,0],current:[.18,.07]},
    {velocity:[.4,-.2,.1],current:[.18,.07]},
    {velocity:[-.3,.6,-.15],current:[-.1,.25]},
    {velocity:[1.2,.8,.3],current:[.5,-.4]}
  ];
  const damp=(v:number,l:number,q:number)=>-(l*v+q*Math.abs(v)*v);
  for(const state of states){
    const relative=[state.velocity[0]-state.current[0],state.velocity[1]-state.current[1],state.velocity[2]];
    const node=dampingWrench3(params,relative);
    const base=[damp(state.velocity[0],6,18),damp(state.velocity[1],18,60),damp(state.velocity[2],8,12)];
    const correction=[damp(relative[0],6,18)-base[0],damp(relative[1],18,60)-base[1],0];
    near(base[0]+correction[0],node[0]);
    near(base[1]+correction[1],node[1]);
    near(base[2]+correction[2],node[2]);
  }
});
