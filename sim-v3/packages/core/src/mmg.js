import {ForceModel} from "./force-model.js";
const value=(entry)=>typeof entry==="number"?entry:entry?.value;
const clamp=(x,a,b)=>Math.min(Math.max(x,a),b);

export function createMmgParameters(reference,{rho=1025}={}){
  const p=reference.principal_particulars,h=reference.hull_derivatives,a=reference.added_mass,i=reference.interaction_coefficients,prop=reference.propeller;
  const L=value(p.lpp),d=value(p.draft),volume=value(p.displacement_volume),mass=rho*volume,kzz=value(reference.test_conditions.yaw_gyration_radius_ratio)*L;
  return{id:reference.id,sourceVessel:"KVLCC2",rho,L,d,B:value(p.breadth),volume,mass,xG:value(p.x_g),Iz:mass*kzz*kzz,DP:value(p.propeller_diameter),AR:value(p.rudder_area),rudderSpan:value(p.rudder_span),xR:-0.5*L,hull:Object.fromEntries(Object.entries(h).map(([k,v])=>[k,value(v)])),added:{mx:value(a.m_x)*0.5*rho*L*L*d,my:value(a.m_y)*0.5*rho*L*L*d,Jz:value(a.J_z)*0.5*rho*L**4*d},interaction:Object.fromEntries(Object.entries(i).map(([k,v])=>[k,value(v)])),propeller:{k0:value(prop.k0),k1:value(prop.k1),k2:value(prop.k2),wP0:value(prop.w_P0)},provenance:{reference:"Yasukawa & Yoshimura 2015",parameterSet:"kvlcc2-yy2015-table3",evidenceScope:"model-structure-reference-reproduction-only",capytaineAddedMassOverwrite:false}};
}

export class MmgManeuveringModel extends ForceModel{
  constructor(params){super();this.params=params;this.lastBreakdown={};this.lastFullWrench=[0,0,0,0,0,0];}
  computeComponents(state,command={}){
    const p=this.params,u=state.velocity?.u??state.u??0,v=state.velocity?.v??state.v??0,r=state.angularRate?.r??state.r??0,delta=command.rudder_rad??command.delta??0,nP=command.propeller_rps??command.nP??0;
    const U=Math.max(Math.hypot(u,v),1e-6),vp=v/U,rp=r*p.L/U,h=p.hull,forceScale=0.5*p.rho*p.L*p.d*U*U,momentScale=forceScale*p.L;
    const resistanceScale=command.resistance_reference_speed_mps?U*U/(command.resistance_reference_speed_mps**2):1;
    const hull=[forceScale*(h.X_vv*vp*vp+h.X_vr*vp*rp+h.X_rr*rp*rp+h.X_vvvv*vp**4)-(command.straight_resistance_n??0)*resistanceScale,forceScale*(h.Y_v*vp+h.Y_R*rp+h.Y_vvv*vp**3+h.Y_vvr*vp*vp*rp+h.Y_vrr*vp*rp*rp+h.Y_rrr*rp**3),momentScale*(h.N_v*vp+h.N_R*rp+h.N_vvv*vp**3+h.N_vvr*vp*vp*rp+h.N_vrr*vp*rp*rp+h.N_rrr*rp**3)];
    const beta=Math.atan2(-v,Math.max(Math.abs(u),1e-9)),betaP=beta-(command.xP_prime??-0.5)*rp,C2=betaP>=0?p.interaction.C2_beta_P_positive:p.interaction.C2_beta_P_negative;
    const wakeRatio=1+(1-Math.exp(-p.interaction.C1*Math.abs(betaP)))*(C2-1),wP=1-(1-p.propeller.wP0)*wakeRatio;
    const advance=nP===0?0:(1-wP)*u/(nP*p.DP),KT=p.propeller.k0+p.propeller.k1*advance+p.propeller.k2*advance*advance,thrust=p.rho*nP*Math.abs(nP)*p.DP**4*KT,propeller=[(1-p.interaction.t_P)*thrust,0,0];
    const betaR=beta-p.interaction.l_R*rp,gamma=betaR<0?p.interaction.gamma_R_beta_R_negative:p.interaction.gamma_R_beta_R_positive,vR=U*gamma*betaR,eta=p.DP/Math.max(p.rudderSpan,1e-9),slip=advance===0?0:Math.sqrt(Math.max(1+8*KT/(Math.PI*advance*advance),0))-1;
    const uR=p.interaction.epsilon*(1-wP)*u*Math.sqrt(Math.max(eta*(1+p.interaction.kappa*slip)**2+(1-eta),0)),alphaR=delta-Math.atan2(vR,uR),UR=Math.hypot(uR,vR),FN=0.5*p.rho*p.AR*p.interaction.f_alpha*UR*UR*Math.sin(alphaR);
    const XR=-(1-p.interaction.t_R)*FN*Math.sin(delta),YR=-(1+p.interaction.a_H)*FN*Math.cos(delta),NR=-(p.xR+p.interaction.a_H*p.interaction.x_H*p.L)*FN*Math.cos(delta),rudder=[XR,YR,NR],total=hull.map((x,index)=>x+propeller[index]+rudder[index]);
    this.lastBreakdown={hull,propeller,rudder,total,diagnostics:{U,v_prime:vp,r_prime:rp,beta,betaP,wakeRatio,wP,advance_ratio:advance,KT,betaR,uR,vR,alphaR,FN}};this.lastFullWrench=[total[0],total[1],0,0,0,total[2]];return this.lastBreakdown;
  }
  computeWrench(ctx){return this.computeComponents(ctx.state,ctx.command).total;}
}

export class MmgPlanarSimulator{
  constructor(params,{dt=0.02,rudderRateRadS=Infinity,integrator="rk4"}={}){this.params=params;this.dt=dt;this.rudderRateRadS=rudderRateRadS;this.integrator=integrator;this.model=new MmgManeuveringModel(params);}
  derivative(s,command){const b=this.model.computeComponents(s,command),p=this.params,m11=p.mass+p.added.mx,m22=p.mass+p.added.my,m66=p.Iz+p.added.Jz+p.mass*p.xG*p.xG,cross=p.mass*p.xG,X=b.total[0],Y=b.total[1],N=b.total[2],uDot=(X+(p.mass+p.added.my)*s.v*s.r+p.mass*p.xG*s.r*s.r)/m11,rhsY=Y-(p.mass+p.added.mx)*s.u*s.r,rhsN=N-p.mass*p.xG*s.u*s.r,det=m22*m66-cross*cross,vDot=(rhsY*m66-cross*rhsN)/det,rDot=(m22*rhsN-cross*rhsY)/det,cos=Math.cos(s.psi),sin=Math.sin(s.psi);return{x:s.u*cos-s.v*sin,y:s.u*sin+s.v*cos,psi:s.r,u:uDot,v:vDot,r:rDot};}
  step(state,command){const desired=clamp(command.rudder_rad??0,-Math.PI/2,Math.PI/2),delta=state.delta??0,maxChange=this.rudderRateRadS*this.dt;state.delta=delta+clamp(desired-delta,-maxChange,maxChange);const applied={...command,rudder_rad:state.delta},keys=["x","y","psi","u","v","r"];
    if(this.integrator!=="rk4"){const d=this.derivative(state,applied);for(const key of keys)state[key]+=d[key]*this.dt;return state;}
    const shifted=(base,d,h)=>Object.fromEntries([...keys.map((key)=>[key,base[key]+d[key]*h]),["delta",base.delta]]),k1=this.derivative(state,applied),k2=this.derivative(shifted(state,k1,this.dt/2),applied),k3=this.derivative(shifted(state,k2,this.dt/2),applied),k4=this.derivative(shifted(state,k3,this.dt),applied);for(const key of keys)state[key]+=this.dt*(k1[key]+2*k2[key]+2*k3[key]+k4[key])/6;return state;
  }
}
