const finite=(value)=>typeof value==="number"&&Number.isFinite(value);
const interpolate=(points,speed)=>{const sorted=[...points].sort((a,b)=>a.speed_mps-b.speed_mps);if(speed<sorted[0].speed_mps||speed>sorted.at(-1).speed_mps)throw new Error(`Resistance table does not cover ${speed} m/s.`);const hi=sorted.findIndex((point)=>point.speed_mps>=speed);if(hi===0)return sorted[0].resistance_n;const a=sorted[hi-1],b=sorted[hi],f=(speed-a.speed_mps)/(b.speed_mps-a.speed_mps);return a.resistance_n+f*(b.resistance_n-a.resistance_n);};

export function schoenherrFrictionCoefficient(reynolds,{tolerance=1e-13,maxIterations=100}={}){
  if(!(reynolds>0))throw new Error("Schoenherr reconstruction requires a positive Reynolds number.");
  let coefficient=0.075/(Math.log10(reynolds)-2)**2;
  for(let iteration=0;iteration<maxIterations;iteration++){const next=0.242/(Math.log10(reynolds*coefficient))**2;if(Math.abs(next-coefficient)<tolerance)return next;coefficient=next;}
  throw new Error("Schoenherr friction-coefficient solve did not converge.");
}

export function resolveResistance(model,speed,context={}){
  if(!model?.type)throw new Error("A resistance model is required.");
  const rho=context.rho??1025,L=context.L,d=context.d;
  let resistance,details={type:model.type};
  if(model.type==="direct_dimensional")resistance=model.resistance_n;
  else if(model.type==="nondimensional_r0_prime")resistance=model.r0_prime*0.5*rho*L*d*speed**2;
  else if(model.type==="polynomial")resistance=model.coefficients_n.reduce((sum,c,index)=>sum+c*speed**index,0);
  else if(model.type==="table")resistance=interpolate(model.points,speed);
  else if(model.type==="callback"){
    if(typeof model.evaluate!=="function")throw new Error("User-supplied resistance callback is missing evaluate().");
    resistance=model.evaluate(speed,context);
  }else if(model.type==="equilibrium_at_fixed_shaft_rate"){
    if(typeof context.effectivePropellerThrust!=="function")throw new Error("Equilibrium resistance requires an effectivePropellerThrust callback.");
    resistance=context.effectivePropellerThrust(model.propeller_rps,speed);details.assumption="Resistance inferred solely to trim straight-ahead net surge force; not a measured resistance curve.";
  }else if(model.type==="schoenherr"){
    const required=["wetted_surface_m2","kinematic_viscosity_m2_s","form_factor","residual_resistance_coefficient","correlation_allowance_coefficient","appendage_allowance_coefficient","air_resistance_n"];
    for(const field of required)if(!finite(model[field]))throw new Error(`Schoenherr reconstruction requires explicit ${field}.`);
    const reynolds=speed*L/model.kinematic_viscosity_m2_s,frictionCoefficient=schoenherrFrictionCoefficient(reynolds),coefficient=(1+model.form_factor)*frictionCoefficient+model.residual_resistance_coefficient+model.correlation_allowance_coefficient+model.appendage_allowance_coefficient;
    resistance=0.5*rho*model.wetted_surface_m2*speed**2*coefficient+model.air_resistance_n;details={...details,wetted_surface_m2:model.wetted_surface_m2,reynolds,friction_coefficient:frictionCoefficient,form_factor:model.form_factor,residual_resistance_coefficient:model.residual_resistance_coefficient,correlation_allowance_coefficient:model.correlation_allowance_coefficient,appendage_allowance_coefficient:model.appendage_allowance_coefficient,air_resistance_n:model.air_resistance_n};
  }else throw new Error(`Unsupported resistance model ${model.type}.`);
  if(!finite(resistance)||resistance<0)throw new Error("Resolved resistance must be finite and non-negative.");
  return{resistance_n:resistance,details,provenance:model.provenance};
}

export function solvePropellerEquilibrium({model,params,speed_mps,resistance_model,tolerance=1e-12,maxIterations=100}){
  const resolved=resolveResistance(resistance_model,speed_mps,params),effective=(rps)=>model.computeComponents({u:speed_mps,v:0,r:0},{propeller_rps:rps}).propeller[0];let low=0,high=30;
  while(effective(high)<resolved.resistance_n&&high<1e5)high*=2;
  for(let iteration=0;iteration<maxIterations;iteration++){const mid=(low+high)/2;if(effective(mid)<resolved.resistance_n)low=mid;else high=mid;if((high-low)/Math.max(high,1)<tolerance)break;}
  const propeller_rps=(low+high)/2,components=model.computeComponents({u:speed_mps,v:0,r:0},{propeller_rps}),residual=components.propeller[0]-resolved.resistance_n;
  return{propeller_rps,propeller_rpm:60*propeller_rps,resistance:resolved,components,residual_n:residual,normalized_residual:Math.abs(residual)/Math.max(resolved.resistance_n,1)};
}
