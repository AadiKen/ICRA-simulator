#!/usr/bin/env python3
"""Infer the live VRX planar Coriolis/bias vector from coast telemetry."""
import argparse, csv, json, math
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument("telemetry",type=Path);p.add_argument("output",type=Path);p.add_argument("--start-time",type=float,default=1.9);p.add_argument("--dt",type=float,default=.05);a=p.parse_args()
    rows=list(csv.DictReader(a.telemetry.open()));start=min(rows,key=lambda x:abs(float(x["time_s"])-a.start_time));end=min(rows,key=lambda x:abs(float(x["time_s"])-(a.start_time+a.dt)))
    u=float(start["velocity_body_x"]);v=-float(start["velocity_body_y"]);r=-float(start["angular_velocity_body_z"])
    ud=(float(end["velocity_body_x"])-float(start["velocity_body_x"]))/a.dt
    vd=-(float(end["velocity_body_y"])-float(start["velocity_body_y"]))/a.dt
    rd=-(float(end["angular_velocity_body_z"])-float(start["angular_velocity_body_z"]))/a.dt
    mass=[52.3+2.615,52.3+39.225,17.891219833333338+1.431297586666667]
    damping=[-(6*u+18*abs(u)*u),-(18*v+60*abs(v)*v),-(8*r+12*abs(r)*r)]
    implied=[damping[i]-mass[i]*[ud,vd,rd][i] for i in range(3)]
    rb=[-52.3*v*r,52.3*u*r,0];added=[-39.225*v*r,2.615*u*r,(39.225-2.615)*u*v]
    node=[rb[i]+added[i] for i in range(3)];diff=[implied[i]-node[i] for i in range(3)]
    report={"schema_version":1,"status":"INVALID_MIXED_TIME_DISCRETIZATION","interval_s":[float(start["time_s"]),float(end["time_s"])],"state_body_ned":{"velocity_mps":[u,v],"yaw_rate_rad_s":r},"body_velocity_derivative_interval_average":[ud,vd,rd],"effective_mass_diagonal":mass,"start_of_interval_damping_force_n_nm":damping,"invalid_implied_bias_n_nm":implied,"start_of_interval_node_theoretical_bias_n_nm":node,"invalid_implied_minus_node_n_nm":diff,"difference_l2_n_nm":math.sqrt(sum(x*x for x in diff)),"attempted_force_balance":"M*nu_dot_interval = F_damping(nu_start) - C(nu_start)*nu_start","valid":False,"limitation":"The equation mixes an interval-average acceleration with forces evaluated only at the interval start. It is not an instantaneous force balance and cannot recover DART's live solver bias vector.","conclusion":"The previous live-bias match is withdrawn. These numbers neither clear nor implicate DART's bias computation; a direct instantaneous solver value or a correctly time-integrated force balance is required."}
    a.output.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2))
if __name__=="__main__":main()
