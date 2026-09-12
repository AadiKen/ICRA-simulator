#include <cmath>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>

namespace vrx_surveyor {
class CurrentRelativeVelocity final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm,
      gz::sim::EventManager &) override {
    gz::sim::Model model(entity);
    this->link = model.LinkByName(ecm, sdf->Get<std::string>("link_name", "base_link").first);
    this->current = sdf->Get<gz::math::Vector3d>("current_enu", gz::math::Vector3d::Zero).first;
    this->xu=sdf->Get<double>("xU",6).first; this->xuu=sdf->Get<double>("xUU",18).first;
    this->yv=sdf->Get<double>("yV",18).first; this->yvv=sdf->Get<double>("yVV",60).first;
    this->xuDot=sdf->Get<double>("xDotU",-2.615).first;
    this->yvDot=sdf->Get<double>("yDotV",-39.225).first;
    const auto diagnostic=sdf->Get<std::string>("diagnostic_output", "").first;
    if(sdf->HasElement("diagnostic_initial_body_velocity")) {
      this->initialBodyVelocity=sdf->Get<gz::math::Vector3d>(
        "diagnostic_initial_body_velocity");
      this->hasInitialBodyVelocity=true;
    }
    this->diagnosticDriveBodyForce=sdf->Get<gz::math::Vector3d>(
      "diagnostic_drive_body_force", gz::math::Vector3d::Zero).first;
    this->diagnosticDriveDuration=sdf->Get<double>(
      "diagnostic_drive_duration_s", 0.0).first;
    if(!diagnostic.empty()) {
      this->diagnostic.open(diagnostic, std::ios::out | std::ios::trunc);
      this->diagnostic << "time_s,velocity_body_x,velocity_body_y,angular_velocity_body_z,current_body_x,current_body_y,force_body_x,force_body_y,force_world_x,force_world_y,moment_body_z,added_mass_eigenvalue_min,added_mass_eigenvalue_max,added_mass_rr,added_mass_body_xx,added_mass_body_xy,added_mass_body_xr,added_mass_body_yy,added_mass_body_yr\n";
    }
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if(info.paused || this->link==gz::sim::kNullEntity) return;
    gz::sim::Link hull(this->link); const auto pose=hull.WorldPose(ecm); const auto velocity=hull.WorldLinearVelocity(ecm);
    const auto angularVelocity=hull.WorldAngularVelocity(ecm);
    if(!pose || !velocity || !angularVelocity) return;
    if(this->hasInitialBodyVelocity && !this->initialVelocityApplied) {
      ecm.CreateComponent(this->link,
        gz::sim::components::WorldLinearVelocityCmd(
          pose->Rot().RotateVector(this->initialBodyVelocity)));
      this->initialVelocityApplied=true;
      return;
    }
    const auto bodyVelocity=pose->Rot().Inverse().RotateVector(*velocity);
    const auto bodyAngularVelocity=pose->Rot().Inverse().RotateVector(*angularVelocity);
    const auto bodyCurrent=pose->Rot().Inverse().RotateVector(this->current);
    const auto damp=[](double v,double linear,double quadratic){return -(linear*v+quadratic*std::abs(v)*v);};
    const double relativeU=bodyVelocity.X()-bodyCurrent.X(), relativeV=bodyVelocity.Y()-bodyCurrent.Y();
    // SimpleHydrodynamics already applies damping at absolute hull velocity.
    // Add only the correction needed to obtain Node's damping(v-current).
    const gz::math::Vector3d correctionBody(
      damp(relativeU,this->xu,this->xuu)-damp(bodyVelocity.X(),this->xu,this->xuu),
      damp(relativeV,this->yv,this->yvv)-damp(bodyVelocity.Y(),this->yv,this->yvv),0);
    // Node's diagonal added-mass Coriolis model produces the current-relative
    // Munk moment (Y_vdot-X_udot)*u_r*v_r.  Apply only the current-induced
    // difference here, preserving the already-validated calm-water response.
    const double currentMomentZ=(this->yvDot-this->xuDot)*
      (relativeU*relativeV-bodyVelocity.X()*bodyVelocity.Y());
    const auto correctionWorld=pose->Rot().RotateVector(correctionBody);
    const double time=std::chrono::duration<double>(info.simTime).count();
    const auto driveBody=time<=this->diagnosticDriveDuration ?
      this->diagnosticDriveBodyForce : gz::math::Vector3d::Zero;
    hull.AddWorldWrench(ecm,correctionWorld+pose->Rot().RotateVector(driveBody),
      pose->Rot().RotateVector(gz::math::Vector3d(0,0,currentMomentZ)));
    if(this->diagnostic.is_open()) {
      const auto worldAddedMass=hull.WorldFluidAddedMassMatrix(ecm);
      double addedMassEigenvalueMin=0,addedMassEigenvalueMax=0,addedMassRr=0;
      double addedMassBodyXx=0,addedMassBodyXy=0,addedMassBodyXr=0;
      double addedMassBodyYy=0,addedMassBodyYr=0;
      if(worldAddedMass) {
        // Link::WorldFluidAddedMassMatrix exposes the SDF-order matrix from
        // gz-physics.  In gz-physics7's DART AddedMass feature the returned
        // coefficients remain expressed at the link origin; do not rotate
        // them a second time based on the link's current world yaw.
        addedMassBodyXx=(*worldAddedMass)(0,0);
        addedMassBodyXy=(*worldAddedMass)(0,1);
        addedMassBodyYy=(*worldAddedMass)(1,1);
        addedMassBodyXr=(*worldAddedMass)(0,5);
        addedMassBodyYr=(*worldAddedMass)(1,5);
        const double trace=addedMassBodyXx+addedMassBodyYy;
        const double discriminant=std::hypot(addedMassBodyXx-addedMassBodyYy,2*addedMassBodyXy);
        addedMassEigenvalueMin=0.5*(trace-discriminant);
        addedMassEigenvalueMax=0.5*(trace+discriminant);
        addedMassRr=(*worldAddedMass)(5,5);
      }
      this->diagnostic << time << ',' << bodyVelocity.X() << ',' << bodyVelocity.Y()
        << ',' << bodyAngularVelocity.Z()
        << ',' << bodyCurrent.X() << ',' << bodyCurrent.Y()
        << ',' << correctionBody.X() << ',' << correctionBody.Y()
        << ',' << correctionWorld.X() << ',' << correctionWorld.Y()
        << ',' << currentMomentZ << ',' << addedMassEigenvalueMin << ','
        << addedMassEigenvalueMax << ',' << addedMassRr << ','
        << addedMassBodyXx << ',' << addedMassBodyXy << ','
        << addedMassBodyXr << ',' << addedMassBodyYy << ','
        << addedMassBodyYr << '\n';
      this->diagnostic.flush();
    }
  }
 private:
  gz::sim::Entity link{gz::sim::kNullEntity}; gz::math::Vector3d current{0,0,0};
  double xu{6},xuu{18},yv{18},yvv{60};
  double xuDot{-2.615},yvDot{-39.225};
  gz::math::Vector3d initialBodyVelocity{0,0,0};
  gz::math::Vector3d diagnosticDriveBodyForce{0,0,0};
  double diagnosticDriveDuration{0};
  bool hasInitialBodyVelocity{false},initialVelocityApplied{false};
  std::ofstream diagnostic;
};
}
GZ_ADD_PLUGIN(vrx_surveyor::CurrentRelativeVelocity,gz::sim::System,
  vrx_surveyor::CurrentRelativeVelocity::ISystemConfigure,
  vrx_surveyor::CurrentRelativeVelocity::ISystemPreUpdate)
