#include <cmath>
#include <memory>
#include <string>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

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
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if(info.paused || this->link==gz::sim::kNullEntity) return;
    gz::sim::Link hull(this->link); const auto pose=hull.WorldPose(ecm); const auto velocity=hull.WorldLinearVelocity(ecm);
    if(!pose || !velocity) return;
    const auto bodyVelocity=pose->Rot().Inverse().RotateVector(*velocity);
    const auto bodyCurrent=pose->Rot().Inverse().RotateVector(this->current);
    const auto damp=[](double v,double linear,double quadratic){return -(linear*v+quadratic*std::abs(v)*v);};
    const double relativeU=bodyVelocity.X()-bodyCurrent.X(), relativeV=bodyVelocity.Y()-bodyCurrent.Y();
    // SimpleHydrodynamics already applies damping at absolute hull velocity.
    // Add only the correction needed to obtain Node's damping(v-current).
    const gz::math::Vector3d correctionBody(
      damp(relativeU,this->xu,this->xuu)-damp(bodyVelocity.X(),this->xu,this->xuu),
      damp(relativeV,this->yv,this->yvv)-damp(bodyVelocity.Y(),this->yv,this->yvv),0);
    hull.AddWorldWrench(ecm,pose->Rot().RotateVector(correctionBody),gz::math::Vector3d::Zero);
  }
 private:
  gz::sim::Entity link{gz::sim::kNullEntity}; gz::math::Vector3d current{0,0,0};
  double xu{6},xuu{18},yv{18},yvv{60};
};
}
GZ_ADD_PLUGIN(vrx_surveyor::CurrentRelativeVelocity,gz::sim::System,
  vrx_surveyor::CurrentRelativeVelocity::ISystemConfigure,
  vrx_surveyor::CurrentRelativeVelocity::ISystemPreUpdate)
