#include <cmath>
#include <memory>
#include <string>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace vrx_surveyor {
class SurveyorRelativeWind final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm,
      gz::sim::EventManager &) override {
    gz::sim::Model model(entity);
    this->link = model.LinkByName(ecm, sdf->Get<std::string>("link_name", "base_link").first);
    this->wind = sdf->Get<gz::math::Vector3d>("wind_enu", gz::math::Vector3d::Zero).first;
    this->rho = sdf->Get<double>("air_density", 1.225).first;
    this->frontalArea = sdf->Get<double>("frontal_area", 0.3822).first;
    this->sideArea = sdf->Get<double>("side_area", 0.7686).first;
    this->dragCoefficient = sdf->Get<double>("drag_coefficient", 1.1).first;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || this->link == gz::sim::kNullEntity) return;
    gz::sim::Link hull(this->link);
    const auto pose = hull.WorldPose(ecm);
    const auto velocity = hull.WorldLinearVelocity(ecm);
    if (!pose || !velocity) return;

    // Match Node WindLoad exactly: form air-relative velocity in the body
    // frame, use the Surveyor frontal / side projected areas there, then
    // rotate the resulting force back to Gazebo ENU world coordinates.
    const auto relativeBody = pose->Rot().Inverse().RotateVector(this->wind - *velocity);
    const double speed = std::hypot(relativeBody.X(), relativeBody.Y());
    const double scale = 0.5 * this->rho * this->dragCoefficient * speed;
    const gz::math::Vector3d forceBody(
        scale * this->frontalArea * relativeBody.X(),
        scale * this->sideArea * relativeBody.Y(), 0);
    hull.AddWorldWrench(ecm, pose->Rot().RotateVector(forceBody), gz::math::Vector3d::Zero);
  }

 private:
  gz::sim::Entity link{gz::sim::kNullEntity};
  gz::math::Vector3d wind{0, 0, 0};
  double rho{1.225};
  double frontalArea{0.3822};
  double sideArea{0.7686};
  double dragCoefficient{1.1};
};
}

GZ_ADD_PLUGIN(vrx_surveyor::SurveyorRelativeWind, gz::sim::System,
  vrx_surveyor::SurveyorRelativeWind::ISystemConfigure,
  vrx_surveyor::SurveyorRelativeWind::ISystemPreUpdate)
