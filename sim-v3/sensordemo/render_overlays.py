from __future__ import annotations


def overlay_snapshot(*, environment: dict, detections: list, tracks: list, waypoint_ne, vessel_track_ne, static_geometry) -> dict:
    """Renderer-neutral overlay payload consumed by UI/video capture code."""
    return {"current_vectors": environment.get("current_vectors", []), "wave": environment.get("wave", {}), "bathymetry": environment.get("bathymetry", {}), "radar_detections": [d.__dict__ for d in detections if d.sensor == "radar"], "ais_detections": [d.__dict__ for d in detections if d.sensor == "ais"], "tracks": [{**t.__dict__, "coasted": environment.get("time_s", 0)-t.last_update_s > 0, "opacity": max(0., 1-(environment.get("time_s", 0)-t.last_update_s)/30)} for t in tracks], "safe_radius_m": environment.get("safe_radius_m", 30.), "commanded_waypoint_ne_m": waypoint_ne, "vessel_track_ne_m": vessel_track_ne, "enc_static_geometry": static_geometry}
