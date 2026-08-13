import os
import tempfile
import unittest
from pathlib import Path

from generate import build_mesh, hydrostatics_comparison, transform_matrix, transform_vector, validate_config


class GeneratorContractTest(unittest.TestCase):
    def fixture(self):
        return {"schema_version": 1, "vehicle_id": "fixture", "mesh": {"path": "hull.gdf", "format": "gdf", "coordinate_system": "capytaine_x_forward_y_left_z_up", "center_of_mass_m": [0, 0, 0]}, "frequency_grid_rad_s": [0.5, 1.0], "wave_heading_grid_rad": [0.0], "water_density_kg_m3": 1025, "gravity_m_s2": 9.81, "water_depth_m": "infinite", "hydrostatics_comparison": {"reference_displaced_volume_m3": 2, "geometry_bootstrap_gm_transverse_m": 0.2, "geometry_bootstrap_gm_longitudinal_m": 0.5}}

    def test_validates_monotonic_frequency_grid(self):
        config = self.fixture()
        config["frequency_grid_rad_s"] = [1.0, 0.5]
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            validate_config(config, Path("."), require_mesh=False)

    def test_requires_explicit_solver_coordinate_convention(self):
        config = self.fixture()
        config["mesh"]["coordinate_system"] = "ned"
        with self.assertRaisesRegex(ValueError, "Capytaine"):
            validate_config(config, Path("."), require_mesh=False)

    def test_rejects_symlinked_mesh(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "real.gdf"
            target.write_text("fixture")
            (root / "hull.gdf").symlink_to(target)
            with self.assertRaisesRegex(ValueError, "regular file"):
                validate_config(self.fixture(), root)

    def test_accepts_explicit_parametric_bootstrap_mesh(self):
        config = self.fixture()
        config["mesh"] = {"generator": "parallelepiped", "coordinate_system": "capytaine_x_forward_y_left_z_up", "center_of_mass_m": [0, 0, 0], "size_m": [4, 1, .5], "center_m": [0, 0, -.25], "resolution": [8, 4, 2], "provenance": {"status": "bootstrap", "source": "vehicle geometry"}}
        validate_config(config, Path("."), require_mesh=False)
        os.environ.setdefault("CAPYTAINE_CACHE_DIR", str(Path(".cache/capytaine").resolve()))
        import capytaine as cpt
        mesh, checksum, provenance = build_mesh(config, Path("."), cpt)
        self.assertGreater(mesh.nb_faces, 0)
        self.assertEqual(len(checksum), 64)
        self.assertTrue(provenance["promotion_blocked"])

    def test_transforms_capytaine_axes_to_body_ned(self):
        diagonal = [[float(i == j) for j in range(6)] for i in range(6)]
        self.assertEqual(transform_matrix(diagonal), diagonal)
        coupling = [[0.0] * 6 for _ in range(6)]
        coupling[0][1] = coupling[1][0] = 2.0
        self.assertEqual(transform_matrix(coupling)[0][1], -2.0)
        self.assertEqual(transform_vector([1 + 2j] * 6)[1], [-1.0, -2.0])

    def test_reports_mesh_and_bootstrap_gm_side_by_side(self):
        config = self.fixture()
        stiffness = [[0.0] * 6 for _ in range(6)]
        denominator = 1025 * 9.81 * 2
        stiffness[3][3] = denominator * 0.3
        stiffness[4][4] = denominator * 0.45
        result = hydrostatics_comparison(config, stiffness)
        self.assertEqual(result["geometry_bootstrap"]["gm_transverse_m"], 0.2)
        self.assertAlmostEqual(result["capytaine_mesh_derived"]["gm_transverse_m"], 0.3)
        self.assertAlmostEqual(result["capytaine_minus_bootstrap"]["gm_longitudinal_m"], -0.05)
        self.assertEqual(result["resolution_policy"], "report-both-no-automatic-overwrite")

    def test_pinned_capytaine_api_is_available(self):
        os.environ.setdefault("CAPYTAINE_CACHE_DIR", str(Path(".cache/capytaine").resolve()))
        import capytaine as cpt
        self.assertGreaterEqual(tuple(map(int, cpt.__version__.split(".")[:2])), (2, 3))
        self.assertTrue(callable(cpt.load_mesh))
        self.assertTrue(callable(cpt.BEMSolver().solve_all))


if __name__ == "__main__":
    unittest.main()
